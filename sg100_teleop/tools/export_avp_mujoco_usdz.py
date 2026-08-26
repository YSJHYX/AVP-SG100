"""Export a MuJoCo scene for the official Tracking Streamer client.

The official MuJoCo USD converter authors an articulated, nested USD scene.
That is useful as an intermediate representation, but the current
``CombinedStreamingView`` recursively drives every visual leaf below a body.
This tool rewrites the converter output so every streamed MuJoCo body is a
direct child of ``Geometry`` and owns only its own default-purpose visuals.

Run this module in the repository's USD environment::

    .venv_usdz/bin/python -m sg100_teleop.tools.export_avp_mujoco_usdz \
        --xml scene.xml --output scene_compat.usdz
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
import tempfile
from typing import Iterable, Sequence
import zipfile


PHYSICS_SCHEMA_PREFIXES = ("Physics", "Physx", "Newton", "Mjc")
PHYSICS_PROPERTY_PREFIXES = ("physics:", "physx", "newton:", "mjc:")
ROBOT_WORLD_GEOM_MARKERS = ("rm65", "sg100", "robot", "link")
# This plane is the MuJoCo renderer's dark background treatment, not physical
# workstation geometry.  RealityKit owns its environment/background, so
# exporting the infinite plane would create a duplicate black floor in AVP.
EXCLUDED_WORLD_VISUALS = frozenset({"workstation_floor_visual"})
MATERIAL_PROTOTYPE_PREFIX = "Flattened_Prototype_"


@dataclass
class VisualRecord:
    body: str | None
    source_path: str
    destination_path: str
    purpose: str
    transform_error: float = 0.0
    bbox_extent_ratio: list[float] = field(default_factory=list)


@dataclass
class ExportAudit:
    xml_path: str
    intermediate_asset: str
    output_path: str
    runtime_body_names: list[str]
    body_paths: dict[str, str]
    kept_renderables: list[VisualRecord]
    skipped_guides: list[str]
    skipped_physics_only: list[str]
    excluded_world_renderables: list[str]
    world_static_renderables: list[str]
    ownership_collisions: dict[str, list[str]]
    nested_runtime_bodies: list[tuple[str, str]]
    physics_prims: list[str]

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)


def _require_dependencies():
    try:
        import mujoco
        import mujoco_usd_converter
        from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdUtils
    except ImportError as exc:
        raise RuntimeError(
            "AVP USDZ export requires mujoco-usd-converter and pxr. "
            "Run with .venv_usdz/bin/python."
        ) from exc
    return (
        mujoco,
        mujoco_usd_converter,
        Gf,
        Sdf,
        Tf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdUtils,
    )


def sanitize_name(value: str) -> str:
    """Match Tracking Streamer's lowercase-alphanumeric sanitization."""

    return "".join(character.lower() for character in value if character.isalnum())


def runtime_body_names(model) -> list[str]:
    """Return every named non-world body sent by ``configure_mujoco``."""

    return [model.body(index).name for index in range(1, model.nbody) if model.body(index).name]


def _is_body_candidate(prim) -> bool:
    from pxr import UsdGeom

    return prim.IsA(UsdGeom.Xform)


def _match_runtime_bodies(stage, body_names: Sequence[str]):
    """Resolve every MuJoCo body to one USD Xform, failing on ambiguity."""

    candidates = [prim for prim in stage.Traverse() if _is_body_candidate(prim)]
    result = {}
    used_paths = set()
    for body_name in body_names:
        wanted = sanitize_name(body_name)
        ranked = []
        for prim in candidates:
            actual = sanitize_name(prim.GetName())
            if wanted and wanted in actual:
                ranked.append((len(actual) - len(wanted), len(prim.GetName()), str(prim.GetPath()), prim))
        if not ranked:
            raise RuntimeError(f"No USD Xform matches runtime body {body_name!r}")
        ranked.sort(key=lambda item: item[:3])
        best_score = ranked[0][:2]
        tied = [item for item in ranked if item[:2] == best_score]
        if len(tied) != 1:
            paths = ", ".join(item[2] for item in tied)
            raise RuntimeError(f"Ambiguous USD Xform match for {body_name!r}: {paths}")
        match = tied[0][3]
        path = str(match.GetPath())
        if path in used_paths:
            raise RuntimeError(f"USD Xform {path} matched more than one runtime body")
        used_paths.add(path)
        result[body_name] = match
    return result


def _find_geometry_scope(stage):
    from pxr import UsdGeom

    root = stage.GetDefaultPrim()
    if not root:
        raise RuntimeError("Intermediate USD has no default prim")
    geometry = root.GetChild("Geometry")
    if not geometry or not geometry.IsA(UsdGeom.Scope):
        raise RuntimeError(f"Intermediate USD has no Geometry scope below {root.GetPath()}")
    return geometry


def _effective_purpose(prim) -> str:
    from pxr import UsdGeom

    return str(UsdGeom.Imageable(prim).ComputePurpose())


def _iter_owned_gprims(body_prim, runtime_paths: set[str]):
    """Yield a body's Gprims, stopping before nested runtime bodies."""

    from pxr import UsdGeom

    def visit(prim):
        if str(prim.GetPath()) in runtime_paths and prim != body_prim:
            return
        if prim.IsA(UsdGeom.Gprim):
            yield prim
            return
        for child in prim.GetChildren():
            yield from visit(child)

    for child in body_prim.GetChildren():
        yield from visit(child)


def _nearest_runtime_owner(prim, runtime_path_to_name: dict[str, str]) -> str | None:
    parent = prim.GetParent()
    while parent:
        owner = runtime_path_to_name.get(str(parent.GetPath()))
        if owner is not None:
            return owner
        parent = parent.GetParent()
    return None


def _make_unique_child_name(parent_path, requested: str, occupied: set[str], Tf) -> str:
    base = Tf.MakeValidIdentifier(requested) or "visual"
    candidate = base
    suffix = 1
    while str(parent_path.AppendChild(candidate)) in occupied:
        candidate = f"{base}_{suffix}"
        suffix += 1
    occupied.add(str(parent_path.AppendChild(candidate)))
    return candidate


def _strip_physics(prim) -> None:
    """Remove physics/simulator APIs and properties but retain the Gprim."""

    from pxr import Sdf

    kept_schemas = [
        schema
        for schema in prim.GetAppliedSchemas()
        if not schema.startswith(PHYSICS_SCHEMA_PREFIXES)
    ]
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(kept_schemas))
    for prop in list(prim.GetProperties()):
        lowered = prop.GetName().lower()
        if lowered.startswith(PHYSICS_PROPERTY_PREFIXES):
            prim.RemoveProperty(prop.GetName())


def _matrix_max_error(lhs, rhs) -> float:
    return max(abs(float(lhs[row][column]) - float(rhs[row][column])) for row in range(4) for column in range(4))


def _extent(prim, time_code, UsdGeom) -> list[float]:
    cache = UsdGeom.BBoxCache(time_code, [UsdGeom.Tokens.default_], useExtentsHint=False)
    value_range = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    size = value_range.GetSize()
    return [float(size[0]), float(size[1]), float(size[2])]


def _extent_ratio(source: Iterable[float], destination: Iterable[float]) -> list[float]:
    ratios = []
    for before, after in zip(source, destination):
        if abs(before) <= 1e-12:
            ratios.append(1.0 if abs(after) <= 1e-12 else math.inf)
        else:
            ratios.append(after / before)
    return ratios


def _looks_like_robot_world_geom(name: str) -> bool:
    lowered = name.lower()
    return any(marker in lowered for marker in ROBOT_WORLD_GEOM_MARKERS)


def _copy_materials(flat_layer, source_root, output_layer, destination_root, Sdf) -> None:
    """Copy material bindings and their flattened instance prototypes.

    ``UsdStage.Flatten`` keeps instanced converter materials as internal
    references to root-level ``Flattened_Prototype_*`` specs.  Copying only
    the Materials scope produces a package that opens with unresolved prim
    warnings.  Keep those material-network prototypes at their authored root
    paths so the internal references remain valid in the compatibility stage.
    """

    source_materials = source_root.GetChild("Materials")
    if not source_materials:
        return
    for root_prim in flat_layer.rootPrims:
        if root_prim.name.startswith(MATERIAL_PROTOTYPE_PREFIX):
            prototype_path = Sdf.Path.absoluteRootPath.AppendChild(root_prim.name)
            Sdf.CopySpec(flat_layer, prototype_path, output_layer, prototype_path)
    destination_path = destination_root.GetPath().AppendChild("Materials")
    Sdf.CopySpec(flat_layer, source_materials.GetPath(), output_layer, destination_path)


def _package_stage(stage_path: Path, output_path: Path, UsdUtils, Sdf) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        prefix=f".{output_path.stem}.", suffix=".usdz", dir=output_path.parent, delete=False
    ) as handle:
        temporary_package = Path(handle.name)
    temporary_package.unlink()
    try:
        if not UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(stage_path)), str(temporary_package)):
            raise RuntimeError(f"USDZ packaging failed for {stage_path}")
        os.replace(temporary_package, output_path)
    finally:
        if temporary_package.exists():
            temporary_package.unlink()


def validate_usdz_package(usdz_path: str | Path) -> dict[str, object]:
    """Validate archive integrity and packaged USD asset references."""

    *_prefix, Sdf, _Tf, Usd, _UsdGeom, _UsdPhysics, UsdUtils = _require_dependencies()
    usdz_path = Path(usdz_path).expanduser().resolve()
    if not usdz_path.is_file() or not zipfile.is_zipfile(usdz_path):
        raise RuntimeError(f"Invalid USDZ archive: {usdz_path}")
    with zipfile.ZipFile(usdz_path) as archive:
        corrupt_member = archive.testzip()
        members = archive.namelist()
    if corrupt_member is not None:
        raise RuntimeError(f"Corrupt USDZ member: {corrupt_member}")
    if not members:
        raise RuntimeError(f"Empty USDZ archive: {usdz_path}")
    if any(Path(member).is_absolute() or ".." in Path(member).parts for member in members):
        raise RuntimeError("USDZ contains an unsafe absolute/parent archive path")

    stage = Usd.Stage.Open(str(usdz_path))
    if stage is None or not stage.GetDefaultPrim():
        raise RuntimeError(f"USDZ stage cannot be reopened: {usdz_path}")
    _layers, resolved_assets, unresolved_assets = UsdUtils.ComputeAllDependencies(
        Sdf.AssetPath(str(usdz_path))
    )
    if unresolved_assets:
        raise RuntimeError(f"USDZ has unresolved assets: {unresolved_assets}")

    absolute_authored_assets = []
    for prim in stage.TraverseAll():
        for attribute in prim.GetAttributes():
            value = attribute.Get()
            values = value if isinstance(value, (list, tuple)) else (value,)
            for item in values:
                if isinstance(item, Sdf.AssetPath) and Path(item.path).is_absolute():
                    absolute_authored_assets.append(
                        f"{prim.GetPath()}.{attribute.GetName()}={item.path}"
                    )
    if absolute_authored_assets:
        raise RuntimeError(
            "USDZ contains absolute authored assets: "
            + ", ".join(absolute_authored_assets)
        )
    return {
        "members": members,
        "resolved_assets": [str(asset) for asset in resolved_assets],
        "unresolved_assets": [],
    }


def audit_compat_stage(stage, body_names: Sequence[str]) -> tuple[dict[str, str], dict[str, list[str]], list[tuple[str, str]], list[str]]:
    """Simulate CombinedStreamingView leaf targeting on a compatibility stage."""

    from pxr import Usd, UsdGeom, UsdPhysics

    body_prims = _match_runtime_bodies(stage, body_names)
    body_paths = {name: str(prim.GetPath()) for name, prim in body_prims.items()}
    path_items = sorted(body_paths.items())
    nested = []
    for body_a, path_a in path_items:
        for body_b, path_b in path_items:
            if body_a != body_b and path_b.startswith(path_a + "/"):
                nested.append((body_a, body_b))

    leaf_owners: dict[str, list[str]] = {}
    for body_name, body_prim in body_prims.items():
        for prim in Usd.PrimRange(body_prim):
            if prim == body_prim:
                continue
            model_children = [child for child in prim.GetChildren() if child.IsA(UsdGeom.Xformable)]
            if prim.IsA(UsdGeom.Gprim) and not model_children:
                leaf_owners.setdefault(str(prim.GetPath()), []).append(body_name)
    collisions = {path: owners for path, owners in leaf_owners.items() if len(owners) != 1}

    physics_prims = []
    for prim in stage.Traverse():
        schemas = prim.GetAppliedSchemas()
        has_physics_schema = any(schema.startswith(PHYSICS_SCHEMA_PREFIXES) for schema in schemas)
        has_physics_property = any(
            prop.GetName().lower().startswith(PHYSICS_PROPERTY_PREFIXES)
            for prop in prim.GetProperties()
        )
        if prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.Scene) or has_physics_schema or has_physics_property:
            physics_prims.append(str(prim.GetPath()))
    return body_paths, collisions, nested, physics_prims


def export_compat_usdz(
    xml_path: str | Path,
    output_path: str | Path,
    *,
    keep_intermediate: bool = False,
    intermediate_dir: str | Path | None = None,
    verbose: bool = False,
) -> ExportAudit:
    """Convert MJCF, flatten runtime visual ownership, and package USDZ."""

    (
        mujoco,
        mujoco_usd_converter,
        Gf,
        Sdf,
        Tf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdUtils,
    ) = _require_dependencies()

    xml_path = Path(xml_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    if not xml_path.is_file():
        raise ValueError(f"MJCF file does not exist: {xml_path}")
    if output_path.suffix.lower() != ".usdz":
        raise ValueError(f"Output must use the .usdz suffix: {output_path}")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    body_names = runtime_body_names(model)

    owned_temporary_dir = None
    if intermediate_dir is None:
        if keep_intermediate:
            intermediate_path = Path(
                tempfile.mkdtemp(prefix=f"{output_path.stem}_intermediate_", dir=output_path.parent)
            )
        else:
            owned_temporary_dir = tempfile.TemporaryDirectory(prefix="avp_mujoco_usd_")
            intermediate_path = Path(owned_temporary_dir.name)
    else:
        intermediate_path = Path(intermediate_dir).expanduser().resolve()
        intermediate_path.mkdir(parents=True, exist_ok=True)

    try:
        converter = mujoco_usd_converter.Converter()
        intermediate_asset = Path(converter.convert(str(xml_path), str(intermediate_path)).path)
        source_stage = Usd.Stage.Open(str(intermediate_asset))
        if source_stage is None:
            raise RuntimeError(f"Could not open converter output: {intermediate_asset}")
        source_geometry = _find_geometry_scope(source_stage)
        source_body_prims = _match_runtime_bodies(source_stage, body_names)
        runtime_paths = {str(prim.GetPath()) for prim in source_body_prims.values()}
        runtime_path_to_name = {str(prim.GetPath()): name for name, prim in source_body_prims.items()}

        skipped_guides = []
        skipped_physics_only = []
        world_gprims = []
        excluded_world_renderables = []
        for prim in Usd.PrimRange(source_geometry):
            if prim == source_geometry or not prim.IsA(UsdGeom.Gprim):
                continue
            purpose = _effective_purpose(prim)
            owner = _nearest_runtime_owner(prim, runtime_path_to_name)
            if purpose != str(UsdGeom.Tokens.default_):
                skipped_guides.append(str(prim.GetPath()))
            elif owner is None:
                if prim.GetName() in EXCLUDED_WORLD_VISUALS:
                    excluded_world_renderables.append(str(prim.GetPath()))
                else:
                    world_gprims.append(prim)

        for prim in source_stage.Traverse():
            if prim.IsA(UsdGeom.Gprim):
                continue
            path = str(prim.GetPath())
            type_name = prim.GetTypeName()
            if (
                prim.IsA(UsdPhysics.Joint)
                or prim.IsA(UsdPhysics.Scene)
                or type_name.startswith(("Physics", "Mjc", "Newton"))
                or "/Physics/" in path
            ):
                skipped_physics_only.append(path)

        bad_world = [str(prim.GetPath()) for prim in world_gprims if _looks_like_robot_world_geom(prim.GetName())]
        if bad_world:
            raise RuntimeError(
                "Robot-like default-purpose geometry remains outside a runtime body: "
                + ", ".join(bad_world)
            )

        flat_layer = source_stage.Flatten()
        flat_stage = Usd.Stage.Open(flat_layer)
        if flat_stage is None:
            raise RuntimeError("Could not open flattened converter stage")
        flat_root = flat_stage.GetDefaultPrim()
        flat_geometry = _find_geometry_scope(flat_stage)
        flat_body_prims = _match_runtime_bodies(flat_stage, body_names)

        compat_stage_path = intermediate_path / "avp_compat.usdc"
        output_stage = Usd.Stage.CreateNew(str(compat_stage_path))
        source_root = source_stage.GetDefaultPrim()
        root_name = source_root.GetName()
        destination_root = UsdGeom.Xform.Define(output_stage, Sdf.Path.absoluteRootPath.AppendChild(root_name)).GetPrim()
        output_stage.SetDefaultPrim(destination_root)
        output_stage.SetMetadata("upAxis", "Z")
        output_stage.SetMetadata("metersPerUnit", 1.0)
        output_stage.SetMetadata(
            "customLayerData",
            {"creator": "VisionProTeleop AVP compatibility exporter"},
        )
        destination_geometry = UsdGeom.Scope.Define(
            output_stage, destination_root.GetPath().AppendChild("Geometry")
        ).GetPrim()
        _copy_materials(flat_layer, flat_root, output_stage.GetRootLayer(), destination_root, Sdf)

        source_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        flat_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        occupied_paths: set[str] = set()
        visual_records: list[VisualRecord] = []

        destination_body_paths = {}
        for body_name in body_names:
            source_body = source_body_prims[body_name]
            flat_body = flat_body_prims[body_name]
            valid_body_name = Tf.MakeValidIdentifier(body_name)
            destination_body_path = destination_geometry.GetPath().AppendChild(valid_body_name)
            destination_body = UsdGeom.Xform.Define(output_stage, destination_body_path).GetPrim()
            destination_body.SetDisplayName(body_name)
            body_relative, _ = source_cache.ComputeRelativeTransform(source_body, source_geometry)
            UsdGeom.Xformable(destination_body).AddTransformOp().Set(body_relative)
            destination_body_paths[body_name] = str(destination_body_path)

            flat_runtime_paths = {str(prim.GetPath()) for prim in flat_body_prims.values()}
            owned = list(_iter_owned_gprims(flat_body, flat_runtime_paths))
            kept = [prim for prim in owned if _effective_purpose(prim) == str(UsdGeom.Tokens.default_)]
            if not kept:
                skipped_physics_only.append(str(source_body.GetPath()))
            for source_visual in kept:
                child_name = _make_unique_child_name(
                    destination_body_path, source_visual.GetName(), occupied_paths, Tf
                )
                destination_visual_path = destination_body_path.AppendChild(child_name)
                Sdf.CopySpec(
                    flat_layer,
                    source_visual.GetPath(),
                    output_stage.GetRootLayer(),
                    destination_visual_path,
                )
                destination_visual = output_stage.GetPrimAtPath(destination_visual_path)
                visual_relative, _ = flat_cache.ComputeRelativeTransform(source_visual, flat_body)
                destination_xformable = UsdGeom.Xformable(destination_visual)
                destination_xformable.ClearXformOpOrder()
                destination_xformable.AddTransformOp().Set(visual_relative)
                _strip_physics(destination_visual)
                visual_records.append(
                    VisualRecord(
                        body=body_name,
                        source_path=str(source_visual.GetPath()),
                        destination_path=str(destination_visual_path),
                        purpose=str(UsdGeom.Tokens.default_),
                    )
                )

        for source_visual in world_gprims:
            child_name = _make_unique_child_name(
                destination_geometry.GetPath(), source_visual.GetName(), occupied_paths, Tf
            )
            destination_visual_path = destination_geometry.GetPath().AppendChild(child_name)
            flat_visual = flat_stage.GetPrimAtPath(source_visual.GetPath())
            Sdf.CopySpec(
                flat_layer,
                flat_visual.GetPath(),
                output_stage.GetRootLayer(),
                destination_visual_path,
            )
            destination_visual = output_stage.GetPrimAtPath(destination_visual_path)
            visual_relative, _ = flat_cache.ComputeRelativeTransform(flat_visual, flat_geometry)
            destination_xformable = UsdGeom.Xformable(destination_visual)
            destination_xformable.ClearXformOpOrder()
            destination_xformable.AddTransformOp().Set(visual_relative)
            _strip_physics(destination_visual)
            visual_records.append(
                VisualRecord(
                    body=None,
                    source_path=str(source_visual.GetPath()),
                    destination_path=str(destination_visual_path),
                    purpose=str(UsdGeom.Tokens.default_),
                )
            )

        output_stage.GetRootLayer().Save()
        output_stage.Reload()
        output_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for record in visual_records:
            source_prim = flat_stage.GetPrimAtPath(record.source_path)
            destination_prim = output_stage.GetPrimAtPath(record.destination_path)
            source_world = flat_cache.GetLocalToWorldTransform(source_prim)
            destination_world = output_cache.GetLocalToWorldTransform(destination_prim)
            record.transform_error = _matrix_max_error(source_world, destination_world)
            record.bbox_extent_ratio = _extent_ratio(
                _extent(source_prim, Usd.TimeCode.Default(), UsdGeom),
                _extent(destination_prim, Usd.TimeCode.Default(), UsdGeom),
            )

        body_paths, ownership_collisions, nested, physics_prims = audit_compat_stage(
            output_stage, body_names
        )
        if ownership_collisions:
            raise RuntimeError(f"Compatibility ownership collisions: {ownership_collisions}")
        if nested:
            raise RuntimeError(f"Nested runtime bodies remain: {nested}")
        if physics_prims:
            raise RuntimeError(f"Physics schemas remain in compatibility stage: {physics_prims}")
        if any(record.transform_error > 1e-8 for record in visual_records):
            raise RuntimeError("Visual world transform changed during compatibility rewrite")

        _package_stage(compat_stage_path, output_path, UsdUtils, Sdf)
        validate_usdz_package(output_path)
        audit = ExportAudit(
            xml_path=str(xml_path),
            intermediate_asset=str(intermediate_asset),
            output_path=str(output_path),
            runtime_body_names=body_names,
            body_paths=body_paths,
            kept_renderables=visual_records,
            skipped_guides=skipped_guides,
            skipped_physics_only=skipped_physics_only,
            excluded_world_renderables=excluded_world_renderables,
            world_static_renderables=[str(prim.GetPath()) for prim in world_gprims],
            ownership_collisions=ownership_collisions,
            nested_runtime_bodies=nested,
            physics_prims=physics_prims,
        )
        if verbose:
            print(audit.to_json())
        return audit
    finally:
        if owned_temporary_dir is not None:
            owned_temporary_dir.cleanup()


def print_hierarchy(usdz_path: str | Path) -> None:
    """Print the compact Root/Geometry/body/visual tree."""

    *_unused, Usd, _UsdGeom, _UsdPhysics, _UsdUtils = _require_dependencies()[3:]
    stage = Usd.Stage.Open(str(usdz_path))
    root = stage.GetDefaultPrim()

    def visit(prim, depth=0):
        print(f"{'  ' * depth}{prim.GetName()} [{prim.GetTypeName()}]")
        for child in prim.GetChildren():
            visit(child, depth + 1)

    visit(root)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export a flattened visualization USDZ for official Tracking Streamer"
    )
    parser.add_argument("--xml", required=True, help="Input MuJoCo XML scene")
    parser.add_argument("--output", required=True, help="Output compatibility .usdz")
    parser.add_argument(
        "--keep-intermediate", action="store_true", help="Keep converter and rewritten USD files"
    )
    parser.add_argument("--verbose", action="store_true", help="Print the full JSON audit")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    audit = export_compat_usdz(
        args.xml,
        args.output,
        keep_intermediate=args.keep_intermediate,
        verbose=args.verbose,
    )
    print(f"Generated: {audit.output_path}")
    print(f"Runtime bodies: {len(audit.runtime_body_names)}")
    print(f"Runtime body targets: {len(audit.body_paths)}")
    print(f"Visual leaves: {len(audit.kept_renderables)}")
    print(f"Ownership collisions: {len(audit.ownership_collisions)}")
    print(f"Nested runtime bodies: {len(audit.nested_runtime_bodies)}")
    print(f"World static renderables: {len(audit.world_static_renderables)}")
    print_hierarchy(audit.output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
