from pathlib import Path
import tempfile

import numpy as np
import yaml

from sg100_teleop.tracking.avp_to_tcp_config import (
    load_avp_to_tcp_config,
    validate_avp_to_tcp_local_rotation,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "sg100_teleop/config/avp_to_tcp_identity.yaml"

SIGNED_PERMUTATION = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def expect_invalid(value, expected_text):
    try:
        validate_avp_to_tcp_local_rotation(value)
    except ValueError as exc:
        if expected_text not in str(exc):
            raise AssertionError(f"Unexpected validation error: {exc}") from exc
    else:
        raise AssertionError("Invalid basis was accepted")


def test_matrix_validation():
    np.testing.assert_allclose(
        validate_avp_to_tcp_local_rotation(SIGNED_PERMUTATION),
        SIGNED_PERMUTATION,
    )
    expect_invalid(np.eye(2), "shape")
    bad_finite = np.eye(3)
    bad_finite[0, 0] = np.nan
    expect_invalid(bad_finite, "finite")
    expect_invalid(np.diag([2.0, 1.0, 1.0]), "orthonormal")
    shear = np.eye(3)
    shear[0, 1] = 0.1
    expect_invalid(shear, "orthonormal")
    expect_invalid(np.diag([-1.0, 1.0, 1.0]), "determinant +1")


def test_default_identity_config_is_explicitly_uncalibrated():
    config = load_avp_to_tcp_config(DEFAULT_CONFIG)
    np.testing.assert_allclose(config.avp_to_tcp_local_rotation, np.eye(3))
    assert config.translation_scale == 1.5
    assert "uncalibrated development default" in config.source


def test_yaml_loads_nonidentity_proper_basis():
    data = {
        "schema_version": 1,
        "source": "synthetic signed-permutation test only",
        "translation_scale": 0.25,
        "avp_to_tcp_local_rotation": SIGNED_PERMUTATION.tolist(),
    }
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "basis.yaml"
        with path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream)
        config = load_avp_to_tcp_config(path)
    assert config.translation_scale == 0.25
    np.testing.assert_allclose(
        config.avp_to_tcp_local_rotation,
        SIGNED_PERMUTATION,
    )


def main():
    test_matrix_validation()
    test_default_identity_config_is_explicitly_uncalibrated()
    test_yaml_loads_nonidentity_proper_basis()
    print("AVP-TO-TCP CONFIG VALIDATION: PASS")


if __name__ == "__main__":
    main()
