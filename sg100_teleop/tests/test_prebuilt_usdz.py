import json
import tempfile
import threading
import zipfile
from pathlib import Path

from avp_stream.mujoco_msg import upload_xml
from avp_stream.streamer import VisionProStreamer


class DummyBody:
    def __init__(self, name):
        self.name = name


class DummyModel:
    nbody = 1

    def body(self, index):
        assert index == 0
        return DummyBody("world")


class CachedHitChannel:
    readyState = "open"

    def __init__(self, streamer):
        self.streamer = streamer
        self.messages = []

    def send(self, message):
        self.messages.append(message)

        if isinstance(message, str):
            metadata = json.loads(message)
            assert metadata["type"] == "metadata"

            # Respond after the transfer method has initialized its
            # cache-response flags.
            def mark_cached():
                self.streamer._usdz_cached_hit = True
                self.streamer._usdz_transfer_complete = True

            threading.Timer(0.01, mark_cached).start()


def make_valid_usdz(path: Path) -> Path:
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(
            "scene.usda",
            "#usda 1.0\ndef Xform \"Scene\" {}\n",
        )

    return path


def bare_streamer():
    streamer = VisionProStreamer.__new__(VisionProStreamer)
    streamer.verbose = False
    streamer._log = lambda *args, **kwargs: None
    return streamer


def assert_raises(exception_type, expected_text, callback):
    try:
        callback()
    except exception_type as exc:
        assert expected_text in str(exc)
    else:
        raise AssertionError(
            f"Expected {exception_type.__name__}: {expected_text}"
        )


def test_validation_and_configure(temp_dir: Path):
    valid_path = make_valid_usdz(temp_dir / "scene.USDZ")
    resolved = VisionProStreamer._validate_prebuilt_usdz(
        str(valid_path)
    )
    assert resolved == str(valid_path.resolve())

    streamer = bare_streamer()
    streamer.set_origin = lambda origin: None
    streamer.configure_mujoco(
        xml_path="scene.xml",
        model=DummyModel(),
        data=object(),
        usdz_path=str(valid_path),
    )
    assert streamer._sim_config["usdz_path"] == resolved


def test_validation_failures(temp_dir: Path):
    missing = temp_dir / "missing.usdz"
    assert_raises(
        FileNotFoundError,
        "does not exist",
        lambda: VisionProStreamer._validate_prebuilt_usdz(missing),
    )

    empty = temp_dir / "empty.usdz"
    empty.touch()
    assert_raises(
        RuntimeError,
        "is empty",
        lambda: VisionProStreamer._validate_prebuilt_usdz(empty),
    )

    wrong_suffix = make_valid_usdz(temp_dir / "scene.zip")
    assert_raises(
        RuntimeError,
        ".usdz suffix",
        lambda: VisionProStreamer._validate_prebuilt_usdz(
            wrong_suffix
        ),
    )

    corrupted = temp_dir / "corrupted.usdz"
    corrupted.write_bytes(b"this is not a ZIP archive")
    assert_raises(
        RuntimeError,
        "not a readable ZIP archive",
        lambda: VisionProStreamer._validate_prebuilt_usdz(corrupted),
    )


def test_prebuilt_local_bypasses_all_conversion(temp_dir: Path):
    valid_path = make_valid_usdz(temp_dir / "local.usdz")
    streamer = bare_streamer()
    streamer._sim_config = {
        "xml_path": str(temp_dir / "unused.xml"),
        "force_reload": True,
        "usdz_path": str(valid_path),
    }
    calls = []
    streamer._send_usdz_data = (
        lambda path, attach, port: calls.append(
            (path, attach, port)
        ) or True
    )
    streamer._convert_to_usdz = lambda *args, **kwargs: (
        (_ for _ in ()).throw(
            AssertionError("local converter was called")
        )
    )

    external_original = upload_xml.convert_and_download
    upload_xml.convert_and_download = lambda *args, **kwargs: (
        (_ for _ in ()).throw(
            AssertionError("external converter was called")
        )
    )

    try:
        result = streamer._load_and_send_mujoco_scene(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            50051,
        )
    finally:
        upload_xml.convert_and_download = external_original

    assert result is True
    assert calls == [
        (
            str(valid_path.resolve()),
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            50051,
        )
    ]


def test_prebuilt_cross_network_bypasses_conversion(temp_dir: Path):
    valid_path = make_valid_usdz(temp_dir / "cross.usdz")
    streamer = bare_streamer()
    streamer._sim_config = {
        "xml_path": str(temp_dir / "unused.xml"),
        "force_reload": False,
        "usdz_path": str(valid_path),
    }
    streamer._webrtc_loop = None
    streamer._convert_to_usdz = lambda *args, **kwargs: (
        (_ for _ in ()).throw(
            AssertionError("cross-network converter was called")
        )
    )
    streamer._webrtc_usdz_channel = CachedHitChannel(streamer)

    external_original = upload_xml.convert_and_download
    upload_xml.convert_and_download = lambda *args, **kwargs: (
        (_ for _ in ()).throw(
            AssertionError("external converter was called")
        )
    )

    try:
        result = streamer._load_and_send_scene_webrtc(None)
    finally:
        upload_xml.convert_and_download = external_original

    assert result is True
    metadata = json.loads(
        streamer._webrtc_usdz_channel.messages[0]
    )
    assert metadata["filename"] == valid_path.name


def test_unspecified_path_preserves_conversion(temp_dir: Path):
    xml_path = temp_dir / "scene.xml"
    xml_path.write_text("<mujoco/>", encoding="utf-8")
    generated = make_valid_usdz(temp_dir / "generated.usdz")
    streamer = bare_streamer()
    streamer._sim_config = {
        "xml_path": str(xml_path),
        "force_reload": True,
        "usdz_path": None,
    }
    conversion_calls = []
    send_calls = []
    streamer._convert_to_usdz = (
        lambda xml, output_path=None: conversion_calls.append(
            (xml, output_path)
        ) or str(generated)
    )
    streamer._send_usdz_data = (
        lambda path, attach, port: send_calls.append(
            (path, attach, port)
        ) or True
    )

    assert streamer._load_and_send_mujoco_scene(None, 50051)
    assert len(conversion_calls) == 1
    assert send_calls == [(str(generated), None, 50051)]


def test_explicit_send_failure_stops_before_pose_streaming(
    temp_dir: Path,
):
    valid_path = make_valid_usdz(temp_dir / "send_failure.usdz")
    streamer = bare_streamer()
    streamer._cross_network_mode = False
    streamer._sim_config = {
        "xml_path": str(temp_dir / "unused.xml"),
        "attach_to": None,
        "grpc_port": 50051,
        "force_reload": False,
        "usdz_path": str(valid_path),
    }
    pose_started = []
    streamer._send_usdz_data = lambda *args, **kwargs: False
    streamer._start_pose_streaming = lambda: pose_started.append(True)

    assert_raises(
        RuntimeError,
        "Failed to send explicitly configured prebuilt USDZ",
        lambda: streamer.serve(),
    )
    assert not pose_started
    assert not hasattr(streamer, "_server_running")


def test_invalid_explicit_path_stops_runner_sequence(
    temp_dir: Path,
):
    streamer = bare_streamer()
    start_calls = []
    streamer.start_webrtc = lambda: start_calls.append(True)

    try:
        streamer.configure_mujoco(
            xml_path="scene.xml",
            model=DummyModel(),
            data=object(),
            usdz_path=str(temp_dir / "missing_runner.usdz"),
        )
        streamer.start_webrtc()
    except FileNotFoundError as exc:
        assert "does not exist" in str(exc)
    else:
        raise AssertionError(
            "Invalid explicit USDZ did not stop runner startup"
        )

    assert not start_calls
    assert not hasattr(streamer, "_sim_config")


def main():
    with tempfile.TemporaryDirectory() as directory:
        temp_dir = Path(directory)
        test_validation_and_configure(temp_dir)
        test_validation_failures(temp_dir)
        test_prebuilt_local_bypasses_all_conversion(temp_dir)
        test_prebuilt_cross_network_bypasses_conversion(temp_dir)
        test_unspecified_path_preserves_conversion(temp_dir)
        test_explicit_send_failure_stops_before_pose_streaming(temp_dir)
        test_invalid_explicit_path_stops_runner_sequence(temp_dir)

    print("PREBUILT USDZ: PASS")


if __name__ == "__main__":
    main()
