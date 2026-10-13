"""Offline verification of D455 reference capture and preview lifecycle."""

from __future__ import annotations

import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from sg100_teleop.camera import view_head_camera_reference as viewer
from sg100_teleop.camera.realsense_camera import RealSenseFrame


def fake_rs(devices):
    class Device:
        def __init__(self, serial, model):
            self.info = {"serial": serial, "name": model}

        def get_info(self, key):
            return self.info[key]

    return SimpleNamespace(
        camera_info=SimpleNamespace(serial_number="serial", name="name"),
        context=lambda: SimpleNamespace(
            query_devices=lambda: [Device(*item) for item in devices],
        ),
    )


def test_exact_inversion_and_overlay_do_not_change_camera_pixels():
    reference = np.array([[[0, 12, 255], [40, 100, 230]]], dtype=np.uint8)
    original = reference.copy()
    background = viewer.invert_rgb(reference)
    np.testing.assert_array_equal(background, [[[255, 243, 0], [215, 155, 25]]])
    np.testing.assert_array_equal(reference, original)
    np.testing.assert_array_equal(viewer.compose_preview(reference, background, 0), background)
    np.testing.assert_array_equal(viewer.compose_preview(reference, background, 1), original)
    np.testing.assert_array_equal(viewer.compose_preview(reference, background, 0.5), 128)
    moved = reference.copy()
    moved[0, 0, 0] = 100
    overlay = viewer.compose_preview(moved, background, 0.5)
    assert overlay[0, 0, 0] != 128
    np.testing.assert_array_equal(background, viewer.invert_rgb(original))


@pytest.mark.parametrize("alpha", [-0.1, 1.1, float("nan"), float("inf")])
def test_invalid_overlay_weight_is_rejected(alpha):
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="alpha"):
        viewer.compose_preview(image, image, alpha)


def test_d455_selection_ignores_wrist_camera_and_requires_unambiguous_head():
    rs = fake_rs([("wrist", "Intel RealSense D405"), ("head", "Intel RealSense D455")])
    assert viewer.select_d455(rs) == ("head", "Intel RealSense D455")
    with pytest.raises(RuntimeError, match="未找到 D455"):
        viewer.select_d455(rs, "wrist")
    multiple = fake_rs([("head", "Intel RealSense D455"), ("other", "RealSense D455")])
    with pytest.raises(RuntimeError, match="多台 D455"):
        viewer.select_d455(multiple)
    assert viewer.select_d455(multiple, "other") == ("other", "RealSense D455")
    with pytest.raises(RuntimeError, match="无 RealSense 设备"):
        viewer.select_d455(fake_rs([]))


class FakeCamera:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.started = False
        self.stopped = False
        self.last_error = ""
        self.reads = 0
        self.reference = np.full((2, 2, 3), [10, 20, 30], dtype=np.uint8)
        self.live = np.full((2, 2, 3), [25, 50, 75], dtype=np.uint8)

    def start(self):
        self.started = True

    def latest_before(self, timestamp):
        self.reads += 1
        return RealSenseFrame(
            role="head", host_receive_monotonic_ns=timestamp,
            device_timestamp_ms=123.0, frame_number=self.reads,
            color_rgb=(self.reference if self.reads == 1 else self.live).copy(),
            depth_uint16=None,
        )

    def metadata(self):
        return {
            "serial": "head", "model": "Intel RealSense D455",
            "color_stream_profile": "2x2@30:RGB8", "color_controls": {},
        }

    def stop(self):
        self.stopped = True


class FakeWindow:
    COLOR_RGB2BGR = cv2.COLOR_RGB2BGR
    WINDOW_NORMAL = cv2.WINDOW_NORMAL
    WND_PROP_VISIBLE = cv2.WND_PROP_VISIBLE
    cvtColor = staticmethod(cv2.cvtColor)
    imwrite = staticmethod(cv2.imwrite)

    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.frames = []
        self.keys = iter([ord("b"), ord("l"), ord("o"), ord("q")])
        self.destroyed = False

    def namedWindow(self, *args):
        # Both the photo and inverted background exist before preview opens.
        assert len(list(self.output_dir.glob("*.png"))) == 2

    def resizeWindow(self, *args):
        pass

    def createTrackbar(self, *args):
        pass

    def getTrackbarPos(self, *args):
        return 50

    def imshow(self, name, image):
        self.frames.append(image.copy())

    def waitKey(self, *args):
        return next(self.keys)

    def getWindowProperty(self, *args):
        return 1

    def destroyWindow(self, *args):
        self.destroyed = True


def run_fake(tmp_path, **overrides):
    args = viewer.build_arg_parser().parse_args([
        "--width", "2", "--height", "2", "--warmup-seconds", "0",
        "--output-dir", str(tmp_path),
    ])
    for key, value in overrides.items():
        setattr(args, key, value)
    cameras = []

    def factory(**kwargs):
        cameras.append(FakeCamera(**kwargs))
        return cameras[-1]

    return args, cameras, factory


def test_capture_saves_lossless_rgb_and_inverted_png_then_releases_camera(tmp_path):
    args, cameras, factory = run_fake(tmp_path, capture_only=True)
    paths = viewer.run_viewer(
        args, rs_module=fake_rs([("head", "Intel RealSense D455")]),
        cv2_module=cv2, camera_factory=factory,
    )
    camera = cameras[0]
    assert camera.started and camera.stopped
    assert camera.kwargs["enable_depth"] is False
    for path, expected in zip(paths[:2], (camera.reference, 255 - camera.reference)):
        actual_rgb = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        np.testing.assert_array_equal(actual_rgb, expected)
    metadata = json.loads(paths[2].read_text())
    assert metadata["serial"] == "head"
    assert metadata["photo"] == paths[0].name
    assert metadata["inverted_photo"] == paths[1].name
    assert metadata["frame_number"] == 1


def test_preview_uses_fixed_reference_and_switches_modes_then_cleans_up(tmp_path, monkeypatch):
    monkeypatch.setenv("DISPLAY", ":fake-display")
    args, cameras, factory = run_fake(tmp_path)
    window = FakeWindow(tmp_path)
    viewer.run_viewer(
        args, rs_module=fake_rs([("head", "Intel RealSense D455")]),
        cv2_module=window, camera_factory=factory,
    )
    camera = cameras[0]
    background = 255 - camera.reference
    expected_overlay = viewer.compose_preview(camera.live, background, 0.5)
    expected = (expected_overlay, background, camera.live, expected_overlay)
    for actual_bgr, expected_rgb in zip(window.frames, expected):
        np.testing.assert_array_equal(actual_bgr, expected_rgb[..., ::-1])
    assert len(window.frames) == 4
    assert camera.stopped and window.destroyed


def test_failed_snapshot_stops_camera_without_opening_window(tmp_path):
    args, cameras, factory = run_fake(tmp_path)
    window = FakeWindow(tmp_path)
    window.imwrite = lambda *_: False
    with pytest.raises(RuntimeError, match="照片保存失败"):
        viewer.run_viewer(
            args, rs_module=fake_rs([("head", "Intel RealSense D455")]),
            cv2_module=window, camera_factory=factory,
        )
    assert cameras[0].stopped
    assert not window.frames


def test_no_desktop_saves_reference_and_releases_camera(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    args, cameras, factory = run_fake(tmp_path)
    window = FakeWindow(tmp_path)
    with pytest.raises(RuntimeError, match="原图和反色图已保存"):
        viewer.run_viewer(
            args, rs_module=fake_rs([("head", "Intel RealSense D455")]),
            cv2_module=window, camera_factory=factory,
        )
    assert len(list(tmp_path.glob("*.png"))) == 2
    assert cameras[0].stopped
    assert not window.frames


def test_stale_frame_is_not_used_as_reference(monkeypatch):
    monkeypatch.setattr(viewer, "FRAME_TIMEOUT_S", 0.001)
    stale = SimpleNamespace(host_receive_monotonic_ns=0)
    camera = SimpleNamespace(latest_before=lambda _: stale, last_error="stale")
    with pytest.raises(RuntimeError, match="拍照超时"):
        viewer.wait_for_reference(camera, 0)
