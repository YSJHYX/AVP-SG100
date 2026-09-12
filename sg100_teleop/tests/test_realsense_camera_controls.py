"""Fixed production color-control contract for D405 and D455."""

from __future__ import annotations

import pytest

from sg100_teleop.camera.realsense_camera import RealSenseCamera


class _Options:
    enable_auto_exposure = "enable_auto_exposure"
    exposure = "exposure"
    gain = "gain"
    enable_auto_white_balance = "enable_auto_white_balance"


class _Rs:
    option = _Options()

    class stream:
        color = "color"


class _Profile:
    def stream_type(self):
        return _Rs.stream.color


class _ColorSensor:
    def __init__(self) -> None:
        self.values = {}
        self.writes = []

    def supports(self, option) -> bool:
        return True

    def set_option(self, option, value) -> None:
        self.values[option] = float(value)
        self.writes.append((option, float(value)))

    def get_option(self, option) -> float:
        return self.values[option]

    def get_stream_profiles(self):
        return [_Profile()]


class _Device:
    def __init__(self, sensor: _ColorSensor) -> None:
        self.sensor = sensor

    def query_sensors(self):
        return [self.sensor]


@pytest.mark.parametrize(
    ("model", "exposure_us", "gain"),
    (
        ("Intel RealSense D405", 7000.0, 16.0),
        ("RealSense D405", 7000.0, 16.0),
        ("Intel RealSense D455", 39.0, 5.0),
        ("RealSense D455", 39.0, 5.0),
    ),
)
def test_fixed_color_controls(model: str, exposure_us: float, gain: float) -> None:
    sensor = _ColorSensor()
    applied = RealSenseCamera._configure_fixed_color_controls(
        _Rs(), _Device(sensor), model
    )

    assert sensor.writes == [
        (_Options.enable_auto_exposure, 0.0),
        (_Options.exposure, exposure_us),
        (_Options.gain, gain),
        (_Options.enable_auto_white_balance, 1.0),
    ]
    assert applied == {
        "enable_auto_exposure": False,
        "exposure_us": exposure_us,
        "gain": gain,
        "enable_auto_white_balance": True,
        "white_balance_mode": "AUTO",
    }


def test_unknown_camera_model_has_no_silent_defaults() -> None:
    with pytest.raises(RuntimeError, match="没有为"):
        RealSenseCamera._configure_fixed_color_controls(
            _Rs(), _Device(_ColorSensor()), "Intel RealSense Unknown"
        )
