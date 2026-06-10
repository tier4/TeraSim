import importlib.util
import sys
import types
from pathlib import Path

import pytest


def load_carla_tools(monkeypatch):
    class Location:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Rotation:
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch = pitch
            self.yaw = yaw
            self.roll = roll

    class Transform:
        def __init__(self, location, rotation):
            self.location = location
            self.rotation = rotation

    monkeypatch.setitem(
        sys.modules,
        "carla",
        types.SimpleNamespace(Location=Location, Rotation=Rotation, Transform=Transform),
    )
    monkeypatch.setitem(sys.modules, "pyproj", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "utm", types.SimpleNamespace())

    module_path = (
        Path(__file__).resolve().parents[2]
        / "packages/terasim-service/terasim_service/utils/carla/tools.py"
    )
    spec = importlib.util.spec_from_file_location("carla_tools_under_test", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sumo_carla_round_trip_with_nonzero_odaiba_offset(monkeypatch):
    tools = load_carla_tools(monkeypatch)
    sumo_location = [89363.34867639952, 42400.23666565769, 0.0]
    sumo_rotation = [0.0, 237.23533630371094, 0.0]
    vehicle_shape = [4.5, 1.8, 1.5]
    offset = [86665.156, 45335.1, 0.0]

    carla_transform = tools.sumo_to_carla(
        sumo_location, sumo_rotation, vehicle_shape, offset
    )
    round_trip_location, round_trip_rotation = tools.carla_to_sumo(
        carla_transform.location, carla_transform.rotation, vehicle_shape, offset
    )

    assert round_trip_location == pytest.approx(sumo_location)
    assert round_trip_rotation == pytest.approx(sumo_rotation)
