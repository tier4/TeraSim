import importlib.util
import json
import sys
import types
from pathlib import Path


def load_carla_cosim(monkeypatch):
    module_path = (
        Path(__file__).resolve().parents[2]
        / "packages"
        / "terasim-service"
        / "terasim_service"
        / "utils"
        / "carla"
        / "cosim.py"
    )

    for package_name in (
        "terasim_service",
        "terasim_service.utils",
        "terasim_service.utils.carla",
    ):
        package = types.ModuleType(package_name)
        package.__path__ = []
        monkeypatch.setitem(sys.modules, package_name, package)

    monkeypatch.setitem(
        sys.modules,
        "carla",
        types.SimpleNamespace(
            TrafficLightState=types.SimpleNamespace(
                Green="green",
                Yellow="yellow",
                Red="red",
                Off="off",
            )
        ),
    )
    monkeypatch.setitem(
        sys.modules, "yaml", types.SimpleNamespace(safe_load=lambda file: {})
    )

    tool_names = [
        "carla_to_sumo",
        "create_bike_blueprint",
        "create_bikeandmotor_blueprint",
        "create_motor_blueprint",
        "create_pedestrian_blueprint",
        "create_police_car_blueprint",
        "create_vehicle_blueprint",
        "destroy_all_actors",
        "draw_text",
        "get_actor_id_from_attribute",
        "log_spawn_actor_failure",
        "sumo_to_carla",
        "spawn_actor",
    ]
    monkeypatch.setitem(
        sys.modules,
        "terasim_service.utils.carla.tools",
        types.SimpleNamespace(
            **{name: (lambda *args, **kwargs: None) for name in tool_names}
        ),
    )

    service_names = [
        "control_agent",
        "start_terasim",
        "stop_terasim",
        "tick_terasim",
        "get_terasim_status",
        "get_terasim_states",
    ]
    monkeypatch.setitem(
        sys.modules,
        "terasim_service.utils.service",
        types.SimpleNamespace(
            **{name: (lambda *args, **kwargs: None) for name in service_names}
        ),
    )

    module_name = "terasim_service.utils.carla.cosim"
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    monkeypatch.setitem(sys.modules, module_name, module)
    spec.loader.exec_module(module)
    return module


class FakeTrafficLight:
    type_id = "traffic.traffic_light"

    def __init__(self, actor_id, opendrive_id=None):
        self.id = actor_id
        self.opendrive_id = opendrive_id
        self.states = []

    def get_opendrive_id(self):
        return self.opendrive_id

    def set_state(self, state):
        self.states.append(state)


class FakeWorld:
    def __init__(self, actors_by_id):
        self.actors_by_id = actors_by_id

    def get_actor(self, actor_id):
        return self.actors_by_id.get(actor_id)


def make_cosim(module, world, traffic_lights):
    cosim = module.CarlaCosim.__new__(module.CarlaCosim)
    cosim.world = world
    cosim.opendrive_traffic_lights = module.CarlaCosim._build_opendrive_traffic_light_map(
        traffic_lights
    )
    return cosim


def test_tls_resolver_prefers_bare_actor_id_before_opendrive_fallback(monkeypatch):
    module = load_carla_cosim(monkeypatch)
    actor_84 = FakeTrafficLight(84, "466")
    actor_466 = FakeTrafficLight(900, "466")
    cosim = make_cosim(module, FakeWorld({84: actor_84}), [actor_466])

    assert cosim._resolve_traffic_light_actor("84")[0] is actor_84
    assert cosim._resolve_traffic_light_actor("od:466")[0] is actor_466
    assert cosim._resolve_traffic_light_actor("466")[0] is actor_466


def test_tls_resolver_normalizes_odaiba_opendrive_signal_ids(monkeypatch):
    module = load_carla_cosim(monkeypatch)
    actor_466 = FakeTrafficLight(900, "2000466")
    cosim = make_cosim(module, FakeWorld({}), [actor_466])

    assert cosim._resolve_traffic_light_actor("od:466")[0] is actor_466
    assert cosim._resolve_traffic_light_actor("od:2000466")[0] is actor_466
    assert cosim._resolve_traffic_light_actor("466")[0] is actor_466


def test_sync_cosim_tls_to_carla_resolves_actor_ids_and_opendrive_ids(monkeypatch):
    module = load_carla_cosim(monkeypatch)
    actor_84 = FakeTrafficLight(84, "84")
    actor_466 = FakeTrafficLight(900, "466")
    cosim = make_cosim(module, FakeWorld({84: actor_84}), [actor_466])
    cosim.args = types.SimpleNamespace(terasim_host="localhost", terasim_port=8000)
    cosim.terasim = {"simulation_id": "sim"}

    monkeypatch.setattr(
        module,
        "get_terasim_states",
        lambda *args: {
            "traffic_light_details": {
                "node": {
                    "tls": "Gr",
                    "information": json.dumps(
                        {
                            "programs": {
                                "0": {
                                    "parameters": {
                                        "linkSignalID:0": "84",
                                        "linkSignalID:1": "od:466",
                                    }
                                }
                            }
                        }
                    ),
                }
            }
        },
    )

    cosim.sync_cosim_tls_to_carla()

    assert actor_84.states == ["green"]
    assert actor_466.states == ["red"]
