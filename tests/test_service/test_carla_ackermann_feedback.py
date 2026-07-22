import json
import sys
import types

import pytest


class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeRotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class FakeTransform:
    def __init__(self, location=None, rotation=None):
        self.location = location or FakeLocation()
        self.rotation = rotation or FakeRotation()


class FakeCommand:
    FutureActor = object()

    @staticmethod
    def ApplyTransform(actor_id, transform):
        return ("transform", actor_id, transform)

    @staticmethod
    def ApplyVehicleAckermannControl(actor_id, control):
        return ("ackermann", actor_id, control)

    @staticmethod
    def SpawnActor(blueprint, transform):
        return types.SimpleNamespace(then=lambda command: ("spawn", blueprint, transform, command))

    @staticmethod
    def SetSimulatePhysics(actor_id, enabled):
        return ("simulate_physics", actor_id, enabled)


class FakeVehicleAckermannControl:
    def __init__(self, steer=0.0, speed=0.0, acceleration=0.0, jerk=0.0):
        self.steer = steer
        self.speed = speed
        self.acceleration = acceleration
        self.jerk = jerk


def install_fake_carla():
    sys.modules["carla"] = types.SimpleNamespace(
        Location=FakeLocation,
        Rotation=FakeRotation,
        Transform=FakeTransform,
        VehicleAckermannControl=FakeVehicleAckermannControl,
        command=FakeCommand,
    )


def test_lane_lookahead_crosses_internal_and_destination_lanes():
    from terasim_service.utils.sumo_lane_geometry import (
        extract_next_link_lane_ids,
        find_lookahead_position_from_lane_shapes,
    )

    next_links = [
        ("outgoing_0", ":junction_0_0", True, True, False, "G", "s", 12.5),
    ]
    assert extract_next_link_lane_ids(next_links) == [":junction_0_0", "outgoing_0"]
    point = find_lookahead_position_from_lane_shapes(
        [[(0.0, 0.0), (10.0, 0.0)], [(10.0, 0.0), (20.0, 0.0)]],
        (8.0, 0.0),
        7.0,
        1.5,
    )
    assert point == pytest.approx((15.0, 0.0, 1.5))


def test_pure_pursuit_steers_and_rate_limits():
    from terasim_service.utils.carla.ackermann_control import (
        AckermannTuning,
        compute_ackermann_control_values,
    )

    tuning = AckermannTuning(max_steer_rad=0.6, max_steer_rate_rad_s=0.1)
    values = compute_ackermann_control_values(
        current_x=0.0,
        current_y=0.0,
        yaw_degrees=0.0,
        current_speed=0.0,
        desired_x=10.0,
        desired_y=0.0,
        lookahead_x=10.0,
        lookahead_y=5.0,
        desired_speed=5.0,
        previous_steer=0.0,
        dt=0.1,
        tuning=tuning,
    )
    assert values.steer == pytest.approx(0.01)
    assert values.acceleration == pytest.approx(3.0)


def test_feedback_wildcard_excludes_av_unless_explicit():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_actor_ids = {"*"}
    cosim.ackermann_feedback_all_background_actors = True
    assert cosim._is_ackermann_feedback_selected_actor("BV") is True
    assert cosim._is_ackermann_feedback_selected_actor("AV") is False

    cosim.ackermann_feedback_actor_ids.add("AV")
    assert cosim._is_ackermann_feedback_selected_actor("AV") is True


def test_feedback_apply_enables_physics_only_for_selected_actors():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_physics_enabled = True
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"*"}
    cosim.ackermann_feedback_all_background_actors = True
    assert cosim._uses_ackermann_physics("BV") is True
    assert cosim._uses_ackermann_physics("AV") is False


def test_feedback_batches_valid_actors_and_isolates_invalid_shape(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module
    from terasim_service.utils.carla.cosim import CarlaCosim

    actor = types.SimpleNamespace(
        get_transform=lambda: FakeTransform(FakeLocation(10.0, 20.0, 3.0), FakeRotation()),
        get_velocity=lambda: types.SimpleNamespace(x=3.0, y=4.0, z=12.0),
    )
    batches = []
    monkeypatch.setattr(
        cosim_module,
        "control_agents_batch",
        lambda host, port, simulation_id, commands: (
            batches.append(commands) or {"queued_count": len(commands)}
        ),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=101))
    cosim.args = types.SimpleNamespace(terasim_host="terasim", terasim_port=8000)
    cosim.terasim = {"simulation_id": "simulation"}
    cosim.terasim_states = {
        "simulation_time": 5.0,
        "agent_details": {
            "vehicle": {
                "good": {"length": 4.0, "width": 2.0, "height": 1.5},
                "bad": {"length": 0.0, "width": 2.0, "height": 1.5},
            }
        },
    }
    cosim.ackermann_feedback_mode = "apply"
    cosim.ackermann_feedback_shadow_enabled = False
    cosim._ackermann_feedback_state = {}
    cosim._ackermann_feedback_candidate_actor_ids = {"good", "bad"}
    cosim._ackermann_feedback_actor_index = {"good": actor, "bad": actor}
    cosim._coord_transformer = None
    cosim.sumo_carla_offset = [0.0, 0.0]

    assert cosim.sync_carla_ackermann_feedback_to_cosim() is False
    assert [[command["agent_id"] for command in batch] for batch in batches] == [["good"]]
    command = batches[0][0]
    assert command["data"]["position"] == pytest.approx([12.0, -20.0])
    assert command["data"]["speed"] == pytest.approx(5.0)
    assert command["data"]["source_carla_frame"] == 101
    assert cosim._ackermann_feedback_state["good"]["feedback_status"] == "queued"
    assert (
        cosim._ackermann_feedback_state["bad"]["feedback_reason"] == "sumo_shape_missing_or_invalid"
    )


def test_http_feedback_tick_orders_feedback_before_next_sumo_step(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module
    from terasim_service.utils.carla.cosim import CarlaCosim

    events = []
    monkeypatch.setattr(
        cosim_module,
        "tick_terasim",
        lambda *args: events.append("request_sumo_tick"),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.args = types.SimpleNamespace(skip_tls=True, terasim_host="terasim", terasim_port=8000)
    cosim.terasim = {"simulation_id": "simulation"}
    cosim._wait_for_terasim_step = lambda: "ticked"
    cosim.sync_cosim_actor_to_carla = lambda: events.append("apply_sumo_state")
    cosim.world = types.SimpleNamespace(tick=lambda: events.append("carla_tick"))
    cosim.sync_carla_ackermann_feedback_to_cosim = lambda: events.append("queue_feedback")

    assert cosim._tick_ackermann_feedback_apply_http() is True
    assert events == [
        "apply_sumo_state",
        "carla_tick",
        "queue_feedback",
        "request_sumo_tick",
    ]


def test_grpc_feedback_is_attached_to_tick_after_carla_frame(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    direct_link_module = types.SimpleNamespace(
        parse_state_json=lambda value: json.loads(value) if value else None
    )
    monkeypatch.setitem(
        sys.modules,
        "terasim_service.utils.carla.direct_link",
        direct_link_module,
    )
    monkeypatch.setitem(sys.modules, "grpc", types.SimpleNamespace(RpcError=Exception))

    events = []
    command = {
        "agent_id": "BV",
        "agent_type": "vehicle",
        "command_type": "set_state",
        "data": {"source_carla_frame": 7},
    }
    record = {
        "actor_id": "BV",
        "source_carla_frame": 7,
        "feedback_status": "rejected",
    }
    future = object()
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._direct_tick_future = None
    cosim._direct_prev_state = {"agent_details": {"vehicle": {}, "vru": {}}}
    cosim.args = types.SimpleNamespace(skip_tls=True)
    cosim.sync_cosim_actor_to_carla = lambda state: events.append("apply_sumo_state")
    cosim.world = types.SimpleNamespace(tick=lambda: events.append("carla_tick"))
    cosim._collect_ackermann_feedback = lambda: (
        events.append("collect_feedback") or ([command], [record])
    )
    cosim.direct_link = types.SimpleNamespace(
        tick_async=lambda commands: (events.append(("grpc_tick", commands)) or future)
    )
    cosim._ackermann_feedback_state = {}
    cosim._record_ackermann_feedback = lambda feedback: events.append(("record", feedback.copy()))

    assert cosim._tick_ackermann_feedback_apply_direct() is True
    assert cosim._direct_tick_future is future
    assert events[0:3] == [
        "apply_sumo_state",
        "carla_tick",
        "collect_feedback",
    ]
    assert events[3] == ("grpc_tick", [command])
    assert events[4][1]["feedback_status"] == "queued"
    assert events[4][1]["feedback_reason"] == "accepted_by_grpc_tick"


def test_feedback_wait_requires_new_completed_tick(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module
    from terasim_service.utils.carla.cosim import CarlaCosim

    responses = iter(
        [
            {"status": "ticked", "completed_tick_count": 7},
            {"status": "running", "completed_tick_count": 7},
            {"status": "ticked", "completed_tick_count": 8},
        ]
    )
    monkeypatch.setattr(cosim_module, "get_terasim_status", lambda *args: next(responses))
    monkeypatch.setattr(cosim_module.time, "sleep", lambda seconds: None)

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.args = types.SimpleNamespace(terasim_host="terasim", terasim_port=8000)
    cosim.terasim = {"simulation_id": "simulation"}
    cosim._initial_terasim_state_pending = False
    cosim._last_completed_terasim_tick_count = 7

    assert cosim._wait_for_terasim_step() == "ticked"
    assert cosim._last_completed_terasim_tick_count == 8


def test_feedback_ack_is_cached_by_shared_sumo_command_handler(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []
    command = types.SimpleNamespace(
        agent_id="AV",
        agent_type="vehicle",
        command_type="set_state",
        data={
            "position": [1.0, 2.0],
            "sumo_angle": 90.0,
            "speed": 3.5,
            "source_carla_frame": 101,
        },
    )
    fake_vehicle = types.SimpleNamespace(
        moveToXY=lambda *args: calls.append(("move", args)),
        setPreviousSpeed=lambda *args: calls.append(("speed", args)),
        getIDList=lambda: [],
    )
    monkeypatch.setattr(plugin_module, "traci", types.SimpleNamespace(vehicle=fake_vehicle))
    monkeypatch.setattr(
        plugin_module.AgentCommand,
        "model_validate_json",
        staticmethod(lambda payload: command),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.controlled_agents_each_step = set()
    plugin.feedback_observed_speeds = {}
    plugin.feedback_source_carla_frames = {}
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    assert plugin._handle_agent_command(b"{}") is True
    assert plugin.feedback_observed_speeds == {"AV": 3.5}
    assert plugin.feedback_source_carla_frames == {"AV": 101}
    assert calls[-1] == ("speed", ("AV", 3.5))


def test_ackermann_feedback_acceleration_uses_observed_speed():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"AV"}
    cosim.ackermann_feedback_all_background_actors = False
    cosim.step_length = 0.1
    cosim.ackermann_feedback_speed_horizon = 1.0
    cosim.ackermann_tuning = AckermannTuning(max_accel=3.0, max_decel=6.0)

    target, acceleration = cosim._resolve_ackermann_longitudinal_target(
        "AV",
        {
            "speed": 3.2,
            "sumo_desired_speed": 1.2,
            "feedback_observed_speed": 1.0,
        },
        current_speed=0.4,
    )
    assert acceleration == pytest.approx(2.0)
    assert target == pytest.approx(2.4)
