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


class FakeVector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


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


class FakeAckermannControllerSettings:
    def __init__(self, speed_kp, speed_ki, speed_kd, accel_kp, accel_ki, accel_kd):
        self.speed_kp = speed_kp
        self.speed_ki = speed_ki
        self.speed_kd = speed_kd
        self.accel_kp = accel_kp
        self.accel_ki = accel_ki
        self.accel_kd = accel_kd


def install_fake_carla():
    sys.modules["carla"] = types.SimpleNamespace(
        Location=FakeLocation,
        Rotation=FakeRotation,
        Transform=FakeTransform,
        Vector3D=FakeVector3D,
        VehicleAckermannControl=FakeVehicleAckermannControl,
        AckermannControllerSettings=FakeAckermannControllerSettings,
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


def test_batched_lane_lookahead_matches_scalar_results():
    from terasim_service.utils.sumo_lane_geometry import (
        compile_lane_shapes,
        find_lookahead_position_from_lane_shapes,
        find_lookahead_positions_from_compiled_paths,
    )

    lane_shapes = [
        [(0.0, 0.0), (10.0, 0.0)],
        [(10.0, 0.0), (10.0, 10.0)],
    ]
    compiled = compile_lane_shapes(lane_shapes)
    positions = [(8.0, 0.0), (10.0, 2.0), (10.0, 9.0)]
    distances = [5.0, 5.0, 5.0]
    z_values = [1.0, 2.0, 3.0]

    expected = [
        find_lookahead_position_from_lane_shapes(
            lane_shapes, position, distance, z
        )
        for position, distance, z in zip(positions, distances, z_values)
    ]
    actual = find_lookahead_positions_from_compiled_paths(
        [compiled, compiled, compiled], positions, distances, z_values
    )

    for actual_point, expected_point in zip(actual, expected):
        assert actual_point == pytest.approx(expected_point)


def test_lookahead_route_cache_avoids_repeated_traci_calls(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    constants = types.SimpleNamespace(VAR_LANE_ID=1, VAR_NEXT_LINKS=2, VAR_ROUTE_ID=3)
    lane_shapes = {
        "edge_0_0": [(0.0, 0.0), (10.0, 0.0)],
        ":junction_0_0": [(10.0, 0.0), (12.0, 0.0)],
        "edge_1_0": [(12.0, 0.0), (30.0, 0.0)],
    }
    shape_calls = []
    next_link_calls = []
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: shape_calls.append(lane_id) or lane_shapes[lane_id]
    )
    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _vehicle_id: (_ for _ in ()).throw(
            AssertionError("context lane ID must be reused")
        ),
        getNextLinks=lambda vehicle_id: next_link_calls.append(vehicle_id) or [
            ("edge_1_0", ":junction_0_0", True, True, False, "G", "s", 2.0)
        ],
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            constants=constants,
            lane=fake_lane,
            vehicle=fake_vehicle,
        ),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.lookahead_path_cache = {}
    plugin.lookahead_lane_shape_cache = {}
    plugin.lookahead_geometry_cache = {}
    context_values = {
        constants.VAR_LANE_ID: "edge_0_0",
        constants.VAR_ROUTE_ID: "route_a",
    }

    first = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values=context_values
    )
    second = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values=context_values
    )
    context_values[constants.VAR_ROUTE_ID] = "route_b"
    after_reroute = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values=context_values
    )

    assert first is second
    assert first is after_reroute
    assert next_link_calls == ["AV", "AV"]
    assert shape_calls == ["edge_0_0", ":junction_0_0", "edge_1_0"]
    assert len(plugin.lookahead_geometry_cache) == 1


def test_route_aware_projection_switches_lane_after_centerline_midpoint():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [
        {"lane_id": "edge_0_0", "shape": [(0.0, 0.0), (100.0, 0.0)], "length": 80.0},
        {"lane_id": "edge_0_1", "shape": [(0.0, 3.2), (100.0, 3.2)], "length": 80.0},
    ]
    source_projection = select_route_aware_lane_projection(
        (25.0, 1.6),
        90.0,
        candidates,
        current_lane_id="edge_0_0",
        lane_switch_hysteresis=0.35,
    )
    target_projection = select_route_aware_lane_projection(
        (25.0, 2.0),
        90.0,
        candidates,
        current_lane_id="edge_0_0",
        lane_switch_hysteresis=0.35,
    )

    assert source_projection["lane_id"] == "edge_0_0"
    assert target_projection["lane_id"] == "edge_0_1"
    assert target_projection["lane_position"] == pytest.approx(20.0)


def test_route_aware_projection_can_preserve_sumo_lane_change_state():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [
        {"lane_id": "edge_0_0", "shape": [(0.0, 0.0), (100.0, 0.0)], "length": 100.0},
        {"lane_id": "edge_0_1", "shape": [(0.0, 3.2), (100.0, 3.2)], "length": 100.0},
    ]
    projection = select_route_aware_lane_projection(
        (25.0, 3.2), 90.0, candidates, current_lane_id="edge_0_0", prefer_current_lane=True
    )
    assert projection["lane_id"] == "edge_0_0"


def test_route_aware_projection_rejects_opposing_or_distant_lanes():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [{"lane_id": "opposing_0", "shape": [(100.0, 0.0), (0.0, 0.0)], "length": 100.0}]
    assert select_route_aware_lane_projection((20.0, 0.0), 90.0, candidates) is None
    assert (
        select_route_aware_lane_projection((20.0, 10.0), 270.0, candidates, max_distance=8.0)
        is None
    )


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


def test_ackermann_controller_settings_are_applied_once(capsys):
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannControllerTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    settings_applied = []
    physics_enabled = []
    actor = types.SimpleNamespace(
        set_simulate_physics=lambda enabled: physics_enabled.append(enabled),
        apply_ackermann_controller_settings=lambda settings: settings_applied.append(settings),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_controller_tuning = AckermannControllerTuning(accel_kp=0.05, accel_kd=0.005)

    cosim._ensure_ackermann_actor_physics(actor, "AV")
    cosim._ensure_ackermann_actor_physics(actor, "AV")

    assert physics_enabled == [True]
    assert len(settings_applied) == 1
    assert settings_applied[0].speed_kp == pytest.approx(0.15)
    assert settings_applied[0].accel_kp == pytest.approx(0.05)
    assert settings_applied[0].accel_kd == pytest.approx(0.005)
    assert cosim._ackermann_actor_state["AV"]["controller_settings_applied"] is True
    assert "CARLA Ackermann controller settings applied" in capsys.readouterr().out


def test_ackermann_physics_initializes_velocity_from_sumo_state():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannControllerTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    velocities = []
    actor = types.SimpleNamespace(
        set_simulate_physics=lambda enabled: None,
        set_target_velocity=velocities.append,
        apply_ackermann_controller_settings=lambda settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_controller_tuning = AckermannControllerTuning()
    transform = FakeTransform(rotation=FakeRotation(yaw=30.0))

    cosim._ensure_ackermann_actor_physics(actor, "BV", 4.0, transform)
    cosim._ensure_ackermann_actor_physics(actor, "BV", 9.0, transform)

    assert len(velocities) == 1
    assert velocities[0].x == pytest.approx(4.0 * 3**0.5 / 2.0)
    assert velocities[0].y == pytest.approx(2.0)
    assert velocities[0].z == pytest.approx(0.0)
    assert cosim._ackermann_actor_state["BV"]["initial_velocity_applied"] is True


def test_actor_radius_filter_uses_exit_hysteresis():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.actor_filter_enabled = True
    cosim.actor_filter_center_id = "AV"
    cosim.actor_filter_radius = 300.0
    cosim.actor_filter_hysteresis = 20.0
    cosim._actor_filter_active_vehicle_ids = set()
    cosim._actor_filter_missing_center_warned = False

    def vehicles_at(distance):
        return {
            "AV": {"x": 0.0, "y": 0.0},
            "BV": {"x": distance, "y": 0.0},
        }

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(299.0), {})
    assert set(filtered) == {"AV", "BV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(310.0), {})
    assert set(filtered) == {"AV", "BV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(321.0), {})
    assert set(filtered) == {"AV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(310.0), {})
    assert set(filtered) == {"AV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(299.0), {})
    assert set(filtered) == {"AV", "BV"}



def test_state_detail_radius_uses_physics_hysteresis():
    from terasim_service.plugins.cosim import TeraSimCoSimPlugin

    plugin = TeraSimCoSimPlugin.__new__(TeraSimCoSimPlugin)
    plugin.state_detail_filter_enabled = True
    plugin.state_detail_radius = 100.0
    plugin.state_detail_hysteresis = 10.0
    plugin.state_filter_center_id = "AV"
    plugin.state_detail_active_vehicle_ids = set()

    def selected_at(distance):
        positions = {
            "AV": (0.0, 0.0, 0.0),
            "BV": (distance, 0.0, 0.0),
        }
        return plugin._update_state_detail_active_vehicle_ids(positions, positions.copy())

    assert selected_at(99.0) == {"AV", "BV"}
    assert selected_at(105.0) == {"AV", "BV"}
    assert selected_at(111.0) == {"AV"}
    assert selected_at(105.0) == {"AV"}
    assert selected_at(99.0) == {"AV", "BV"}


def test_state_detail_filter_disabled_preserves_full_state_contract():
    from terasim_service.plugins.cosim import TeraSimCoSimPlugin

    plugin = TeraSimCoSimPlugin.__new__(TeraSimCoSimPlugin)
    plugin.state_detail_filter_enabled = False
    plugin.state_detail_active_vehicle_ids = {"stale"}

    selected = plugin._update_state_detail_active_vehicle_ids({"AV", "BV"}, {})

    assert selected == {"AV", "BV"}
    assert plugin.state_detail_active_vehicle_ids == {"AV", "BV"}


def _state_subscription_constants():
    return types.SimpleNamespace(
        VAR_DISTANCE=1,
        VAR_POSITION=2,
        VAR_POSITION3D=3,
        VAR_ANGLE=4,
        VAR_SPEED=5,
        VAR_ACCELERATION=6,
        CMD_GET_VEHICLE_VARIABLE=7,
        VAR_LANE_ID=8,
        VAR_LANEPOSITION=9,
        VAR_LANEPOSITION_LAT=10,
        VAR_NEXT_LINKS=11,
        VAR_ROUTE_ID=12,
    )


def _state_filter_plugin(plugin_module):
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.state_filter_enabled = True
    plugin.state_filter_radius = 320.0
    plugin.state_filter_center_id = "AV"
    plugin.state_filter_missing_center_logged = False
    plugin.state_filter_error_logged = False
    plugin.state_context_subscription_enabled = True
    plugin.state_context_subscription_active = False
    plugin.state_context_subscription_error_logged = False
    plugin.state_vehicle_context_results = {}
    plugin.logger = types.SimpleNamespace(warning=lambda *args, **kwargs: None)
    return plugin


def test_state_filter_uses_context_subscription_without_per_vehicle_position_queries(
    monkeypatch,
):
    from terasim_service.plugins import cosim as plugin_module

    constants = _state_subscription_constants()
    subscription_calls = []
    results = {
        "AV": {
            constants.VAR_POSITION3D: (0.0, 0.0, 0.0),
            constants.VAR_ANGLE: 90.0,
            constants.VAR_SPEED: 5.0,
            constants.VAR_ACCELERATION: 0.0,
        },
        "near": {
            constants.VAR_POSITION3D: (100.0, 0.0, 0.0),
            constants.VAR_ANGLE: 90.0,
            constants.VAR_SPEED: 4.0,
            constants.VAR_ACCELERATION: -1.0,
        },
    }
    fake_vehicle = types.SimpleNamespace(
        subscribeContext=lambda *args: subscription_calls.append(args),
        getContextSubscriptionResults=lambda _center_id: results,
        getPosition3D=lambda _vehicle_id: (_ for _ in ()).throw(
            AssertionError("per-vehicle position query must not be used")
        ),
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(constants=constants, vehicle=fake_vehicle),
    )
    plugin = _state_filter_plugin(plugin_module)

    selected, positions = plugin._filter_vehicle_ids_for_state(["AV", "near", "far"])

    assert selected == ["AV", "near"]
    assert positions == {
        "AV": (0.0, 0.0, 0.0),
        "near": (100.0, 0.0, 0.0),
    }
    assert subscription_calls == [
        (
            "AV",
            constants.CMD_GET_VEHICLE_VARIABLE,
            320.0,
            [
                constants.VAR_DISTANCE,
                constants.VAR_POSITION,
                constants.VAR_POSITION3D,
                constants.VAR_ANGLE,
                constants.VAR_SPEED,
                constants.VAR_ACCELERATION,
                constants.VAR_LANE_ID,
                constants.VAR_LANEPOSITION,
                constants.VAR_LANEPOSITION_LAT,
                constants.VAR_ROUTE_ID,
            ],
        )
    ]


def test_state_filter_falls_back_when_context_subscription_fails(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    constants = _state_subscription_constants()
    positions = {
        "AV": (0.0, 0.0, 0.0),
        "near": (100.0, 0.0, 0.0),
        "far": (400.0, 0.0, 0.0),
    }
    fake_vehicle = types.SimpleNamespace(
        subscribeContext=lambda *args: (_ for _ in ()).throw(RuntimeError("failed")),
        getContextSubscriptionResults=lambda _center_id: None,
        getPosition3D=lambda vehicle_id: positions[vehicle_id],
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(constants=constants, vehicle=fake_vehicle),
    )
    plugin = _state_filter_plugin(plugin_module)

    selected, position_cache = plugin._filter_vehicle_ids_for_state(
        ["AV", "near", "far"]
    )

    assert selected == ["AV", "near"]
    assert position_cache == positions
    assert plugin.state_context_subscription_error_logged is True


def test_detail_profile_helpers_skip_timing_when_profile_is_disabled(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    def unexpected_timer_call():
        raise AssertionError("perf_counter must not run while profiling is disabled")

    monkeypatch.setattr(plugin_module.time, "perf_counter", unexpected_timer_call)
    profile_ctx = {}

    assert plugin_module.TeraSimCoSimPlugin._profile_detail_traci_call(
        profile_ctx, "get_value", lambda value: value + 1, 2
    ) == 3
    assert plugin_module.TeraSimCoSimPlugin._profile_detail_python_call(
        profile_ctx, "geometry", lambda value: value * 2, 3
    ) == 6
    assert "cosim_profile" not in profile_ctx


def test_detail_profile_helpers_record_time_and_traci_count(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    timestamps = iter((1.0, 1.25, 2.0, 2.5))
    monkeypatch.setattr(plugin_module.time, "perf_counter", lambda: next(timestamps))
    profile_ctx = {"cosim_profile": {}}

    plugin_module.TeraSimCoSimPlugin._profile_detail_traci_call(
        profile_ctx, "get_value", lambda: "value"
    )
    plugin_module.TeraSimCoSimPlugin._profile_detail_python_call(
        profile_ctx, "geometry", lambda: "point"
    )

    breakdown = profile_ctx["cosim_profile"]["terasim_internal"]["state_export"][
        "ackermann_detail_breakdown"
    ]
    assert breakdown["traci"] == {
        "total_s": 0.25,
        "get_value_s": 0.25,
        "get_value_calls": 1.0,
    }
    assert breakdown["python"] == {
        "total_s": 0.5,
        "geometry_s": 0.5,
    }


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


def test_feedback_lc_keep_right_is_applied_once_and_verified(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []
    parameters = {}

    def set_parameter(actor_id, name, value):
        calls.append((actor_id, name, value))
        parameters[(actor_id, name)] = value

    fake_vehicle = types.SimpleNamespace(
        setParameter=set_parameter,
        getParameter=lambda actor_id, name: parameters[(actor_id, name)],
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.ackermann_feedback_lc_keep_right = 0.0
    plugin.ackermann_feedback_lc_keep_right_actor_ids = {"AV"}
    plugin.ackermann_feedback_lane_change_settings_applied = set()
    plugin.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)

    plugin._ensure_ackermann_feedback_lane_change_settings("AV")
    plugin._ensure_ackermann_feedback_lane_change_settings("AV")
    plugin._ensure_ackermann_feedback_lane_change_settings("BV")

    assert calls == [("AV", "laneChangeModel.lcKeepRight", "0")]
    assert plugin.ackermann_feedback_lane_change_settings_applied == {"AV"}


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
    lane_state = {
        "road_id": "edge_0",
        "lane_id": "edge_0_0",
        "lane_position": 10.0,
        "route_index": 0,
    }
    warnings = []

    def move_to_xy(*args):
        calls.append(("move", args))
        if args[2] == -1:
            lane_state.update(
                road_id="edge_0",
                lane_id="edge_0_1",
                lane_position=11.0,
                route_index=0,
            )

    fake_vehicle = types.SimpleNamespace(
        moveToXY=move_to_xy,
        setPreviousSpeed=lambda *args: calls.append(("speed", args)),
        getIDList=lambda: [],
        getRoadID=lambda _actor_id: lane_state["road_id"],
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getLanePosition=lambda _actor_id: lane_state["lane_position"],
        getRouteIndex=lambda _actor_id: lane_state["route_index"],
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
    plugin.ackermann_feedback_position_mode = "moveToXY"
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        warning=lambda message, *args, **kwargs: warnings.append(message % args),
        error=lambda *args, **kwargs: None,
    )

    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0] == ("move", ("AV", "", 0, 1.0, 2.0, 90.0, 0))
    assert plugin.feedback_observed_speeds == {"AV": 3.5}
    assert plugin.feedback_source_carla_frames == {"AV": 101}
    assert calls[-1] == ("speed", ("AV", 3.5))
    calls.clear()
    plugin.controlled_agents_each_step.clear()
    plugin.ackermann_feedback_lane_index = -1
    plugin.ackermann_feedback_keep_route = 1
    plugin.ackermann_feedback_log_lane_transitions = True
    plugin.feedback_lane_states = {}

    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0] == ("move", ("AV", "", -1, 1.0, 2.0, 90.0, 1))
    assert plugin.feedback_lane_states["AV"]["lane_id"] == "edge_0_1"
    assert any("source=feedback_moveToXY" in warning for warning in warnings)


def test_feedback_move_to_is_immediate_and_preserves_current_sumo_lane(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []
    command = types.SimpleNamespace(
        agent_id="BV",
        agent_type="vehicle",
        command_type="set_state",
        data={
            "position": [25.0, 2.8],
            "sumo_angle": 90.0,
            "speed": 3.5,
            "source_carla_frame": 101,
        },
    )
    lane_state = {
        "road_id": "edge_0",
        "lane_id": "edge_0_0",
        "lane_position": 10.0,
        "route_index": 0,
    }
    lane_shapes = {
        "edge_0_0": [(0.0, 0.0), (100.0, 0.0)],
        "edge_0_1": [(0.0, 3.2), (100.0, 3.2)],
        ":junction_0_0": [(100.0, 0.0), (105.0, 0.0)],
        "edge_1_0": [(105.0, 0.0), (205.0, 0.0)],
    }

    def move_to(actor_id, lane_id, lane_position):
        calls.append(("move", (actor_id, lane_id, lane_position)))
        lane_state.update(
            road_id=lane_id.rsplit("_", 1)[0],
            lane_id=lane_id,
            lane_position=lane_position,
        )

    fake_vehicle = types.SimpleNamespace(
        moveTo=move_to,
        moveToXY=lambda *args: calls.append(("move_xy", args)),
        setPreviousSpeed=lambda *args: calls.append(("speed", args)),
        getIDList=lambda: ["BV"],
        getRoadID=lambda _actor_id: lane_state["road_id"],
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getLanePosition=lambda _actor_id: lane_state["lane_position"],
        getRouteIndex=lambda _actor_id: lane_state["route_index"],
        getRoute=lambda _actor_id: ("edge_0", "edge_1"),
        getNextLinks=lambda _actor_id: [
            ("edge_1_0", ":junction_0_0", True, True, False, "G", "s", 5.0)
        ],
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: lane_shapes[lane_id],
        getLength=lambda lane_id: 5.0 if lane_id.startswith(":") else 100.0,
    )
    fake_edge = types.SimpleNamespace(
        getLaneNumber=lambda edge_id: 2 if edge_id == "edge_0" else 1,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane, edge=fake_edge),
    )
    monkeypatch.setattr(
        plugin_module.AgentCommand,
        "model_validate_json",
        staticmethod(lambda payload: command),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.controlled_agents_each_step = set()
    plugin.feedback_observed_speeds = {}
    plugin.feedback_source_carla_frames = {}
    plugin.feedback_lane_states = {}
    plugin.ackermann_feedback_position_mode = "moveTo"
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_log_lane_transitions = True
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0][0] == "move"
    assert calls[0][1][0:2] == ("BV", "edge_0_0")
    assert calls[0][1][2] == pytest.approx(25.0)
    assert calls[1] == ("speed", ("BV", 3.5))
    assert not any(call[0] == "move_xy" for call in calls)
    assert plugin.feedback_lane_states["BV"]["lane_id"] == "edge_0_0"

    # Even when CARLA is closer to the adjacent lane, moveTo must not switch
    # lanes on CARLA position. An invalid current-lane projection is rejected.
    calls.clear()
    plugin.controlled_agents_each_step.clear()
    command.data["position"] = [25.0, 10.0]
    assert plugin._handle_agent_command(b"{}") is False
    assert calls == []
    assert plugin.last_agent_command_failure["reason"] == (
        "ackermann_feedback_moveTo_mapping_failed"
    )

    # A wider background-only tolerance does not relax the AV limit.
    plugin.ackermann_feedback_background_move_to_max_distance = 20.0
    plugin.last_agent_command_failure = None
    plugin.controlled_agents_each_step.clear()
    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0][0] == "move"
    assert calls[0][1][0:2] == ("BV", "edge_0_0")
    assert calls[0][1][2] == pytest.approx(25.0)


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
    cosim._ackermann_actor_state = {}

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


def test_ackermann_feedback_uses_sumo_emergency_decel():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"AV"}
    cosim.ackermann_feedback_all_background_actors = False
    cosim.step_length = 0.1
    cosim.ackermann_feedback_speed_horizon = 0.1
    cosim.ackermann_tuning = AckermannTuning(max_accel=3.0, max_decel=6.0)
    cosim._ackermann_actor_state = {}

    target, acceleration = cosim._resolve_ackermann_longitudinal_target(
        "AV",
        {
            "speed": 1.0,
            "sumo_desired_speed": 0.0,
            "feedback_observed_speed": 1.0,
            "sumo_emergency_decel": 7.06,
        },
        current_speed=1.0,
    )

    assert acceleration == pytest.approx(-7.06)
    assert target == pytest.approx(0.294)


def test_fail_closed_brake_uses_last_sumo_emergency_decel():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    controls = []
    ticks = []
    actor = types.SimpleNamespace(apply_ackermann_control=controls.append)
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"AV"}
    cosim.ackermann_feedback_all_background_actors = False
    cosim._ackermann_feedback_actor_index = {"AV": actor}
    cosim._ackermann_actor_state = {"AV": {"steer": 0.2, "sumo_emergency_decel": 7.06}}
    cosim.ackermann_tuning = AckermannTuning(max_decel=6.0)
    cosim.step_length = 0.1
    cosim.args = types.SimpleNamespace(passive_tick=False)
    cosim.world = types.SimpleNamespace(tick=lambda: ticks.append("tick"))

    assert cosim._apply_ackermann_fail_closed_brake("test") == 1
    assert len(controls) == 1
    assert controls[0].speed == 0.0
    assert controls[0].acceleration == pytest.approx(-7.06)
    assert ticks == ["tick"]


def test_direct_command_failure_stops_before_sumo_step():
    from terasim_service.plugins import cosim_direct as direct_module

    plugin = direct_module.TeraSimCoSimDirectPlugin.__new__(direct_module.TeraSimCoSimDirectPlugin)
    plugin._lock = direct_module.threading.Lock()
    plugin._status = "wait_for_tick"
    plugin._state_json = ""
    plugin._completed_sumo_time = 0.0
    plugin._completed_tick_count = 0
    plugin._pending_commands = [b"invalid"]
    plugin._stop_requested = False
    plugin._tick_requested = direct_module.threading.Event()
    plugin._tick_requested.set()
    plugin._step_done = direct_module.threading.Event()
    plugin.controlled_agents_each_step = set()
    plugin.last_agent_command_failure = {
        "actor_id": "AV",
        "reason": "ackermann_feedback_moveTo_mapping_failed",
    }
    plugin.redis_client = None
    plugin._handle_agent_command = lambda raw: False
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
        critical=lambda *args, **kwargs: None,
    )
    simulator = types.SimpleNamespace(
        running=True,
        env=types.SimpleNamespace(record={}),
    )

    assert plugin.function_before_env_step(simulator, {}) is False
    assert simulator.running is False
    assert simulator.env.record["finish_reason"] == ("ackermann_feedback_moveTo_mapping_failed")
    assert plugin._status == "error"
    assert plugin._step_done.is_set()


def test_ackermann_control_trace_records_sumo_command_and_carla_response(capsys):
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_control_log_records = True
    cosim._ackermann_actor_state = {
        "AV": {
            "sumo_requested_acceleration": -7.06,
            "sumo_emergency_decel": 7.06,
        }
    }
    cosim.step_length = 0.1
    cosim.terasim_states = {"simulation_time": 12.3}
    cosim.world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=42))
    vehicle = types.SimpleNamespace(
        get_acceleration=lambda: types.SimpleNamespace(x=-6.5, y=0.0, z=0.0),
        get_control=lambda: types.SimpleNamespace(throttle=0.0, brake=0.75, steer=0.1),
    )
    transform = FakeTransform(rotation=FakeRotation(yaw=0.0))

    cosim._record_ackermann_control_trace(
        veh_id="AV",
        veh_info={
            "sumo_desired_speed": 0.0,
            "acceleration": -7.06,
            "feedback_observed_speed": 6.62,
        },
        vehicle=vehicle,
        current_transform=transform,
        current_speed=6.19,
        target_speed=0.0,
        target_acceleration=-7.06,
        position_error=0.4,
        feedback_unhealthy=False,
        target_behind=False,
    )

    prefix, payload = capsys.readouterr().out.strip().split(" ", 1)
    record = json.loads(payload)
    assert prefix == "AckermannControlTrace"
    assert record["sumo_requested_acceleration"] == pytest.approx(-7.06)
    assert record["ackermann_target_acceleration"] == pytest.approx(-7.06)
    assert record["carla_speed"] == pytest.approx(6.19)
    assert record["carla_longitudinal_acceleration"] == pytest.approx(-6.5)
    assert record["carla_applied_throttle"] == pytest.approx(0.0)
    assert record["carla_applied_brake"] == pytest.approx(0.75)
    assert record["carla_applied_steer"] == pytest.approx(0.1)
