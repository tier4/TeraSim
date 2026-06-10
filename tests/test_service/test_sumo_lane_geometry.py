import pytest

from terasim_service.utils.sumo_lane_geometry import reconstruct_position_from_lane_geometry


def test_reconstruct_position_on_straight_lane_centerline():
    position = reconstruct_position_from_lane_geometry(
        lane_shape=[(0.0, 0.0), (10.0, 0.0)],
        lane_position=4.0,
        lateral_offset=0.0,
        z=1.5,
    )

    assert position == pytest.approx((4.0, 0.0, 1.5))


def test_reconstruct_position_applies_left_lateral_offset():
    position = reconstruct_position_from_lane_geometry(
        lane_shape=[(0.0, 0.0), (10.0, 0.0)],
        lane_position=4.0,
        lateral_offset=1.25,
    )

    assert position == pytest.approx((4.0, 1.25, 0.0))


def test_reconstruct_position_follows_polyline_segment():
    position = reconstruct_position_from_lane_geometry(
        lane_shape=[(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)],
        lane_position=5.0,
        lateral_offset=0.5,
    )

    assert position == pytest.approx((2.5, 2.0, 0.0))


def test_reconstruct_position_returns_none_for_invalid_shape():
    assert reconstruct_position_from_lane_geometry([], 1.0, 0.0) is None
    assert reconstruct_position_from_lane_geometry([(0.0, 0.0)], 1.0, 0.0) is None
