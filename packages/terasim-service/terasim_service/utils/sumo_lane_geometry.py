import math


def reconstruct_position_from_lane_geometry(
    lane_shape,
    lane_position: float,
    lateral_offset: float,
    z: float = 0.0,
):
    """Reconstruct a SUMO x/y/z position from lane-relative coordinates."""
    if not lane_shape or len(lane_shape) < 2:
        return None

    try:
        points = [(float(point[0]), float(point[1])) for point in lane_shape]
        target_distance = max(0.0, float(lane_position))
        lateral_offset = float(lateral_offset)
        z = float(z)
    except (TypeError, ValueError, IndexError):
        return None

    travelled = 0.0
    last_segment = None
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 0.0:
            continue
        last_segment = (start, dx, dy, segment_length)
        if travelled + segment_length >= target_distance:
            ratio = (target_distance - travelled) / segment_length
            center_x = start[0] + dx * ratio
            center_y = start[1] + dy * ratio
            normal_x = -dy / segment_length
            normal_y = dx / segment_length
            return (
                center_x + normal_x * lateral_offset,
                center_y + normal_y * lateral_offset,
                z,
            )
        travelled += segment_length

    if last_segment is None:
        return None

    start, dx, dy, segment_length = last_segment
    end_x = start[0] + dx
    end_y = start[1] + dy
    normal_x = -dy / segment_length
    normal_y = dx / segment_length
    return (
        end_x + normal_x * lateral_offset,
        end_y + normal_y * lateral_offset,
        z,
    )
