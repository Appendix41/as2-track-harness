"""Monocular ground projection: an image route becomes body-frame metres.

Uses the declared camera mount (fixed intrinsics, height, pitch) and a flat
ground. The output is a direction and an odometry-consistent progress measure,
not a depth sensor and not a standoff measurement.
"""
import math

from .contracts import number


def load_intrinsics(mount):
    """Return validated colour intrinsics from config/camera_mount.json."""
    if not isinstance(mount, dict):
        raise ValueError('invalid_camera_mount')
    values = mount.get('intrinsics')
    if not isinstance(values, dict):
        raise ValueError('camera_intrinsics_missing: add intrinsics to config/camera_mount.json')
    out = {k: values.get(k) for k in ('fx', 'fy', 'ppx', 'ppy', 'width', 'height')}
    if (not all(number(v) for v in out.values())
            or not all(out[k] > 0 for k in ('fx', 'fy', 'width', 'height'))
            or not 0 <= out['ppx'] <= out['width'] or not 0 <= out['ppy'] <= out['height']):
        raise ValueError('invalid_camera_intrinsics')
    height_m, pitch_rad = mount.get('height_m', mount.get('height_hint_m')), mount.get('pitch_rad', 0.)
    if not number(height_m) or not .05 <= height_m <= 3.:
        raise ValueError('invalid_camera_height')
    if not number(pitch_rad) or abs(pitch_rad) > 1.2:
        raise ValueError('invalid_camera_pitch')
    out.update(height_m=float(height_m), pitch_rad=float(pitch_rad))
    return out


def image_point_to_local(u, v, intrinsics):
    """Project one normalised image point onto the ground plane at camera height.

    Returns (x_forward_m, y_left_m) or None for a point at or above the horizon.
    Camera pitch is positive when the optical axis points down.
    """
    fx, fy = intrinsics['fx'], intrinsics['fy']
    width, height = intrinsics['width'], intrinsics['height']
    height_m, pitch = intrinsics['height_m'], intrinsics['pitch_rad']
    a = (u*width-intrinsics['ppx'])/fx          # right in camera frame
    b = (v*height-intrinsics['ppy'])/fy         # down in camera frame
    cos, sin = math.cos(pitch), math.sin(pitch)
    denominator = b*cos+sin
    if denominator <= 1e-3:                      # at or above the horizon line
        return None
    t = height_m/denominator
    x = t*(cos-b*sin)
    y = -t*a
    if not all(math.isfinite(value) for value in (x, y)) or x <= 0.05:
        return None
    return x, y


def project_image_route(points_uv, intrinsics, max_range_m=4.):
    """Convert an ordered image route into a body-frame route starting at the robot.

    Points that do not hit the ground are dropped; the rest are clamped to
    max_range_m and deduplicated. Fewer than two usable points is an error, not
    a silently shortened route.
    """
    if not number(max_range_m) or max_range_m <= 0:
        raise ValueError('invalid_max_range')
    route = [(0., 0.)]
    for point in points_uv:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise ValueError('invalid_plan_waypoint')
        projected = image_point_to_local(point[0], point[1], intrinsics)
        if projected is None:
            continue
        x, y = projected
        distance = math.hypot(x, y)
        if distance > max_range_m:
            scale = max_range_m/distance
            x, y = x*scale, y*scale
        if math.hypot(x-route[-1][0], y-route[-1][1]) < .05:
            continue
        route.append((x, y))
    if len(route) < 3:
        raise ValueError('plan_route_above_horizon')
    path = []
    for index, (x, y) in enumerate(route):
        if index+1 < len(route):
            yaw = math.atan2(route[index+1][1]-y, route[index+1][0]-x)
        elif len(path) > 1:
            yaw = path[-1][2]
        else:
            yaw = 0.
        path.append((x, y, yaw))
    return tuple(path)
