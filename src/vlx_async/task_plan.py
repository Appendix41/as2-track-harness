"""One plan per task: a frozen body-frame route with odometry-measured progress.

The route is captured when DeepSeek answers, expressed in the robot frame at
that instant. Every later tick re-expresses it through the current pose, so the
robot's own movement consumes the route without any further model call.
Arrival is measured along that same route, which keeps progress consistent even
when the monocular projection biases absolute distances.
"""
from dataclasses import dataclass
import math

from .contracts import TaskConfig, number
from .geometry import align, distance_to_segment


@dataclass(frozen=True)
class RoutePlan:
    """Accepted DeepSeek plan for exactly one task."""
    request_id: str
    task_key: tuple
    points: tuple          # (x forward, y left, yaw) body frame at plan time
    confidence: float
    description: str
    provider: str
    created_ns: int
    pose: object           # contracts.Pose captured with the request
    source: str = 'monocular_ground'
    calls: int = 1

    def __post_init__(self):
        if not isinstance(self.request_id, str) or not self.request_id:
            raise ValueError('invalid_plan_request')
        if not isinstance(self.points, tuple) or len(self.points) < 2:
            raise ValueError('invalid_plan_route')
        if not all(isinstance(p, (list, tuple)) and len(p) == 3
                   and all(number(v) for v in p) for p in self.points):
            raise ValueError('invalid_plan_route')
        if not number(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('invalid_plan_confidence')
        if type(self.created_ns) is not int or self.created_ns <= 0:
            raise ValueError('invalid_plan_clock')
        if self.pose is None or not getattr(self.pose, 'frame_id', None):
            raise ValueError('invalid_plan_pose')


def path_length(path):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:]))


def remaining_length(path):
    """Closest-segment remaining length and cross-track offset from the origin.

    A path in the current body frame starts at the pose of the robot when the
    plan was made; the origin is where the robot is now. The closest segment
    therefore tells how much route is still ahead and how far sideways the
    robot has drifted.
    """
    if len(path) < 2:
        return 0., float('inf')
    best = min(((distance_to_segment((0., 0.), a, b)[0], i, distance_to_segment((0., 0.), a, b)[2])
                for i, (a, b) in enumerate(zip(path, path[1:]))), key=lambda v: v[0])
    cross_track, index, closest = best
    remaining = math.hypot(path[index+1][0]-closest[0], path[index+1][1]-closest[1])
    remaining += sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path[index+1:], path[index+2:]))
    return remaining, cross_track


def plan_status(plan, pose, now_ns, config):
    """Aligned route plus progress for one tick. Raises on an unusable pose."""
    if plan is None:
        raise ValueError('plan_unavailable')
    if pose is None:
        raise ValueError('pose_unavailable')
    if pose.frame_id != plan.pose.frame_id:
        raise ValueError('odometry_frame_changed')
    aligned = align(plan.points, plan.pose, pose)
    remaining, cross_track = remaining_length(aligned)
    total = max(path_length(plan.points), 1e-9)
    fraction = min(1., max(0., 1.-remaining/total))
    return dict(route=aligned, remaining_m=remaining, total_m=total,
                cross_track_m=cross_track, progress_fraction=fraction,
                completed=bool(remaining <= config.arrival_distance_m),
                age_s=max(0., (now_ns-plan.created_ns)/1e9))


def heading_deviation(path):
    """Largest heading change from straight ahead along a body-frame route."""
    return max((abs(math.atan2(y, x)) for x, y, _ in path[1:] if math.hypot(x, y) > .1), default=0.)


def validate_route_shape(path, config):
    """Reject routes the differential walker cannot start following sensibly."""
    if len(path) < 2:
        return 'plan_route_too_short'
    if path_length(path) < config.arrival_distance_m:
        return 'plan_route_too_short'
    # A target behind the robot is allowed: the command gate rotates in place
    # until the plan direction is reachable, then drives forward.
    return None


def _point_at(points, distance):
    """Point at an arc length from the start of a polyline."""
    for a, b in zip(points, points[1:]):
        segment = math.hypot(b[0]-a[0], b[1]-a[1])
        if segment <= 1e-9:
            continue
        if distance <= segment:
            r = distance/segment
            return (a[0]+(b[0]-a[0])*r, a[1]+(b[1]-a[1])*r), (b[0]-a[0], b[1]-a[1])
        distance -= segment
    a, b = points[-2], points[-1]
    direction = (b[0]-a[0], b[1]-a[1])
    length = math.hypot(*direction) or 1.
    return (b[0]+direction[0]/length*distance, b[1]+direction[1]/length*distance), direction


def resample_route(path, count=8, step_m=.10):
    """Turn a plan route into the executor's 8 points, starting at the robot.

    When a plan exists it IS the trajectory: the route is resampled at a fixed
    step so the command's speed comes from the operator's limits, not from the
    VLX step spacing. Points past the route end continue straight.
    """
    if len(path) < 2 or not number(step_m) or step_m <= 0 or count < 2:
        raise ValueError('plan_route_too_short')
    best = min(((distance_to_segment((0., 0.), a, b)[0], i, distance_to_segment((0., 0.), a, b)[2])
                for i, (a, b) in enumerate(zip(path, path[1:]))), key=lambda v: v[0])
    _, index, closest = best
    points = [tuple(closest)] + [tuple(p) for p in path[index+1:]]
    if len(points) < 2:
        raise ValueError('plan_route_too_short')
    out = [(0., 0., 0.)]
    previous = (0., 0.)
    for k in range(1, count):
        point, direction = _point_at(points, k*step_m)
        yaw = math.atan2(point[1]-previous[1], point[0]-previous[0]) if k > 1 else \
            math.atan2(point[1], point[0])
        out.append((point[0], point[1], yaw))
        previous = point
    return tuple(out)


def route_heading(path, lookahead_m=.30):
    """Direction of the plan route as seen from the robot, at a lookahead."""
    if len(path) < 2:
        raise ValueError('plan_route_too_short')
    best = min(((distance_to_segment((0., 0.), a, b)[0], i, distance_to_segment((0., 0.), a, b)[2])
                for i, (a, b) in enumerate(zip(path, path[1:]))), key=lambda v: v[0])
    _, index, closest = best
    points = [tuple(closest)] + [tuple(p) for p in path[index+1:]]
    if len(points) < 2:
        raise ValueError('plan_route_too_short')
    target, _ = _point_at(points, lookahead_m)
    return math.atan2(target[1], target[0])

