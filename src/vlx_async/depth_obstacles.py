"""Optional depth obstacle reader: measured obstacles in the body frame.

Reports solid objects in front of the robot (distance, width). It does not pick
targets, range goals or touch the plan. A frame with too little floor support is
reported as unavailable.
"""
import argparse
from datetime import datetime
import json
import math
import signal
import threading
import time

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - reported at start-up
    rs = None

from .storage import ROOT, atomic_json

WIDTH, HEIGHT = 480, 270
STEP = 4
MAX_RANGE_M = 4.0
MIN_RANGE_M = 0.20
FLOOR_BAND = .35          # bottom fraction of the image used to fit the floor
OBSTACLE_ABOVE_FLOOR_M = .08
OBSTACLE_MAX_HEIGHT_M = 1.30
CORRIDOR_HALF_WIDTH_M = .30
BIN_DEG = 3.


def _intrinsics(profile):
    stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    i = stream.get_intrinsics()
    return dict(fx=i.fx, fy=i.fy, ppx=i.ppx, ppy=i.ppy, width=i.width, height=i.height)


def depth_to_body(depth, scale, intrinsics):
    """Downsampled depth image -> body-frame points (x forward, y left, z up)."""
    rows = np.arange(0, intrinsics['height'], STEP)
    cols = np.arange(0, intrinsics['width'], STEP)
    u, v = np.meshgrid(cols, rows)
    z = depth[::STEP, ::STEP].astype(np.float32)*scale
    valid = (z > MIN_RANGE_M) & (z < MAX_RANGE_M)
    if not np.any(valid):
        return None
    z = z[valid]
    u, v = u[valid], v[valid]
    x_cam = (u-intrinsics['ppx'])/intrinsics['fx']*z
    y_cam = (v-intrinsics['ppy'])/intrinsics['fy']*z
    return dict(forward=z, left=-x_cam, up=-y_cam, height=v)


def fit_floor(points, seed=0):
    """Robust floor plane (up = a*forward + b*left + c) by RANSAC + refit.

    The floor dominates the depth image, but obstacles and the robot's own body
    pollute any fixed image band, so a plain least-squares fit biases the plane
    and turns floor into "obstacles". Sample triples, count inliers within 3 cm,
    then refit on the winning inliers.
    """
    f, l, u = points['forward'], points['left'], points['up']
    count = f.size
    if count < 300:
        return None
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(120):
        idx = rng.choice(count, 3, replace=False)
        a = np.column_stack([f[idx], l[idx], np.ones(3)])
        try:
            coef = np.linalg.solve(a, u[idx])
        except np.linalg.LinAlgError:
            continue
        if not np.all(np.isfinite(coef)):
            continue
        residual = np.abs(u-(coef[0]*f+coef[1]*l+coef[2]))
        inliers = residual < .03
        score = int(inliers.sum())
        if best is None or score > best[0]:
            best = (score, coef, inliers)
    if best is None or best[0] < .08*count:
        return None
    inliers = best[2]
    design = np.column_stack([f[inliers], l[inliers], np.ones(int(inliers.sum()))])
    try:
        coef, *_ = np.linalg.lstsq(design, u[inliers], rcond=None)
    except np.linalg.LinAlgError:
        return None
    residual = np.abs(u-(coef[0]*f+coef[1]*l+coef[2]))
    return dict(a=float(coef[0]), b=float(coef[1]), c=float(coef[2]),
                inlier_fraction=float(np.mean(residual < .03)))


def obstacles_from_points(points, floor):
    """Cluster points above the floor into body-frame obstacle circles.

    Height is perpendicular to the fitted floor (the camera is pitched down) and
    the threshold grows with range. Bins are narrow and split on depth
    discontinuities so two objects at different distances do not merge into one
    wall that closes a real gap.
    """
    if floor is None:
        return None
    f, l, u = points['forward'], points['left'], points['up']
    normal = math.sqrt(1.+floor['a']**2+floor['b']**2)
    height = (u-(floor['a']*f+floor['b']*l+floor['c']))/normal
    distance = np.hypot(f, l)
    threshold = np.maximum(.10, .012*distance**2)
    near = (f > MIN_RANGE_M) & (f < 3.5) & (np.abs(l) < f*math.tan(math.radians(55.)))
    solid = near & (height > threshold) & (height < OBSTACLE_MAX_HEIGHT_M)
    if np.count_nonzero(solid) < 8:
        return []
    f, l = f[solid], l[solid]
    distance, height = distance[solid], height[solid]
    bearing = np.degrees(np.arctan2(l, f))
    bins = np.floor(bearing/BIN_DEG).astype(int)
    clusters = []
    for index in sorted(set(bins.tolist())):
        mask = bins == index
        if np.count_nonzero(mask) < 4:
            continue
        if np.percentile(height[mask], 90) < .20:
            continue
        ordered = np.sort(distance[mask])
        groups = np.split(ordered, np.where(np.diff(ordered) > .35)[0]+1)
        for group in groups:
            if group.size < 4:
                continue
            d = float(group.min())
            centre = math.radians((index+.5)*BIN_DEG)
            half_width = math.radians(BIN_DEG)*d/2.
            clusters.append(dict(bearing_deg=round(math.degrees(centre), 1), distance_m=round(d, 2),
                                 x_m=round(d*math.cos(centre), 2), y_m=round(d*math.sin(centre), 2),
                                 radius_m=round(min(.30, max(.20, half_width)), 2),
                                 height_m=round(float(np.percentile(height[mask], 90)), 2)))
    merged = []
    for cluster in sorted(clusters, key=lambda c: c['bearing_deg']):
        if (merged and cluster['bearing_deg']-merged[-1]['bearing_deg'] <= BIN_DEG*1.2
                and abs(cluster['distance_m']-merged[-1]['distance_m']) < .35):
            last = merged[-1]
            last['distance_m'] = round(min(last['distance_m'], cluster['distance_m']), 2)
            last['bearing_deg'] = round(.5*(last['bearing_deg']+cluster['bearing_deg']), 1)
            last['x_m'] = round(last['distance_m']*math.cos(math.radians(last['bearing_deg'])), 2)
            last['y_m'] = round(last['distance_m']*math.sin(math.radians(last['bearing_deg'])), 2)
            last['radius_m'] = round(min(.35, last['radius_m']+cluster['radius_m']*.5), 2)
        else:
            merged.append(dict(cluster))
    corridor = [c['distance_m'] for c in merged if abs(c['y_m']) <= CORRIDOR_HALF_WIDTH_M]
    return dict(obstacles=merged[:14], nearest_m=min((c['distance_m'] for c in merged), default=None),
                corridor_m=min(corridor) if corridor else None)


def snapshot(reader, intrinsics):
    depth, scale = reader()
    if depth is None:
        return dict(status='unavailable', reason='no_depth_frame')
    points = depth_to_body(depth, scale, intrinsics)
    if points is None:
        return dict(status='unavailable', reason='no_valid_depth')
    floor = fit_floor(points)
    result = obstacles_from_points(points, floor)
    if result is None:
        return dict(status='unavailable', reason='floor_not_fitted')
    return dict(status='running', **result, floor=floor, valid_points=int(points['forward'].size))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0., help='0 runs until stopped')
    parser.add_argument('--hz', type=float, default=10.)
    parser.add_argument('--output', default=str(ROOT/'runs/perception_live/obstacles.json'))
    args = parser.parse_args(argv)
    if not 0 <= args.duration <= 86400 or not 1. <= args.hz <= 30.:
        parser.error('invalid duration or rate')
    if rs is None:
        raise SystemExit('pyrealsense2 is required for depth obstacles')
    output = ROOT/'runs/perception_live'
    output.mkdir(parents=True, exist_ok=True)
    pipe, profile = rs.pipeline(), rs.config()
    profile.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
    started = pipe.start(profile)
    intrinsics = _intrinsics(started)
    scale = started.get_device().first_depth_sensor().get_depth_scale()
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())

    def read():
        frames = pipe.poll_for_frames()
        if not frames:
            return None, scale
        depth = frames.get_depth_frame()
        if not depth:
            return None, scale
        return np.asanyarray(depth.get_data()), scale

    run_dir = ROOT/'runs'/('depth_obstacles_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run_dir.mkdir(parents=True, exist_ok=True)
    print(json.dumps(dict(output=str(run_dir), status='started',
                          intrinsics=intrinsics, depth_scale=scale)), flush=True)
    began = time.monotonic()
    try:
        with (run_dir/'frames.jsonl').open('w') as log:
            while not stop.is_set() and (args.duration <= 0 or time.monotonic()-began < args.duration):
                tick = time.monotonic()
                try:
                    result = snapshot(read, intrinsics)
                except Exception as exc:  # keep the service alive and say why
                    result = dict(status='unavailable', reason=type(exc).__name__)
                result.update(schema='vlx-depth-obstacles-v1', timestamp_ns=time.monotonic_ns(),
                              robot_commands_sent=0)
                atomic_json(ROOT/'runs/perception_live/obstacles.json', result)
                log.write(json.dumps(result, allow_nan=False)+'\n')
                log.flush()
                stop.wait(max(.01, 1./args.hz-(time.monotonic()-tick)))
    finally:
        atomic_json(ROOT/'runs/perception_live/obstacles.json',
                    dict(schema='vlx-depth-obstacles-v1', status='finished',
                         timestamp_ns=time.monotonic_ns()))
        try:
            pipe.stop()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())


def latest_obstacles(path=None, max_age_ns=500_000_000):
    """Current measured obstacles as (x, y, radius) body-frame circles."""
    source = Path(path) if path else ROOT/'runs/perception_live/obstacles.json'
    try:
        data = json.loads(source.read_text())
        stamp = data.get('timestamp_ns')
        if (data.get('status') != 'running' or type(stamp) is not int
                or not 0 <= time.monotonic_ns()-stamp <= max_age_ns):
            return (), None
        circles = tuple((float(o['x_m']), float(o['y_m']), float(o['radius_m']))
                        for o in data.get('obstacles') or [])
        return circles, data
    except (OSError, ValueError, TypeError, KeyError):
        return (), None


def obstacles_enabled(path=None):
    """Operator switch (config/depth.json). Missing or broken means enabled."""
    import json as _json
    source = Path(path) if path else ROOT/'config/depth.json'
    try:
        data = _json.loads(source.read_text())
        value = data.get('depth_obstacles_enabled', True)
        return value if type(value) is bool else True
    except (OSError, ValueError, TypeError):
        return True
