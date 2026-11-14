"""Planar transforms, remaining-path extraction and current-scene checks."""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def align(path, old, current):
    if old.frame_id != current.frame_id:
        raise ValueError("odometry_frame_changed")
    c0, s0 = math.cos(old.yaw), math.sin(old.yaw)
    c1, s1 = math.cos(current.yaw), math.sin(current.yaw)
    out = []
    for x, y, yaw in path:
        wx, wy = old.x + c0*x - s0*y, old.y + s0*x + c0*y
        dx, dy = wx-current.x, wy-current.y
        out.append((c1*dx+s1*dy, -s1*dx+c1*dy, wrap(yaw+old.yaw-current.yaw)))
    return out


def distance_to_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx + dy*dy
    t = min(1., max(0., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2)) if length2 else 0.
    q = (a[0]+t*dx, a[1]+t*dy)
    return math.hypot(p[0]-q[0], p[1]-q[1]), t, q


def remaining(path):
    if len(path) < 2:
        return [], float("inf")
    best = min(((*distance_to_segment((0, 0), a, b), i)
                for i, (a, b) in enumerate(zip(path, path[1:]))), key=lambda v: v[0])
    distance, t, q, i = best
    if i == len(path)-2 and t >= 1-1e-6:
        return [], distance
    yaw = wrap(path[i][2] + t*wrap(path[i+1][2]-path[i][2]))
    return [(q[0], q[1], yaw)] + list(path[i+1:]), distance


def path_distance(path, reference):
    return sum(min(distance_to_segment(p, a, b)[0] for a, b in zip(reference, reference[1:]))
               for p in path) / len(path)


def rotate(path, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [(c*x-s*y, s*x+c*y, wrap(yaw+angle)) for x, y, yaw in path]


def first_path_collision(path, obstacles, radius):
    """Return the first conflicting measured circle and segment, if any.

    This uses the same swept-circle test as the command gate. Diagnostics do
    not discard points, shorten paths, or change the collision boundary.
    """
    connected = [(0., 0., 0.)] + list(path)
    if len(connected) < 2:
        return None
    # Reject distant circles using the full path's bounding box first. This is
    # an exact broad phase: every segment, including origin->first, is inside
    # this box; circles touching it still get the original distance check.
    xmin, xmax = min(p[0] for p in connected), max(p[0] for p in connected)
    ymin, ymax = min(p[1] for p in connected), max(p[1] for p in connected)
    segments = list(zip(connected, connected[1:]))
    for ox, oy, rr in obstacles:
        reach = radius+rr
        if ox < xmin-reach or ox > xmax+reach or oy < ymin-reach or oy > ymax+reach:
            continue
        for index, (a, b) in enumerate(segments):
            distance, fraction, closest = distance_to_segment((ox, oy), a, b)
            if not distance > reach:
                finite = all(math.isfinite(v) for v in (ox, oy, rr, radius, distance))
                return dict(obstacle_xy_m=[ox, oy] if finite else None,
                    obstacle_radius_m=rr if finite else None, footprint_radius_m=radius,
                    segment_index=index, segment_fraction=fraction if finite else None,
                    path_point_xy_m=list(closest) if finite else None,
                    clearance_m=distance-reach if finite else None,
                    overlaps_current_footprint=math.hypot(ox, oy) <= reach if finite else None,
                    invalid_geometry=not finite)
    return None


def collision_free(path, obstacles, radius):
    return first_path_collision(path, obstacles, radius) is None


def observed_path(path, free_cells, resolution, radius):
    """Every newly swept footprint cell must have measured floor support.

    The standing robot's current footprint is excluded. This does not fill the
    camera's blind region: any newly entered, unseen cell still rejects motion.
    """
    if (not path or not math.isfinite(resolution) or not .02 <= resolution <= .20
            or not math.isfinite(radius) or radius <= 0 or not free_cells):
        return False
    known = set(free_cells)
    def footprint(x, y):
        reach = radius+resolution/math.sqrt(2)
        return {(i, j) for i in range(math.floor((x-reach)/resolution), math.ceil((x+reach)/resolution)+1)
                for j in range(math.floor((y-reach)/resolution), math.ceil((y+reach)/resolution)+1)
                if math.hypot((i+.5)*resolution-x, (j+.5)*resolution-y) <= reach}
    initial = footprint(0., 0.)
    connected = [(0., 0., 0.)]+list(path)
    for a, b in zip(connected, connected[1:]):
        count = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/(resolution/2)))
        for i in range(1, count+1):
            x, y = a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count
            if not footprint(x, y)-initial <= known:
                return False
    return True


def regrid_free_cells(cells, resolution, old_pose, current_pose):
    """Conservatively warp observed cells between odometry frames.

    A new grid cell is accepted only if all four corners map into observed
    source cells. This erodes edges; it never fills unknown space by rounding.
    """
    known = {tuple(c) for c in cells}
    if not known:
        return ()
    if old_pose.frame_id != current_pose.frame_id:
        raise ValueError('odometry_frame_changed')
    if old_pose.x == current_pose.x and old_pose.y == current_pose.y and old_pose.yaw == current_pose.yaw:
        return tuple(sorted(known))
    centers = align([((x+.5)*resolution, (y+.5)*resolution, 0.) for x,y in known], old_pose, current_pose)
    possible = {(math.floor(x/resolution), math.floor(y/resolution)) for x,y,_ in centers}
    result = []
    for i,j in possible:
        corners = [((i+dx)*resolution, (j+dy)*resolution, 0.) for dx,dy in ((0,0),(0,1),(1,0),(1,1))]
        original = align(corners, current_pose, old_pose)
        if all((math.floor(x/resolution),math.floor(y/resolution)) in known for x,y,_ in original):
            result.append((i,j))
    return tuple(sorted(result))


def project_image_path(path, homography):
    """H maps normalized image ground pixels to the VLX local metre frame."""
    if len(homography) != 3 or any(len(row) != 3 for row in homography):
        raise ValueError("invalid_homography")
    out = []
    for u, v, _ in path:
        if not 0 <= u <= 1 or not 0 <= v <= 1:
            raise ValueError("invalid_image_coordinate")
        x, y, w = [row[0]*u+row[1]*v+row[2] for row in homography]
        if not math.isfinite(w) or abs(w) < 1e-8:
            raise ValueError("image_point_near_horizon")
        x, y = x/w, y/w
        if not all(math.isfinite(t) for t in (x, y)) or x < 0 or math.hypot(x, y) > 15:
            raise ValueError("projection_out_of_bounds")
        out.append((x, y, 0.))
    for i in range(len(out)-1):
        x, y, _ = out[i]
        out[i] = (x, y, math.atan2(out[i+1][1]-y, out[i+1][0]-x))
    if len(out)>1:
        out[-1] = (*out[-1][:2], out[-2][2])
    return out
