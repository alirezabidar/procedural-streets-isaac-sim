"""Driving Script Node: pure-pursuit route following with RTX LiDAR braking.

Inputs: routePoints, position, carOrientation, velocity.
Outputs: steering, throttle, brake, obstacleDistance (float), sensorReady (bool).
Scene: metres, Y-up, vehicle local +Z forward; route points in world space.
Route tracking plus low-speed LiDAR braking in a corridor that follows the route.
Requires the existing ROS_LidarRTX RenderProduct and RTX sensor extensions.
This is not pedestrian classification or autonomous route planning.
"""
import math

CRUISE_SPEED = 1.5      # m/s cruise speed
MAX_THROTTLE = 1.0      # allow full accelerator when speed is below target
BRAKING_DECEL = 0.8     # m/s^2 used to plan the endpoint slowdown
WHEELBASE = 2.6         # measured from this Wizard vehicle
REAR_OFFSET = 1.3       # vehicle origin to rear axle, in metres
MAX_STEER = math.radians(30.0)
STOP_MARGIN = 0.75      # full-brake threshold before the route endpoint
MAX_ROUTE_ERROR = 3.0   # brake if the vehicle leaves the route corridor


def _vector(value, size):
    result = tuple(float(v) for v in value)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError("Missing, incorrectly shaped, or non-finite input")
    return result


def _route_info(points, position):
    """Project the vehicle centre onto every segment in the XZ ground plane."""
    segments, total, best_error, progress = [], 0.0, math.inf, 0.0
    for a, b in zip(points, points[1:]):
        dx, dz = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dz)
        if length < 1e-6:  # repeated points do not define a usable segment
            continue
        t = max(0.0, min(1.0, ((position[0]-a[0])*dx +
                              (position[1]-a[1])*dz) / length**2))
        error = math.hypot(position[0]-a[0]-t*dx, position[1]-a[1]-t*dz)
        if error < best_error:
            best_error, progress = error, total + t*length
        segments.append((a, b, total, length))
        total += length
    return segments, progress, total, best_error


def _point_at(segments, distance):
    """Interpolate by distance along the polyline, not by point count."""
    for a, b, start, length in segments:
        if distance <= start + length:
            t = max(0.0, (distance-start) / length)
            return a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
    return segments[-1][1]


def _drive_route(db):
    # Every frame starts with a safe command. Invalid data keeps the brake on.
    db.outputs.steering, db.outputs.throttle, db.outputs.brake = 0.0, 0.0, 1.0
    try:
        points3 = [_vector(p, 3) for p in db.inputs.routePoints]
        if len(points3) < 2:
            return True
        points = [(p[0], p[2]) for p in points3]  # Y is height, not road distance
        px, _, pz = _vector(db.inputs.position, 3)
        vx, _, vz = _vector(db.inputs.velocity, 3)
        x, y, z, w = _vector(db.inputs.carOrientation, 4)  # OGN order is XYZW
        norm = math.hypot(x, y, z, w)
        if not math.isfinite(norm) or norm < 1e-8:
            return True
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

        # Rotate local +Z by the quaternion, then project onto the road plane.
        fx, fz = 2*(x*z + w*y), 1-2*(x*x + y*y)
        flat_norm = math.hypot(fx, fz)
        if flat_norm < 1e-6:
            return True
        fx, fz = fx/flat_norm, fz/flat_norm
        segments, progress, total, error = _route_info(points, (px, pz))
        remaining = total-progress
        if not segments or error > MAX_ROUTE_ERROR or remaining <= STOP_MARGIN:
            return True

        speed = math.hypot(vx, vz)
        if not math.isfinite(speed):
            return True
        lookahead = 2.5 + 0.5*speed  # metres ahead of the vehicle centre
        tx, tz = _point_at(segments, min(total, progress+lookahead))
        # Pure pursuit measures its target from the REAR axle.
        dx, dz = tx-(px-REAR_OFFSET*fx), tz-(pz-REAR_OFFSET*fz)
        if dx*fx + dz*fz <= 0.0:  # target behind us: brake instead of reversing
            return True
        lateral = dx*fz - dz*fx  # positive toward local +X = positive steer
        angle = math.atan2(2*WHEELBASE*lateral, dx*dx+dz*dz)
        steering = max(-1.0, min(1.0, angle/MAX_STEER))

        # Slow in turns and approach the endpoint at a decreasing speed.
        desired = min(CRUISE_SPEED/(1+0.75*abs(steering)),
                      math.sqrt(2*BRAKING_DECEL*max(0.0, remaining-STOP_MARGIN)))
        speed_error = desired-speed
        throttle = min(MAX_THROTTLE, 0.08+0.4*speed_error) if speed_error > 0 else 0.0
        brake = min(1.0, -0.6*speed_error) if speed_error < 0 else 0.0
        if not all(math.isfinite(v) for v in (steering, throttle, brake)):
            return True
        db.outputs.steering = float(steering)
        db.outputs.throttle = float(throttle)
        db.outputs.brake = float(brake)
    except (TypeError, ValueError, AttributeError, IndexError, OverflowError):
        pass
    # No stored progress: stopping/restarting playback cannot leave a stale index.
    return True


# Sensor configuration for this scene. No obstacle prim paths are used.
VEHICLE_PATH = "/City/WizardVehicle1/Vehicle"
LIDAR_PATH = VEHICLE_PATH + "/Example_Rotary"
RENDER_PRODUCT_OUTPUT = "/Graph/ROS_LidarRTX/RenderProduct.outputs:renderProductPath"
FRONT_BUMPER = 2.0
DIAGNOSTIC_HOLD = False  # temporarily hold still while recording a scan
SELF_MARGIN = 0.10  # reject noisy returns on the chassis boundary
HALF_CORRIDOR_WIDTH = 1.25
STALE_SECONDS = 0.6
RELEASE_DISTANCE = 4.0
CLEAR_RAY_TOLERANCE_DEG = 1.0  # accommodates scan spacing at the stopped pose


def _corridor_distance(points):
    """Vehicle-local XYZ, Y up. Return bumper clearance, or -1 if clear.

    Fallback used only when the route cannot be read: a straight box ahead.
    Ignore the road and the car itself.
    """
    import numpy as np
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    keep = (np.isfinite(p).all(axis=1)
            & (np.abs(p[:, 0]) <= HALF_CORRIDOR_WIDTH)
            & (p[:, 1] > -0.55) & (p[:, 1] < 2.5)
            & (p[:, 2] > FRONT_BUMPER + SELF_MARGIN) & (p[:, 2] < 24.0))
    return float(p[keep, 2].min() - FRONT_BUMPER) if keep.any() else -1.0


PATH_STEP = 0.5        # metres between samples of the route ahead
MAX_RANGE = 24.0       # how far ahead along the route to look


def _path_ahead(route_points, position):
    """Sample the route from the car's position to MAX_RANGE ahead (XZ plane).

    Returns (samples Nx2, distance along the route from the car for each sample),
    or None when the route is not usable.
    """
    import numpy as np
    points = [(p[0], p[2]) for p in (_vector(q, 3) for q in route_points)]
    if len(points) < 2:
        return None
    segments, progress, total, _ = _route_info(points, (position[0], position[2]))
    if not segments:
        return None
    steps = np.arange(0.0, MAX_RANGE + PATH_STEP, PATH_STEP)
    samples = [_point_at(segments, min(total, progress + d)) for d in steps]
    return np.asarray(samples, dtype=float), steps


def _route_corridor(points, world_points, path):
    """Obstacle distance along the route ahead, or -1 if clear.

    points: LiDAR returns in the car frame (Y up, +Z forward), used for height
    and to ignore the car itself. world_points: the same returns in world space.
    A return counts only if it is within HALF_CORRIDOR_WIDTH of the route the car
    is about to drive, so the corridor bends with the road in turns and ignores
    trees, lamps and buildings beside it.
    Returns (distance from the front bumper, index of the nearest return).
    """
    import numpy as np
    samples, along = path
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    w = np.asarray(world_points, dtype=float).reshape(-1, 3)
    on_car = (np.abs(p[:, 0]) <= 1.2) & (p[:, 2] > -2.6) & (p[:, 2] <= FRONT_BUMPER + SELF_MARGIN)
    keep = np.isfinite(p).all(axis=1) & (p[:, 1] > -0.55) & (p[:, 1] < 2.5) & ~on_car
    lo = samples.min(axis=0) - HALF_CORRIDOR_WIDTH
    hi = samples.max(axis=0) + HALF_CORRIDOR_WIDTH
    keep &= (w[:, 0] >= lo[0]) & (w[:, 0] <= hi[0]) & (w[:, 2] >= lo[1]) & (w[:, 2] <= hi[1])
    candidates = np.flatnonzero(keep)
    if candidates.size == 0:
        return -1.0, None
    xz = w[candidates][:, [0, 2]].astype(np.float32)
    d2 = ((xz[:, None, :] - samples[None, :, :].astype(np.float32)) ** 2).sum(axis=2)
    nearest_sample = d2.argmin(axis=1)
    lateral = np.sqrt(d2[np.arange(len(xz)), nearest_sample])
    ahead = along[nearest_sample]
    hit = (lateral <= HALF_CORRIDOR_WIDTH) & (ahead > FRONT_BUMPER + SELF_MARGIN)
    if not hit.any():
        return -1.0, None
    best = np.flatnonzero(hit)[np.argmin(ahead[hit])]
    return float(ahead[best] - FRONT_BUMPER), int(candidates[best])


def _must_brake(distance, speed, latched):
    """Hysteresis prevents creeping toward the same stationary obstacle."""
    if distance < 0:
        return False
    threshold = max(4.0, 1.0 + 0.35*speed + speed*speed/(2*BRAKING_DECEL))
    return distance < max(threshold, RELEASE_DISTANCE if latched else 0.0)


def _clear_scan_duration(start, previous, stamp, clear):
    """Measure continuous evidence using sensor nanoseconds, not UI time."""
    if not clear:
        return None, False
    if previous is not None and stamp == previous:
        return start, False
    if (start is None or previous is None or stamp < previous
            or stamp - previous > int(STALE_SECONDS * 1e9)):
        start = stamp
    return start, stamp - start >= 1_000_000_000


def _world_to_car(points, position, quaternion):
    import numpy as np
    x, y, z, w = _vector(quaternion, 4)
    n = math.hypot(x, y, z, w)
    if n < 1e-8:
        raise ValueError("Invalid car orientation")
    x, y, z, w = x/n, y/n, z/n, w/n
    # Column-vector local-to-world rotation. Row-vector inverse uses R.
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return (points - np.asarray(_vector(position, 3))) @ rotation


def _space_observed_clear(world_points, origin, blocked_point):
    """Require returns beyond the old obstacle along nearly the same rays.

    Missing returns are unknown space, not evidence of clearance. This is a
    small stationary-obstacle experiment, not a complete occupancy mapper.
    """
    import numpy as np
    if blocked_point is None:
        return False
    target = np.asarray(blocked_point) - np.asarray(origin)
    target_range = np.linalg.norm(target)
    if not np.isfinite(target_range) or target_range < 1.0:
        return False
    rays = np.asarray(world_points) - np.asarray(origin)
    ranges = np.linalg.norm(rays, axis=1)
    valid = np.isfinite(rays).all(axis=1) & (ranges > 1e-5)
    rays, ranges = rays[valid], ranges[valid]
    cosines = (rays @ (target / target_range)) / ranges
    same_direction = cosines > math.cos(math.radians(CLEAR_RAY_TOLERANCE_DEG))
    # Nearby blocking returns veto clearance, even if some rays pass an edge.
    if np.any(same_direction & (ranges < target_range + 0.4)):
        return False
    return bool(np.count_nonzero(same_direction & (ranges > target_range + 0.75)) >= 3)


def setup(db):
    import omni.timeline
    s = db.per_instance_state
    s.writer = None
    s.product = None
    s.next_attach_attempt = 0.0
    s.sample = None
    s.latched = False
    s.clear_since = None
    s.message = None
    s.pose = None
    s.path = None
    s.diagnostic = None
    s.reported_detection = False
    s.reported_scan = False
    s.blocked_world = None
    s.nearest_world = None
    s.clear_last_stamp = None
    s.resume_report_time = -100.0
    print("[LiDAR braking] Controller loaded")
    s.timeline = omni.timeline.get_timeline_interface()

    def reset_on_stop(event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            s.sample = None
            s.latched = False
            s.clear_since = None
            s.reported_detection = False
            s.blocked_world = None
            s.clear_last_stamp = None
    s.subscription = s.timeline.get_timeline_event_stream().create_subscription_to_pop(reset_on_stop)


def cleanup(db):
    s = db.per_instance_state
    writer = getattr(s, "writer", None)
    if writer is not None:
        writer.detach()
    s.writer = None
    s.subscription = None
    s.sample = None


def _attach_sensor(s):
    """Reuse the wizard's render product; do not create another sensor."""
    import numpy as np
    import time
    import omni.graph.core as og
    import omni.usd
    import omni.replicator.core as rep
    from pxr import UsdGeom, UsdRender
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

    product = og.Controller.attribute(RENDER_PRODUCT_OUTPUT).get()
    if not product:
        raise ValueError("Waiting for ROS_LidarRTX RenderProduct; leave that graph enabled")
    product = str(product)
    if s.writer is not None and product == s.product:
        return
    if time.monotonic() < s.next_attach_attempt:
        raise ValueError("Waiting before retrying LiDAR attachment")
    s.next_attach_attempt = time.monotonic() + 2.0
    stage = omni.usd.get_context().get_stage()
    sensor = stage.GetPrimAtPath(LIDAR_PATH)
    rp = UsdRender.Product(stage.GetPrimAtPath(product))
    if not sensor.IsValid() or not rp:
        raise ValueError("LiDAR or render product not found; check the two paths in this script")
    targets = [str(p) for p in rp.GetCameraRel().GetTargets()]
    if LIDAR_PATH not in targets:
        raise ValueError("Render product targets a different sensor")
    if str(sensor.GetParent().GetPath()) != VEHICLE_PATH:
        raise ValueError("LiDAR must be a direct child of Vehicle")
    frame = str(sensor.GetAttribute("omni:sensor:Core:outputFrameOfReference").Get()).upper()
    if frame not in ("SENSOR", "LOCAL", "WORLD"):
        raise ValueError("Unsupported LiDAR output frame: " + frame)
    mount = np.asarray(UsdGeom.Xformable(sensor).GetLocalTransformation(), dtype=float)

    class LidarReader:
        def __init__(self):
            self.annotators = [rep.annotators.get("GenericModelOutput"),
                               rep.annotators.get("IsaacExtractRTXSensorPointCloud")]

        def attach(self, products):
            self.attached = []
            try:
                for annotator in self.annotators:
                    annotator.attach(products[0])
                    self.attached.append(annotator)
            except Exception:
                self.detach()
                raise

        def detach(self):
            for annotator in getattr(self, "attached", []):
                try:
                    annotator.detach()
                except Exception:
                    pass
            self.attached = []

        def poll(self):
            # Flat metadata preserves the RTX capture transform alongside data.
            values = {
                name: annotator.get_data(device="cpu", use_legacy_structure=False)
                for name, annotator in zip(
                    ("GenericModelOutput", "IsaacExtractRTXSensorPointCloud"),
                    self.annotators)
            }
            self.write({"renderProducts": {product: values}})

        def write(self, data):
            if not s.timeline.is_playing():
                return
            try:
                for values in data.get("renderProducts", {}).values():
                    raw = values.get("GenericModelOutput")
                    cloud = values.get("IsaacExtractRTXSensorPointCloud")
                    cloud_metadata = cloud if isinstance(cloud, dict) else {}
                    raw = raw.get("data") if isinstance(raw, dict) else raw
                    cloud = cloud.get("data") if isinstance(cloud, dict) else cloud
                    if raw is None or cloud is None:
                        continue
                    gmo = parse_generic_model_output_data(raw)
                    count, stamp = int(gmo.numElements), int(gmo.timestampNs)
                    if count <= 0 or (s.sample is not None and stamp == s.sample[0]):
                        continue
                    cloud = cloud.numpy() if hasattr(cloud, "numpy") else cloud
                    points = np.asarray(cloud).reshape(-1, 3)[:count].copy()
                    # Empty/no-return buffers do not prove that the road is clear.
                    valid = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-5)
                    if len(points) != count or not valid.any():
                        continue
                    points = points[valid]
                    if s.pose is None:
                        continue
                    if frame == "WORLD":
                        world_points = points
                    else:
                        # RTX supplies its own basis conversion in this matrix.
                        # The USD prim mount alone does not describe point coordinates.
                        capture = np.asarray(cloud_metadata.get("transform"), dtype=float)
                        if capture.size != 16 or not np.isfinite(capture).all():
                            raise ValueError("Missing sensor capture transform")
                        capture = capture.reshape(4, 4)
                        world_points = points @ capture[:3, :3] + capture[3, :3]
                    points = _world_to_car(world_points, *s.pose)
                    if s.path is not None:
                        distance, nearest_index = _route_corridor(points, world_points, s.path)
                    else:
                        distance, nearest_index = _corridor_distance(points), None
                        if distance >= 0:
                            mask = ((np.abs(points[:, 0]) <= HALF_CORRIDOR_WIDTH)
                                    & (points[:, 1] > -0.55) & (points[:, 1] < 2.5)
                                    & (points[:, 2] > FRONT_BUMPER + SELF_MARGIN) & (points[:, 2] < 24.0))
                            indices = np.flatnonzero(mask)
                            nearest_index = indices[np.argmin(points[indices, 2])]
                    if distance >= 0 and nearest_index is not None:
                        nearest = points[nearest_index]
                        s.nearest_world = world_points[nearest_index].copy()
                        s.diagnostic = (frame, nearest.tolist(), mount.tolist())
                    sensor_transform = np.asarray(cloud_metadata.get("transform"), dtype=float).reshape(4, 4)
                    clear = _space_observed_clear(world_points, sensor_transform[3, :3], s.blocked_world)
                    s.sample = (stamp, s.timeline.get_current_time(), time.monotonic(), distance, clear)
            except Exception as error:
                s.sample = None
                _report(s, "Sensor read failed: " + str(error))

    if s.writer is not None:
        s.writer.detach()
    s.sample = None
    s.writer = None
    s.product = None
    reader = LidarReader()
    reader.attach([product])
    s.writer = reader
    s.product = product


def _report(s, message):
    if message != s.message:
        print("[LiDAR braking] " + message)
        s.message = message


def compute(db):
    import time
    db.outputs.steering = 0.0
    db.outputs.throttle = 0.0
    db.outputs.brake = 1.0
    db.outputs.obstacleDistance = -1.0
    db.outputs.sensorReady = False
    s = db.per_instance_state
    try:
        if not hasattr(s, "timeline"):
            setup(db)
        if not s.timeline.is_playing():
            return True
        s.pose = (_vector(db.inputs.position, 3), _vector(db.inputs.carOrientation, 4))
        try:
            s.path = _path_ahead(db.inputs.routePoints, s.pose[0])
        except (TypeError, ValueError, IndexError):
            s.path = None  # fall back to the straight corridor
        _attach_sensor(s)
        s.writer.poll()
        now = time.monotonic()
        sample = s.sample
        # UI timeline can loop while RTX/physics continue advancing. Receipt
        # time is the watchdog; GMO timestamps measure consecutive clear scans.
        if sample is None or not (0 <= now-sample[2] <= 2.0):
            s.clear_since = None
            _report(s, "Waiting for fresh LiDAR measurements; holding brakes")
            return True
        distance = sample[3]
        db.outputs.obstacleDistance = float(distance)
        db.outputs.sensorReady = True
        speed = math.hypot(*(_vector(db.inputs.velocity, 3)[i] for i in (0, 2)))
        if _must_brake(distance, speed, s.latched):
            if not s.latched:
                s.blocked_world = s.nearest_world
            s.latched = True
            s.clear_since = None
        elif s.latched and sample[0] != s.clear_last_stamp:
            s.clear_since, release = _clear_scan_duration(
                s.clear_since, s.clear_last_stamp, sample[0],
                sample[4] and speed < 0.15)
            s.clear_last_stamp = sample[0]
            if release:
                s.latched = False
                s.blocked_world = None
                s.clear_since = None
                s.reported_detection = False
                _report(s, "Blocked space observed clear for one second; resuming")
        if DIAGNOSTIC_HOLD:
            _report(s, "Diagnostic mode: brakes held while recording scan")
            return True
        _drive_route(db)
        if s.latched:
            if now - s.resume_report_time >= 2.0:
                print("[LiDAR resume] distance=" + str(round(distance, 3))
                      + " clear_ray_evidence=" + str(sample[4])
                      + " speed=" + str(round(speed, 4))
                      + " clear_since=" + str(s.clear_since)
                      + " sensor_stamp=" + str(sample[0])
                      + " timeline=" + str(sample[1])
                      + " blocked_point=" + str(s.blocked_world))
                s.resume_report_time = now
            db.outputs.throttle, db.outputs.brake = 0.0, 1.0
            _report(s, "Obstacle ahead; holding brakes")
            if not s.reported_detection:
                print("[LiDAR diagnostic] distance=" + str(distance) + " frame,point,mount=" + str(s.diagnostic))
                s.reported_detection = True
        else:
            _report(s, "Sensor ready; route controller active")
    except Exception as error:
        db.outputs.throttle, db.outputs.brake = 0.0, 1.0
        db.outputs.sensorReady = False
        _report(s, "Holding brakes: " + str(error))
    return True
