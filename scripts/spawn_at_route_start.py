"""Spawn Script Node: puts the car at the start of the route, facing along it.

Runs once after the scene opens (On Tick -> Once) and again when the route's
points change (On USD Object Change), only while the timeline is stopped.
Both event nodes have Only Simulate On Play off; this node's Exec Out is unwired.
It writes the car's pose to the simulation layer in memory and saves nothing.
"""
import math

ROUTE_PATH = "/City/DrivingRoute"
VEHICLE_PATH = "/City/WizardVehicle1/Vehicle"
CLEARANCE = 0.94  # metres above route: route Y=0.06 gives chassis Y=1.00


def _spawn_pose(world_points):
    """Return position and USD quaternion (W, X, Y, Z), with local +Z forward."""
    points = iter(world_points)
    first = tuple(float(v) for v in next(points))
    if len(first) != 3 or not all(math.isfinite(v) for v in first):
        raise ValueError("The route start must be a finite 3D point.")
    for point in points:
        point = tuple(float(v) for v in point)
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError("The route tangent must use finite 3D points.")
        dx, dz = point[0] - first[0], point[2] - first[2]
        if math.hypot(dx, dz) > 1e-6:  # skip repeated/vertical-only points
            yaw = math.atan2(dx, dz)   # +Z -> +X is positive Y rotation
            position = (first[0], first[1] + CLEARANCE, first[2])
            quaternion = (math.cos(yaw / 2), 0.0, math.sin(yaw / 2), 0.0)
            return position, quaternion
    raise ValueError("The route needs two distinct points in the XZ plane.")


def compute(db):
    import omni.timeline
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom

    # Pause is not Stop: never teleport a running or paused physics vehicle.
    if not omni.timeline.get_timeline_interface().is_stopped():
        return True

    try:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise ValueError("No USD stage is open.")
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.y:
            raise ValueError("The scene must be Y-up.")
        if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0, abs_tol=1e-8):
            raise ValueError("The scene must be in metres: metersPerUnit = 1.")

        route = stage.GetPrimAtPath(ROUTE_PATH)
        vehicle = stage.GetPrimAtPath(VEHICLE_PATH)
        if not route or not vehicle:
            raise ValueError("DrivingRoute or WizardVehicle1/Vehicle is missing.")
        root = stage.GetRootLayer()
        if not root.permissionToEdit:
            raise ValueError("The simulation root layer is not editable.")

        # We transform the route into world space. For the car, explicitly reject
        # transformed parents instead of accidentally writing world values locally.
        time = Usd.TimeCode.Default()
        parent_world = UsdGeom.Xformable(vehicle.GetParent()).ComputeLocalToWorldTransform(time)
        if not Gf.IsClose(parent_world, Gf.Matrix4d(1.0), 1e-6):
            raise ValueError("The vehicle's parent world transform must be identity.")
        if list(vehicle.GetAttribute("xformOpOrder").Get() or []) != [
            "xformOp:translate", "xformOp:orient", "xformOp:scale"
        ]:
            raise ValueError("The vehicle must use translate, orient, scale in that order.")
        scale = vehicle.GetAttribute("xformOp:scale").Get()
        if scale is None or not Gf.IsClose(Gf.Vec3d(scale), Gf.Vec3d(1.0), 1e-6):
            raise ValueError("The vehicle scale must be (1, 1, 1).")

        point_attr = route.GetAttribute("points")
        if not point_attr or point_attr.GetTypeName() != Sdf.ValueTypeNames.Point3fArray:
            raise ValueError("DrivingRoute.points must be point3f[].")
        points = point_attr.Get(time)
        counts = route.GetAttribute("curveVertexCounts").Get()
        if points is None or len(points) < 2 or list(counts or []) != [len(points)]:
            raise ValueError("DrivingRoute must contain one curve with at least two points.")
        route_world = UsdGeom.Xformable(route).ComputeLocalToWorldTransform(time)
        position, quaternion = _spawn_pose(route_world.Transform(Gf.Vec3d(p)) for p in points)

        # Validate every destination before authoring any property. These types
        # match the saved Wizard car; its USD orient is quatf, ordered WXYZ.
        V = Sdf.ValueTypeNames
        zero = Gf.Vec3f(0.0)
        writes = [
            ("xformOp:translate", V.Float3, Gf.Vec3f(*position)),
            ("xformOp:orient", V.Quatf, Gf.Quatf(quaternion[0], Gf.Vec3f(*quaternion[1:]))),
            ("physics:velocity", V.Vector3f, zero),
            ("physics:angularVelocity", V.Vector3f, zero),
            ("physxVehicleController:steer", V.Float, 0.0),
            ("physxVehicleController:accelerator", V.Float, 0.0),
            ("physxVehicleController:brake0", V.Float, 1.0),
        ]
        for name, expected_type, value in writes:
            attr = vehicle.GetAttribute(name)
            if not attr or attr.GetTypeName() != expected_type or attr.GetNumTimeSamples():
                raise ValueError(f"{name} must be a static {expected_type} attribute.")

        # Write only the simulation layer, even if another layer is selected.
        with Usd.EditContext(stage, root):
            for name, expected_type, value in writes:
                if not vehicle.GetAttribute(name).Set(value):
                    raise RuntimeError(f"Could not set {name}.")
        print(f"Spawned car at {position}; velocities zero, brake held.")
    except Exception as error:
        db.log_error(f"Route spawn: {error}")

    # Script Node enables its built-in Exec Out after compute. Leave it unwired;
    # keep the separate driving controller/writer chain on On Playback Tick.
    return True
