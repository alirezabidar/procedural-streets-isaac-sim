# Procedural Streets for Autonomous Driving

A Houdini HDA generates a fictional street district and a driving route that
follows the layout. The cooked geometry, the route and the layout controls are
exported to one USD file. In Isaac Sim, a physically simulated car follows the
route and uses RTX LiDAR to stop for a pedestrian crossing the street.

## Houdini

Open `houdini/procedural_driving.hip` in Houdini 21.0.440 and select
`/obj/procedural_street_district/street_district`. The asset is
`houdini/street_district.hda` (`abidar::street_district_linked::2.0`).

The **Street Layout**, **Architecture** and **Street Details** tabs expose the
controls: block counts, block spacing, road width, sidewalk width, seed, building
footprint, and toggles for crosswalks, trees and lamps. The **Vehicle** tab sets
the car size used by the curb check.

Inside, native SOPs are grouped into five branches: `ROAD_SURFACES`,
`BLOCK_PARCELS`, `BUILDING_LIBRARY`, `ROAD_MARKINGS` and `STREET_FURNITURE`.
There are no Python SOPs or wrangles in the geometry.

### Driving route

The route is built from the same layout controls as the roads, so it moves with
the streets when the layout changes:

| Node | What it does |
|---|---|
| `graph_paper_points` (Add) | 4 points in block units: (1, 0.15), (1, 1), (2, 1), (2, 1.7). x = street number from the left edge, z = street number from the top edge. |
| `stretch_to_city` (Transform) | Scale = block spacing, translate = city edge. |
| `step_into_lane` (Transform) | Moves the route into the right-hand lane: X −road width/4, Z +road width/4. |
| `round_right_turn` (PolyBevel) | Rounds the right turn, radius 0.65 × road width. |
| `round_left_turn` (PolyBevel) | Rounds the left turn, radius 1.25 × road width. |
| `point_every_25cm` (Resample) | Even spacing for the car controller. |
| `DRIVING_ROUTE` (Null) | Route output. |

`route_usd_path` (Attribute Create) sets the USD path `/City/DrivingRoute` and
marks the route as guide geometry (`usdpurpose`). A Merge
combines it with the city, and `layout_metadata` (Attribute Create) stores the
layout controls as detail attributes, which the export writes as constant
primvars (`blocks_x`, `blocks_z`, `pitch_x`, `pitch_z`, `road_width`, `sidewalk`).
Isaac Sim reads the route and these values from the same file.

The traffic is right-hand. The scene is Y-up: facing +Z, the driver's right is −X.

### Layout rules and curb check

`layout_rules` (Error SOP) sits between the route and `DRIVING_ROUTE`. It stops
the cook, and therefore the export, when:

- road width < 10 m,
- X spacing < 2.6 × road width + 2 m,
- Z spacing < 2 × road width + 10 m, or
- the car would touch a raised sidewalk.

The curb check is a small SOP branch:

| Node | What it does |
|---|---|
| `route_frames` (PolyFrame) + `route_up` (Attribute Create) | Direction of travel (`N`) and `up` at each route point. |
| `vehicle_from_usd` (USD Import) / `car_size_manual` (Box) → `car_source` (Switch) | The car's size: read from the vehicle in `usd/street_driving.usda` (2.15 × 4.0 m including wheels), or typed in on the **Vehicle** tab. |
| `car_footprint` (Box) + `ray_down` (Attribute Create) | Car size plus clearance (0.05 m each side, 0.45 m each end) = 2.25 × 4.90 m, sampled every ~0.25 m, 2 m above the route, with rays pointing down. |
| `car_along_route` (Copy to Points) | One footprint per route point. |
| `sidewalks_only` (Blast) | Raised sidewalks and the plots inside them. |
| `car_on_sidewalk` (Ray) → `CURB_CHECK` (Null) | Footprint points that land on a sidewalk go into the group `on_sidewalk`, which must be empty. |

Tested on the 10 benchmark layouts (all clear), a road width of 8 m (rejected) and
the route pushed 3 m toward the curb (rejected).

### Export

The saved layout is 4 × 2 blocks, 46 × 38 m spacing, 10 m roads and 2.2 m
sidewalks, which is the layout the Isaac Sim scene (pedestrian path and NavMesh)
is set up for.

**Export District USD** on the HDA writes `usd/street_district.usdc` through the
`USD_EXPORT` node (USD Export SOP): mesh geometry, the route as a guide
`BasisCurves` prim, and the layout values. A layout that breaks a rule fails with
the rule's message.

The 10 benchmark layouts are exported from a copy of this HDA by a TOP network in
[driving-scenario-benchmark](https://github.com/alirezabidar/driving-scenario-benchmark).

### Check (hython)

```powershell
& "C:\Program Files\Side Effects Software\Houdini 21.0.440\bin\hython.exe" houdini\validate_district.py
```

Six parameter cases: repeatable seeds, toggles and building footprints.

## Isaac Sim scene

Open `usd/street_driving.usda` in Isaac Sim 6 and press **Play**. The scene
sublayers the Houdini export (`usd/street_district.usdc`) and adds everything that
belongs to the simulation on top of it, so the city can be re-exported without
touching the car, sensors or graph:

- a PhysX vehicle (`/City/WizardVehicle1`, made with the Vehicle Wizard) with an
  RTX LiDAR (`Example_Rotary`) and a front camera,
- static collision on the roads and raised sidewalks,
- a pedestrian that walks across the street on a baked NavMesh,
- the Action Graph `/City/ActionGraph`.

### Action Graph

| Branch | Nodes | What it does |
|---|---|---|
| Drive | On Playback Tick → Read Prim Attribute (route points, velocity), Get Local to World Transform → Decompose Matrix, Get Rotation Quaternion → Script Node → three Write Prim Attribute nodes | Every frame: read the route and the car's state, compute steering, throttle and brake, and write them to the vehicle controller. |
| Spawn / reset | On Tick → Once, and On USD Object Change → Script Node | When the scene opens, or the route changes after a re-export, place the car at the start of the route facing along it, with zero velocity and the brake on. |

The two Script Nodes load their code from `scripts/`:

- `scripts/lidar_braking_controller.py`: pure pursuit from the rear axle sets
  steering; speed is reduced in turns and before the route end. A LiDAR return
  counts as an obstacle only if it is within 1.25 m of the route the car is about
  to drive (the next 24 m), so the detection corridor bends with the road in
  turns and ignores trees, lamps and buildings beside it. The nearest such return,
  measured along the route, is the obstacle distance. The car stops when that distance is
  inside its stopping distance and waits until the blocked space is seen clear for
  one second before driving on.
- `scripts/spawn_at_route_start.py`: the spawn / reset step.

Python is used only where the graph needs it: reducing a LiDAR point cloud to one
distance, and the controller maths. Everything else is native graph nodes.

The scene is Y-up and in metres. The car's local +Z is forward.

## Limits

Simulator ground-truth pose; one pedestrian on a fixed crossing path; no traffic
rules, route planning or learned perception. The LiDAR corridor is a geometric
filter around the planned route on flat roads, not a full collision predictor.
