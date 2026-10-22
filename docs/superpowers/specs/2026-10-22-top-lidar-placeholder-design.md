# Top-down Livox placeholder, and CoACD furniture hulls — design

Status: for approval · 2026-10-22 (builder check done)

## Why

The next physical Tinker is being designed with a second Livox Mid-360 mounted
high (about 1.0–1.5 m) and hanging **upside down, tilted 45°**, so it can see
the tops of tables and other furniture. The base Mid-360 sits on the chassis
deck at 0.27 m and only looks up and out. It cannot see a tabletop from above.

The finished URDF for that robot is some way off. Features that need a lidar
that sees tops of furniture have to be built and tested now, so the sim needs a
**placeholder**: something close enough to the planned mount to develop
against. It gets thrown away when the real URDF arrives.

For the placeholder to be useful, the arena has to look like its furniture
from above. Today most furniture collides as boxes or single convex hulls, and
a raycast lidar sees collision shapes, not visual meshes. The user's
requirement is that arena furniture gets **CoACD collision hulls** (convex
decomposition that follows the real shape), and that this happens **before**
the top lidar is useful.

## Goals

1. Every piece of rcw2026 arena furniture collides as a CoACD convex
   decomposition of its visual mesh, in **every** run, not only top-lidar runs.
2. A check proves the CoACD hulls' **top-down projection is still the same**
   as the navigation map's (user requirement).
3. A top-lidar test mode: one simulated Mid-360 at the planned high mount,
   upside down and tilted 45°, publishing a point cloud while the robot drives
   a commanded route.

## Non-goals

- Running both lidars at once. The user said one lidar active at a time is
  fine.
- Nav2 in top-lidar runs. The robot drives a commanded route instead. Nav2's
  scan comes from the base lidar, which is off in this mode.
- A visual or collision model of the new sensor or its bracket. The user said
  the sensor can float, with no collision and no colour.
- Modelling the new robot's body. Only the sensor pose is placeholder.
- An IMU for the top sensor.
- The RTX lidar (visual-mesh fidelity). It needs rendering and is out of scope.
- Changing the maps produced by map-check (`tools/map_check`). Its arenas are
  extruded boxes and CoACD does not apply to them.

## Anti-goals (would make this a failure even if it works)

- Default runs behave differently because of the top-lidar work. With the top
  lidar off, Part 2 must change nothing: the base lidar's ray pattern, mount,
  frame and topic, the `livox360` transform, the robot artifact and the nav
  map stay as they were after Part 1. (Part 1 intentionally changes what the
  base lidar hits, because the furniture colliders change in every run.)
- The navigation map silently changes. The projection check exists to catch
  that.
- Objects placed on tables, desks, shelves or the sofa start getting flung off
  again. That is the reason the box override in
  `backend.py:_apply_arena_support_surface_colliders` exists.
- A tilt that is silently dropped. Today the lidar mount code takes only the
  mount link's **position** (`lidar_rig.py:_resolve_mount`) and ignores its
  rotation. A lidar that looks level when it should look down is the worst
  outcome, because everything downstream would still "work".

## Order

Part 1 (CoACD hulls) ships first, then Part 2 (top lidar). The user chose this
order: the top lidar is not useful until the furniture looks right from above.

---

## Part 1 — CoACD furniture hulls

### Where things stand

- The arena is a content-addressed artifact,
  `artifacts/arena/rcw2026/<id>/`, built by `tools/arena_import.py` from the
  pinned SOBITS world. `current.json` selects the live one.
- `tools/tinker_sim_deploy/arena_convert.py:author_model_colliders` writes
  colliders per furniture model. SDF `<box>`/`<cylinder>` become invisible
  `UsdGeom.Cube` prims. SDF `<mesh>` becomes one `convexHull` over the whole
  visual mesh (a bookshelf becomes one solid block).
- `config/arena-import.json` `collision_overrides` replaces a model's
  colliders with explicit boxes. Today only `rcw26_shelf` uses it
  (`open_shelf`: hand-measured boards and panels).
- At sim start, `backend.py:_apply_arena_support_surface_colliders` switches
  mesh colliders on the five placement-surface models (kitchen table, shelf,
  side table, laundry desk, sofa) to `boundingCube`. A single hull's tilted
  facets flung objects off at landing.
- The AMCL map (`map.pgm` in the arena artifact) is rasterized by
  `tools/tinker_sim_deploy/arena_map.py` from the **declared box colliders**
  (plus a USD AABB for mesh-collider models), sliced to the 2-D scan's height
  band: about 0.27–2.27 m above the floor at 5 cm resolution. It does **not**
  read live colliders, so CoACD cannot change the map by itself.
- `coacd` 1.0.7 is already in `uv.lock`, as an Isaac Sim dependency.
- `arena_import.ReuseFurnitureHooks` can strip and re-collide a published
  artifact's furniture without Kit.

### What changes

- **CoACD for every allowlisted furniture model.** Decompose each model's
  visual mesh with CoACD offline, at arena import, and store the convex pieces
  in the furniture USD as invisible collision meshes, each with a `convexHull`
  approximation (one PhysX convex per CoACD piece). This replaces the SDF box
  colliders and the whole-mesh convex hull. It does not run at sim start.
  - *My call:* explicit `collision_overrides` (the shelf's measured boards)
    **keep precedence** over CoACD. They are already exact, and thin boards
    are where a decomposition is most likely to merge levels. If the builder
    finds CoACD reproduces the shelf boards cleanly, swapping is fine, but
    the shelf must still show separate board levels from above.
  - *My call:* run CoACD with a fixed seed and pinned parameters recorded in
    the artifact, so the import stays reproducible like the rest of the
    pipeline. Picking the parameters is left to the builder.
- **The map keeps its current source.** *My call, following the user's "check
  their top-down projection is still the same":* the declared SDF boxes (and
  the mesh-model AABBs) stay in the import as map-only data. `arena_map.py`
  keeps rasterizing from them, so the new artifact's `map.pgm` comes out
  identical to today's. CoACD hulls are physics colliders only. The
  projection check compares the hulls against that unchanged map, so it is
  not circular.
- **Retire the support-surface box override** in
  `backend.py:_apply_arena_support_surface_colliders` for models that now
  carry CoACD hulls. Otherwise it would turn the tables back into boxes
  without saying so. Objects must still settle on all five placement surfaces
  (see Checks).
  - *My call, if CoACD tops still fling objects:* add a thin, flat box
    collider exactly on that model's declared placement surface (the
    rectangle and height in `placement.json`), on top of the CoACD hulls.
    Never fall back to boxing the whole model. A flat plate at the tabletop is
    the tabletop's own shape, so the top lidar sees the same thing either way.
- **Publish a new rcw2026 arena artifact** through the existing pipeline, and
  switch `current.json` only after the checks below pass.

### Checks

1. **Projection check (user requirement).** For each furniture instance,
   compare the top-down outline of its CoACD hulls with the outline the map
   is drawn from: its declared boxes or AABB, placed in the world.
   - **The user's rule:** the hull outline must be the same as the map
     outline, **no bigger and no smaller**. Deviations under **5 mm**, such
     as small corners, are acceptable. Both directions count.
   - 5 mm is far below the map's 5 cm cells, so this is a geometric
     comparison of the 2-D outlines (largest distance between them), not a
     comparison of `map.pgm` cells. `map.pgm` itself must still come out
     identical.
   - *My call:* project only the part of each hull inside the map's scan band
     (about 0.27–2.27 m above the floor), the same slice `arena_map.py`
     uses, so low plinths the map leaves out on purpose don't count as
     differences.
   - The check writes a diff image (hull outline vs map outline per model,
     with bigger and smaller regions in two colours, labelled) and a
     per-model largest deviation in millimetres.
   - Expect failures. The importer's existing mesh-vs-declared-box check
     (`bounds_tolerance_m`) allows 2 cm, four times looser than this rule,
     and the door and sink are exempt from it entirely. Shapes that aren't
     boxes (plants, chair) are likely to come out smaller than their map
     outline.
   - If a model fails because CoACD is legitimately tighter than today's box
     or AABB (for example a round table or an L-shaped desk), that is a
     finding to bring back to the user, not something to loosen the check
     for.
   - Any failing model blocks the `current.json` switch until the user has
     seen the diff and decided. This is a planned checkpoint with the user,
     partway through Part 1.
2. **Placement still holds.** A live check drops a test object onto every
   declared placement surface (kitchen table top, shelf plate, side table
   top, laundry desk top, sofa seat). Each object must come to rest on the
   surface, not get flung off or sink through. This guards the problem the
   box override fixed.
3. **Speed.** Measure real-time factor before and after on the standard
   navigation-parity run in rcw2026, same physics rate and lidar scale, and
   report both numbers. More convex pieces mean more shapes per lidar ray,
   so a drop is expected. There is no pass threshold; the numbers go to the
   user.

---

## Part 2 — Top-lidar test mode

### The mount

- Pose is relative to `base_link`, and every value is a setting, because the
  mechanical design is not final:
  - height: 1.0–1.5 m above the floor. *My call:* default **1.2 m above the
    floor**, i.e. `z = 1.2 − 0.0775` in `base_link`, using base_link's rest
    height as `arena_map.livox_scan_height` does.
  - forward offset: *My call:* default **x = 0.09, y = 0**, directly above
    today's lidar.
  - orientation: **upside down, then pitched 45° nose-down**. In URDF terms,
    `rpy = (π, π/4, 0)`, i.e. `R = Ry(45°)·Rx(180°)`.
- What that orientation sees (*my reading of "upside down at 45°"*): the
  Mid-360's −7°…+52° field flips to −52°…+7°. With the 45° nose-down tilt,
  straight ahead it covers 38° below horizontal through straight down (7°
  past vertical). Behind, it covers −7°…+52°, which is mostly the robot's own
  mast and arm and is removed by the self-filter. From 1.2 m, the forward
  edge reaches a 0.75 m tabletop about 0.6 m ahead and the floor about 1.5 m
  ahead.
- The sensor floats: no link mass, no collision, no visual. *My call:* it is
  **not** a new robot artifact. `robot.urdf` and `robot.usd` are unchanged.
  The mount is a sim-side setting, and the frame comes from a static
  transform, the same way `livox360` is published today. That keeps the
  checksummed `tinker2` artifact, the canonicalizer and the nav map
  untouched.

### The sensor

- The same Mid-360 raycast lidar as today (`lidar_rig.RaycastLidar`): same
  −7°…+52° × 360° pattern, same channel/column defaults and
  `TINKER_SIM_LIDAR_*` overrides, same sweep, same 0.2 m minimum range, same
  self-filter.
- **The rig must apply the mount rotation.** Today it uses only the
  translation. The fix has to leave the base lidar's output identical (its
  mount has no rotation), and a test must prove the top mount's rays really
  point where the pose says.
- The cloud is expressed in the **tilted** `livox_top` frame, the way a real
  upside-down Mid-360 reports it, and the static transform carries the tilt.
  `Raycast.create` is given only `translations` today, and hits come back in
  `output_frame="SENSOR"`. The Isaac Sim 6.0.1 API also accepts
  `orientations` (wxyz quaternions; see `impl/raycast.py` in
  `isaacsim.sensors.experimental.physics`), so pass the mount rotation
  there. Whether `SENSOR`-frame hits then come back in the rotated frame is
  for the mount-geometry check below to prove. The final result, points in
  `livox_top` plus the static transform, must match it.
- Publishes `sensor_msgs/PointCloud2` on **`/livox/lidar_top`**, frame
  **`livox_top`**, 10 Hz, sensor-data QoS, with the same point layout as
  `/livox/lidar`. *My call on both names*, so nothing confuses it with the nav
  lidar. A static transform `base_link → livox_top` is published from the
  same mount settings that place the rays, from one source so the two cannot
  drift apart.

### One lidar at a time

- A launch-time switch selects the top lidar. With it on, the raycast lidar
  rides the top mount: `/livox/lidar`, `/scan` and Nav2 do not run. With it
  off (the default), nothing about today's runs changes.

### Driving a commanded route

- A small route driver reads a route file (a list of `x, y` map waypoints,
  optional final yaw) and drives `/cmd_vel` along it at a modest speed, then
  stops and reports that it finished.
- *My call:* it steers by **wheel odometry**, `/tracer/odom`, which
  `base_facade` publishes. That node runs independently of Nav2; today it is
  started by `navigation.launch.py` and `gpsr.launch.py`, so the top-lidar
  launch must start it too. `map → odom` is a static transform at the spawn
  pose, taken from the same spawn setting the sim uses (`--spawn` or the
  scenario), because AMCL isn't running. It does **not** read
  `/sim/truth/*`: `simulation/README.md` reserves those topics for the
  evaluator. Over a short route in sim the odometry drift is small. The
  features under test see the same TF tree they would on hardware without
  localization.
- One default route in rcw2026 passes beside each of the five placement
  surfaces, close enough that the top lidar's forward edge covers each
  surface.

### Checks

1. **Mount geometry (no sim).** Given the mount settings, the computed ray
   directions and the published static transform agree, and a forward ray at
   sensor elevation 0° points 45° below horizontal.
2. **Base lidar unchanged (no sim).** With the top lidar off, the base
   lidar's ray origins, directions, time offsets, mount translation and
   rotation, topic and frame are identical to before Part 2. A live run is
   not bit-reproducible, so this is checked on the computed pattern and
   mount, not on live frames.
3. **Sees the tops (live).** Drive the default route. The evaluator side (it
   may read truth) transforms the top cloud into the world frame and
   confirms, for each of the five placement surfaces, that points land inside
   the surface rectangle at its declared height (±3 cm). Also save an rviz
   or rendered view of the accumulated cloud for the user to look at.

---

## Constraints for the builder

- The GPU is shared with other sessions. Coordinate live sim runs and put a
  hard timeout on every live step. CoACD itself runs on CPU at import time.
- Artifacts are content-addressed and reproducible. A new arena artifact goes
  through `publish_asset_artifact`. Never hand-edit a published one.
- Physics rate and lidar scale drive real-time factor more than anything
  else (see `lidar_rig.resolve_lidar_scale`). Keep them the same when
  comparing speed.

## Left to the builder

- CoACD parameters (concavity threshold, max pieces per model), as long as
  the checks pass and the values are pinned.
- How the mount settings are exposed (environment variables, a launch
  argument, a small config file), as long as one source drives both the rays
  and the static transform.
- Whether the top-lidar mode gets its own launch file or a flag on an
  existing one.
- The route file format and the default route's exact waypoints.
- The route driver's control law and speed.
