# Map Check: 2D map → sim arena, and Tinker renders at marked poses

Status: for approval · 2026-10-19 (builder check done)

## Intent

Teammates mark navigation poses in Map Studio. Right now they can't easily
tell whether those positions are correct and usable: whether the robot
actually fits there, doesn't overlap a wall or a piece of furniture, and
faces what it should. On 2026-10-16 this was done by hand: a 2D map was
extruded into a sim arena, and Tinker was rendered at each pose in Isaac Sim.
That surfaced a pose whose arm sat 24 cm from a wall, and some naming
mistakes. The arm figure used a stale copy of the arm pose; see Arm pose
below.

Map Check turns that manual work into a page any teammate can use:

1. **Map → arena.** Upload a 2D Nav2 map. See it as a 3D arena, adjust it,
   then save the 2D map where Map Studio looks for maps and the 3D arena
   where the sim looks for arenas.
2. **Check poses.** Pick a saved map. See the poses Map Studio has marked on
   it, add pasted poses from Map Studio on another machine, and render Tinker
   at the chosen poses. Problems are flagged.

## Users and access

- Fellow teammates. They sit at this machine, or reach it through an ssh
  tunnel from their own laptop.
- The server binds to **127.0.0.1 only**, on a fixed port. 9081 is my call:
  Map Studio is on 9080 and 9090 is taken.
- Uploads come from the teammate's own browser, so a file on their laptop
  arrives through the tunnel. The page never reads arbitrary server paths
  for uploads.
- No login, as with Map Studio.

## Goals

- Turn a Nav2 occupancy map into a sim arena that the sim loads with
  `--arena <name>`, with the teammate in control of heights, fill and noise.
- Let teammates judge pose correctness visually, and flag overlaps
  automatically.
- Keep Map Studio light. This is a separate page and a separate server.

## Non-goals

- Editing or writing Map Studio's pose files. This page only reads them. The
  sole way poses leave it is the export the user copies or downloads.
- Editing the 2D map. The erase brush changes the arena only, and the saved
  2D map is the uploaded one, byte for byte. Map cleanup stays in Map
  Studio's editor.
- Spawning or moving the robot in a running sim. The separate "place robot"
  feature for Map Studio is not part of this work.
- Navigation or reachability checks such as path planning, or whether Nav2
  can get there.
- Multi-user accounts or remote (non-localhost) access.

## Anti-goals: failing even if it "works"

- Silently overwriting an existing map or arena.
- Starting a GPU render while a sim run is using the GPUs without the
  teammate explicitly overriding the warning. Shared GPU and CPU skew other
  sessions' sim batteries (RTF).
- Holding a GPU or Isaac process while idle.
- A preview that doesn't match what gets saved. The 3D preview and the saved
  `arena.usd` must come from the same geometry.

## Screen 1 · Map → arena

Mockup: `2026-10-17-map-check/mockups/map-arena-v1.html`. It is live: it
fetches `mapdata.json` beside it and needs the brainstorm companion server
or any static server, plus three.js r128 from the CDN.

Layout: a settings rail on the left, in the order of use (upload → walls →
filled shapes → erase noise → save). The main area holds the 2D map editor
and the live 3D preview side by side, stacked on narrow screens.

1. **Source map.** There are two ways in:
   - **Pick a map Map Studio already has.** A dropdown of `<nav maps dir>`.
     This is how a teammate builds or rebuilds the arena for a map that
     already has marked poses.
   - **Upload a new map** from their own computer: the `.yaml` plus its image
     (`.pgm`, and `.png` is accepted).

   The page shows cells, resolution and origin. The occupancy rule follows
   Nav2 trinary: occupied when `(255−v)/255 > occupied_thresh`, with `negate`
   honored.
2. **Walls.** One height for every occupied cell that isn't part of a filled
   shape. The default is 0.60 m.
3. **Filled shapes.** Every free region fully enclosed by occupied cells is a
   *closed outline* (scipy `binary_fill_holes` minus the occupied cells,
   labeled into regions).
   - Each outline is listed with its id, centre x/y and area. It has a fill
     checkbox and **its own height**.
   - "Fill closed outlines up to N m²" (default 4.0) sets the fill checkboxes.
     The guard exists because a room sealed off by the map would otherwise
     become a solid block.
   - A default shape height can be applied to all outlines.
   - Clicking an outline on the 2D map or in the list highlights it in both
     views.
4. **Erase noise.** A brush (size in cells) paints occupied cells to "erased"
   on the 2D map, and each stroke can be undone. Erased cells are left out of
   the arena only.
5. **Save.** **The arena always has the same name as its map.** That name is
   how Check poses finds the map's Map Studio poses, and how the sim finds the
   arena (`--arena <map name>`).
   - **Picked Map Studio map:** Save writes only the arena, under the map's
     name. If that map already has an arena, the save becomes a **new arena
     version** that `current.json` points to. Older versions are kept and
     never overwritten. The 2D map is not touched.
   - **Uploaded map:** a name is required, defaulting to the file's stem. If
     the name already exists in Map Studio's maps folder or in the sim's
     arenas, the page refuses and asks for another name; nothing is
     overwritten. *My call:* if the uploaded files are byte-identical to an
     existing Map Studio map, the page says so and offers to switch to that
     map (the picked-map path) instead of refusing outright.
   - The mockup shows the old "refuse" message for an existing map. This
     section supersedes it.

**3D preview:** drawn in the browser from boxes (three.js or equivalent),
with orbit, pan and zoom. It updates instantly as settings change. It uses no
GPU-side Isaac and needs no round trip, apart from fetching the grid once.
The boxes come from the same row-run rectangle merge as the saved arena
(`prototypes/build_arena.py: merge_rects`). Running that merge in both the
browser and the server is fine as long as both produce identical rectangles
(a test should pin this). The alternative is for the server to return the
box list.

**What gets saved:**

| What | Where | Notes |
|---|---|---|
| 2D map (uploads only) | `<nav maps dir>/<name>.yaml` + `<name>.pgm` (or the original image extension) | The uploaded bytes. The yaml `image:` field is rewritten to the new file name if the name changed. `<nav maps dir>` = `/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps` (source tree, configurable). A picked Map Studio map is never rewritten. |
| 3D arena | `artifacts/arena/<name>/<version dir>/` holding `arena.usd`, `map.yaml`, the map image, `manifest.json` and the box list used for overlap checks, plus `artifacts/arena/<name>/current.json` → newest manifest | Must resolve through `validation/run_sim.py: resolve_arena_artifact`, i.e. `--arena <name>` gets the current version. The manifest marks the arena as built by Map Check. It records the source map name and sha256, and the build settings (wall height, per-shape fill and height, erased cells), so the build is reproducible. Version dirs are never overwritten. How closely the manifest follows `arena-artifact-1`, and how versions are named, is left to the builder. |

- Map Studio reads the source-tree folder, so a new map shows up there
  immediately.
- Nav2 stacks read the **installed** copy. After saving, the page tells the
  teammate to rebuild `navigation_bringup` before Nav2 can use the map. The
  install is a symlink install: each file is a symlink into the source tree.
- The arena geometry is USD Cubes with `PhysicsCollisionAPI`, the same form
  as `build_arena.py` uses. Each filled shape keeps its own height. Prim
  names should say what each box is (`wall_###`, `shape<id>_###`).

## Screen 2 · Check poses

Mockup: `2026-10-17-map-check/mockups/check-poses-v7.html`. Its images are
missing outside the brainstorm session; the striped boxes stand in for them.
Dark theme throughout.

**Top: results strip ("Last render").** It sits directly under the header. A
horizontal strip with **its own horizontal scroll bar** holds the overview
first, then one card per rendered pose holding only the views that pose
asked for. **Every image is the same height** (230 px in the mockup) whatever
its aspect ratio. Clicking an image opens a full-screen viewer: ← and → step
through all views, Esc or a click outside closes it. A pose with an overlap
has a red card header with the flag text. The results stay until the next
render.

**Below: the pose table and the paste panel.**

- **Map.** A dropdown of maps that have an arena **built by Map Check**
  (teammate's decision). Arenas built any other way, such as
  `measured-home-20261015` or `rcw2026` with its mesh furniture, are out of
  scope. Their maps get an arena by rebuilding them on Screen 1. Choosing a
  map loads its current arena version and its Map Studio poses from
  `<nav maps dir>/<map>.poses.yaml` (read only; the format is below).
- **Arm pose (per render).** A dropdown of every `arm_pos_*` key in HRI's
  `constants.json` (degrees, joints 1–7). On 2026-10-19 those were
  `arm_pos_navigating`, `arm_pos_orbbec_look`, `arm_pos_point_to`,
  `arm_pos_handover` and `arm_pos_drop`. The default is `arm_pos_navigating`.
  - The file is
    `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/HRI/constants.json`.
    That's the copy HRI's own code loads (`HRI/config.py`).
  - Two stale copies exist with different values: `HRI/HRI/constants.json`
    and the `tk25_decision-hri-following` worktree. Don't use them. The
    2026-10-16 renders used the stale values by mistake.
  - The path is configurable.
- **Table columns:**
  - Render (include)
  - Pose name
  - x, y, yaw
  - Behind (behind-left close-up)
  - Top (top-down close-up)
  - Head cam (with per-pose pan and tilt)
  - Remove
- **Column header checkboxes** select or deselect that column for every row
  **currently shown**, after the filter. They show an indeterminate state
  when only some rows are on. The view columns act only on included rows.
- The Head cam header also has pan/tilt boxes and a "set" button that apply
  those angles to every shown, included row that has Head cam on.
- **Filter box** (substring on name). **Shift-click** on a row's Render box
  sets a range.
- Rows not included are dimmed, and their view checkboxes are disabled.
- **Groups:** "From Map Studio (n)" and "Pasted (n)", each showing "k of n in
  render". Pasted has "clear".
- **Remove (✕):**
  - On a pasted pose it deletes that pose.
  - On a Map Studio pose it removes it from this page's list only (the file
    is untouched). The group heading then shows "n removed · restore".
- **Name clash:** a pasted pose whose name matches a Map Studio pose on this
  map is compared after stripping `POS_` and lowercasing. Both rows stay, and
  each gets a chip "same name · d cm apart".
- **Paste panel:** a textarea plus "Add to list". It accepts Map Studio's
  "Copy all as code" text: one or more
  `POS_<NAME> = PoseStamped(... Point(x=…, y=…, z=…) ... Quaternion(x=…, y=…, z=…, w=…))`
  blocks, with or without the import lines. Yaw comes from the quaternion.
  Pasted names are kept as written (`POS_…`). Anything unparsable is reported
  per block, and the valid blocks are still added. Pasted poses usually come
  from Map Studio on **another machine** for the same map, so the page can't
  verify that they belong to it. The teammate's map choice is trusted.
- **Export list…:** a dialog that offers **both** Map Studio formats:
  - code, identical to Map Studio's `pose_code` output and imports;
  - a `.poses.yaml` document (`map:`, `frame_id: map`, and
    `poses: "name": {x: %.4f, y: %.4f, yaw: %.4f}`).

  It can export everything listed or only the poses ticked for render, and
  offers Copy and Download. Pasted names are exported lowercased without
  `POS_`. The dialog warns when two exported poses share a name.
- **Render bar** (sticky bottom):
  - "Next render: n poses · m views incl. overview · about t s";
  - the GPU status chip;
  - the Render button, disabled with nothing ticked.
- **Overlap flags** appear in the list (red chip) as soon as a pose is listed
  or the arm pose changes, before any render. See below.

**Views per render:**
- **Overview:** the whole arena top-down, with every rendered pose labeled,
  as in the 2026-10-16 renders. Labels are placed so they don't collide, with
  leader lines.
- **Behind-left close-up:** camera behind and to the left of the robot,
  looking past it at what it faces (`prototypes/render_tinker_solo.py`). The
  other rendered robots are hidden from it.
- **Top-down close-up:** straight down on the robot and its surroundings.
- **Head camera:** from Tinker's head camera with the pose's pan
  (`pan_joint`, ±180°) and tilt (`tilt_joint`, −30°…+90°, URDF limits).
  Intrinsics come from `simulation/sensors/hardware-parity.json` →
  `head_camera` (1280×720, 90° HFOV, mounted at
  `head_camera_color_optical_frame`). The UI must state which tilt direction
  is "down", verified against a render.

## Shared bench

Teammates may use the page at the same time, and they share **one bench**
(teammate's decision). Everyone sees the same pose list and the same last
render.

- **State on the server**, kept per map:
  - pasted poses;
  - removed Map Studio rows;
  - include ticks;
  - per-pose views and pan/tilt;
  - the arm pose choice;
  - the last render: its images and flags, plus who started it and when.
- It survives page reloads and server restarts (stored on disk). A change
  made by one teammate appears in the others' open pages without a reload.
  How (polling, SSE, WebSocket) is left to the builder.
- *My call:* the selected **map** is per browser. Two teammates can look at
  different maps, and each map has its own shared bench.
- *My call:* conflicting edits are last-write-wins per field. No locking.
- Render jobs are shared too. Everyone sees the running job's progress and
  the queue (or "busy"), and the result replaces that map's last render for
  everyone.
- Map Studio poses are re-read from the file whenever the bench is shown, so
  rows follow Map Studio edits. A removed row stays removed by name, and a
  pose deleted in Map Studio simply disappears.

## Rendering and the GPU

- Renders run Isaac Sim 6.0.1 from this repo's `.venv`, headless,
  RaytracedLighting. One Isaac process per render job, which exits when the
  job is done, so no GPU is held while idle.
- The robot is the current `artifacts/robot/tinker2/current.json` artifact,
  posed kinematically:
  - URDF forward kinematics writes each link's transform
    (`prototypes/tinker_fk.py`);
  - physics is disabled on the copies;
  - the base sits at the grounded height.
- Before a render the server checks whether a sim run is using the GPUs. That
  means any process on either GPU (nvidia-smi compute apps) or a running
  `run_sim.py` / `gpsr-stack` / Isaac process that isn't Map Check's own.
  - If one is found, the job **doesn't start**. The page shows what it found,
    and the Render button becomes "Render anyway…", which needs an explicit
    confirm.
  - The override is per click and isn't remembered.
- One render job at a time. A second request while one is running is queued
  and shows its place in the queue, or is refused with "busy". Which of the
  two is left to the builder.
- Long renders show progress, with the poses and views done so far. A hard
  timeout kills the Isaac process tree; a hang once held the shared GPU for
  about 2 hours.
- Known GPU limit: about 18 render products with 16 robots crashed the GPU
  (DEVICE_LOST) on 2026-10-16. Batch the work, e.g. at most 6 close-up
  render products per Isaac step, overview separately.
- GPU choice: prefer the GPU with no compute processes, and GPU 1 when both
  are free.

## Overlap flag

The teammate's rule is **overlap only**: flag a pose when **any part of the
robot**, in the selected arm pose, overlaps a wall or a filled shape of the
arena, with a small margin.

- The check is 3D. A link that passes **over** a lower filled shape (an arm
  over a 0.45 m table) isn't an overlap.
- It uses the robot's link geometry (collision shapes, or link bounds) posed
  by FK, against the arena's own box list (walls plus filled shapes, with
  their heights), which is saved with the arena.
- *My call:* the margin is 2 cm, and it makes the check **stricter**: a
  pose is flagged when any part of the robot is closer than 2 cm to a box,
  as if the robot were inflated by 2 cm. The UI states the margin.
- The flag text gives the real geometry, not the inflated geometry:
  - a pose that actually penetrates reads "arm (link3) overlaps wall by
    3 cm";
  - a pose inside the margin without touching reads "base 1 cm from shape 7
    (under 2 cm)".
- No clearance thresholds. Spacing is judged from the renders. Table poses
  deliberately stop with the base front about 24 cm from the furniture ahead.

## Map Studio formats (read and produced)

- Pose file: `<map>.poses.yaml`
  ```yaml
  map: "robocup_home_2026_measured_5cm"
  frame_id: map
  poses:
    "livingroom_destination": {x: 1.2150, y: 1.0520, yaw: -2.2305}
  ```
- Code export (`map_studio/poses.py: pose_code` + `CODE_IMPORTS`): one
  `POS_<NAME.upper()> = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), pose=Pose(position=Point(x=…, y=…, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=sin(yaw/2), w=cos(yaw/2))))`
  block per pose.

## Constraints

- **Code location:** the tinker-sim repo (`/home/tinker/tinker-sim/6.0.1`),
  e.g. `tools/map_check/`. The exact layout is left to the builder.
- **Stack:** the same as Map Studio, decided with the teammate:
  - FastAPI backend;
  - React 18 + TypeScript + Vite frontend, with vitest for frontend tests.

  The backend runs under this repo's `.venv` Python (fastapi is already
  installed there). The Isaac render runs as a subprocess, so the web server
  stays responsive and holds no GPU.
- **Map Studio is not modified.** This page reads Map Studio's maps folder
  and writes only *new* map files there, on Save.
- **Shared tree:** this repo is shared with other sessions. Commit only this
  work's own hunks, and don't touch unrelated modified files.
- **Light on CPU when idle,** because the host CPU is shared with sim runs.

## Decided by the teammate

- Users are teammates checking whether their positions are correct and
  usable. Problems are flagged, not just shown.
- Access is at this machine or through an ssh tunnel. Local uploads must
  work.
- It's its own page; Map Studio stays light.
- Saved files go where Map Studio and the sim already look.
- A GPU in use means warn and don't start, with a manual override.
- The teammate sees the 3D result and adjusts settings before saving.
- The preview is live in the browser, as in the mockup.
- Heights are per filled shape. An erase-noise brush is included.
- On a name that already exists, refuse and ask for a rename.
- Map Studio's poses for the chosen map are shown; pasted poses join them.
  Each pose has an include toggle, with select/deselect-all at the top of
  each column.
- Views are chosen per pose (behind, top, head cam), and the head cam has a
  pan and tilt per pose.
- The arm pose is picked per render, from HRI's named poses.
- Pasted poses usually come from another machine. On a same-name clash, show
  both and mark the distance.
- Flags are for overlap only.
- Poses can be removed from the list. Export offers both formats.
- Dark theme. Results on top, in a horizontal strip with its own scroll bar,
  all images the same height, click to enlarge.
- Code lives in tinker-sim. The stack is the same as Map Studio.
- The arena is named after its map. Screen 1 can start from a map Map Studio
  already has, and rebuilding makes a new arena version.
- Check poses works on Map Check arenas only.
- One shared bench: everyone sees the same list and the same last render.

## My calls (check these)

- Port 9081, 127.0.0.1 only.
- The page reads Map Studio's poses file directly from the maps folder, not
  through Map Studio's API, so it works when Map Studio isn't running.
- The overlap margin is 2 cm and makes the check stricter (an inflated
  robot), while flag text reports the real distance. The check is 3D, and
  flags show before render.
- An upload byte-identical to an existing Map Studio map is offered as that
  map instead of being refused.
- The selected map is per browser, and the bench is shared per map.
  Conflicting edits are last-write-wins.
- The arm-pose default file is the one HRI's code loads, and the dropdown
  lists every `arm_pos_*` key.
- One Isaac process per render job, no warm worker. Startup costs about
  40 s per job, but no GPU is held while idle.
- GPU-busy means any compute process on either GPU or a known sim process.
  Prefer GPU 1.
- The erase brush affects the arena only; the 2D map is saved as uploaded.
- The yaml `image:` field is rewritten when the saved name differs from the
  upload.
- The "Fill up to N m²" default is 4.0. The default heights are 0.60 m for
  walls and shapes.

## Left to the builder

- Module layout, API routes and the job model (queue vs refuse while busy).
- How the shared bench is stored and pushed to open pages.
- Whether rectangle merging runs in the browser, on the server, or both, as
  long as the preview equals the saved arena.
- How closely the arena manifest follows `arena-artifact-1`, and the folder
  naming under `artifacts/arena/<name>/`.
- Camera framing details, label placement, image sizes, the time estimate
  formula.
- How to source the robot link geometry for the overlap check.

## Prior work and references

- `2026-10-17-map-check/prototypes/`. These were used on 2026-10-16 and are
  working but unpolished:
  - `build_arena.py`: occupancy → filled mask → rectangles → USDA cubes, and
    the artifact layout;
  - `tinker_fk.py`: URDF forward kinematics, checked against the authored USD
    to 3e-6 m;
  - `render_rooms.py`: Isaac scene setup (floor, materials, lights), robot
    posing, top/oblique cameras, non-overlapping labels with leader lines;
  - `render_tinker_solo.py`: the behind-left close-up and the base-overlap
    check against the filled mask;
  - `robot_extent.py`: FK-posed robot bounds in pure pxr, without Isaac.
- Example output: `artifacts/arena/measured-home-extruded-0p6m-20261016/`,
  i.e. the arena and `renders/` (full, rooms, poses).
- Map Studio: `/home/tinker/tk25_ws/src/tk26_navigation/src/map_studio/`. The
  pose store, the code export, and the UI conventions to match.
- Sim arena resolution: `validation/run_sim.py: resolve_arena_artifact`,
  `simulation/tinker_sim_isaac/backend.py: resolve_arena_inputs`.
