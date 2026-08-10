# Migrating the Tinker Sim deployment to a remote server

This guide compiles the full migration procedure for the **tinker-sim 6.0.1**
whole-robot Isaac Sim deployment (`/home/tinker/tinker-sim/6.0.1` on the
source host) plus its external Humble workspace. It is **source-of-truth
verified against the current tree** on 2026-08-10, not a re-statement of the
README.

Two runtime halves move together:

| Half | Python | Where it runs | What it is |
|------|--------|---------------|------------|
| Isaac Sim runtime | 3.12.13 (uv-managed) | the uv project at `<SIM>/` | this checkout |
| Humble overlay | 3.10 (system Humble) | the external ROS workspace | `tk25_ws` + `<SIM>/ros2_ws` overlay |

---

## 1. Target-machine prerequisites (preflight enforces these)

From `deployment.json` and `tools/tinker_sim_deploy/preflight.py`:

- OS **Ubuntu 22.04** x86_64, glibc ≥ 2.35
- ≥ **100 GB** disk, ≥ **32 GB** RAM (64 GB recommended)
- **NVIDIA RTX** GPU, ≥ **16 GB VRAM**, driver ≥ **595.58.03** (older drivers warn
  and are not release-qualified), NVENC encoder registered
- Python **3.12.13** managed by **uv 0.10.x** (`pyproject.toml` pins
  `>=0.10,<0.11`); install uv 0.10.8+ from its official release
- Fresh shell that has **not sourced `/opt/ros/humble`** for any Isaac-side
  step; the launch boundary refuses to start if `PYTHONPATH`,
  `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
  `ROS_PACKAGE_PATH`, or `LD_LIBRARY_PATH` contains `/opt/ros/` or
  `python3.10` (`tools/tinker_sim_deploy/ros_boundary.py`).

---

## 2. Choose a migration path

There are two supported paths. The **offline bundle** is the only path that
carries the warmed caches + pinned uv + managed Python and is fully offline on
the target; it is what the rest of this guide assumes. The **online path** is
`git clone` of the repo + fresh `scripts/bootstrap --mode online` on the target
and needs full network + the NVIDIA/uv indexes.

### What the offline bundle carries (`tools/tinker_sim_deploy/bundle.py`)

`PROJECT_ENTRIES` (checked-in tree): `README.md`, `release-manifest.json`,
`ros2_ws`, `scripts`, `simulation`, `tools`, `uv.lock`, `validation`.

Plus (all gitignored, so **git clone alone will NOT reproduce them**):

- `.cache/uv/0.10` (populated uv cache) and `.cache/uv-python/3.12.13`
- `.cache/isaac/6.0.1` (warmed Isaac extension/asset cache — required, and
  `prewarm.json` inside it is the launch gate)
- `.deps/IsaacLab` (pinned commit `ffff603eafc6b74264a5261cc0183d6a65390d78`)
- `.deps/IsaacSim-ros_workspaces` (tag `IsaacSim-6.0.1`, commit `dd3eeede…`)
- `.cache/ros-debs/humble` + `.ros-vendor/humble` (vendored ROS Debian artifacts)
- `artifacts/` (generated robot USDs, asset manifest, current.json generation)
- a **bundle-local `bin/uv`** (the exact uv executable, hash-pinned)

It **does not** carry `.venv` (28 GB) — the bundle contains the uv cache +
lock so `uv sync --frozen --offline` rebuilds it deterministically on the
target. It also does not carry `outputs/` (5.1 GB of qualification artifacts;
copy those separately if you want them).

`bundle-create` refuses to package an unverified Isaac Lab tree or an unwarmed
Isaac cache, and requires one verified immutable robot generation
(`artifacts/robot/tinker2/current.json`).

---

## 3. Create the offline bundle (on the source host)

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
set -a; source .deployment.env; set +a

./scripts/tinker-sim preflight            # host checks
./scripts/bootstrap --mode online          # if not already fully provisioned
./scripts/tinker-sim bundle-create /home/tinker/tinker-sim-bundle-6.0.1.tar.gz
```

The bundle is a deterministic reproducible tar.gz with a per-file SHA-256
`checksums.json`. Expect it to be on the order of the cache+artifacts (tens of
GB). If you also want the previous qualification evidence:

```bash
tar -C /home/tinker/tinker-sim/6.0.1 -czf /home/tinker/tinker-sim-outputs.tar.gz outputs
```

---

## 4. Restore on the remote server

```bash
# same absolute path is recommended (see §7 hardcoded paths), but not required
sudo mkdir -p /srv/tinker-sim/6.0.1
sudo chown "$USER" /srv/tinker-sim/6.0.1

# restore requires an EMPTY destination and verifies every file against checksums.json
./scripts/tinker-sim bundle-restore \
  /media/bundle/tinker-sim-6.0.1.tar.gz /srv/tinker-sim/6.0.1

cd /srv/tinker-sim/6.0.1
export PATH="$PWD/bin:$PATH"               # project-local uv, NOT system uv
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
set -a; source .deployment.env; set +a
./scripts/tinker-sim preflight
./scripts/bootstrap --mode offline          # uv sync --frozen --offline, never network
```

`bootstrap --mode offline` runs `uv sync --frozen --offline` from the bundled
cache — it never falls back to the network. If uv's managed-Python bootstrap is
impossible, the Miniconda recovery path exists (`scripts/tinker-sim conda-export`
+ `conda create -n tinker-isaac-recovery python=3.12.13`), but the uv path is
the qualified one.

### Verify the restored runtime

```bash
./scripts/tinker-sim preflight
# smoke: 10k headless PhysX steps + RTX camera/LiDAR + NVENC already ran during
# source bootstrap; on the target, at minimum confirm the launch boundary passes
# and a physics-only launch comes up.
./scripts/launch-isaac --sensor-profile physics-only --scenario empty
```

---

## 5. Provision the Humble side (external workspace `tk25_ws`)

The external workspace is a read-only runtime input to the simulator — it is
**never bundled**. `scripts/launch-humble` and `scripts/build-humble-overlay`
both require `TINKER_WS` (or the launch-file `tinker_workspace:=` override);
there is no host-specific default.

On the target:

```bash
# 1. System ROS 2 Humble (apt) on Ubuntu 22.04.
# 2. The Tinker workspace. tk25_ws is a multi-repo root: clone each sub-repo
#    under src/, run bootstrap.sh, then restore_venvs.sh (see tk25_ws
#    docs/ENV_SETUP.md). The source-lock contract used by the simulator pins
#    exact SHA-256 identities of the external workspace, so the checkout must
#    match what the lock expects or workspace-verify fails.
export TINKER_WS=/home/tinker/tk25_ws          # use the same absolute path as source
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. Build the simulator's Humble overlay (tinker_sim_bridge + interfaces).
#    NEVER raw colcon; the wrapper forces MAKEFLAGS='-j2 -l2' + --parallel-workers 2.
cd /srv/tinker-sim/6.0.1
./scripts/build-humble-overlay

# 4. Verify the external workspace matches the pinned source lock.
cd /srv/tinker-sim/6.0.1
./scripts/tinker-sim workspace-verify --workspace "$TINKER_WS"
```

`workspace-verify` compares SHA-256 source records of the external workspace
against `artifacts/provenance/tinker2-source-lock.json`. If the target checkout
differs (e.g. newer sub-repo HEAD), re-capture the lock on the source host with
`./scripts/tinker-sim workspace-lock --workspace "$TINKER_WS"` before bundling,
or export a fresh artifact generation (`artifact-export`). The immutable
per-artifact lock is authoritative for that generation.

### Interop notes (domain / DDS / networking)

- Default DDS profile is **`local`** = Fast DDS shared memory, no
  `FASTRTPS_DEFAULT_PROFILES_FILE`. For multi-machine runs, every machine must
  use `--dds-profile lan` (`config/fastdds-lan.xml`, UDP-only) and be on a
  network that routes it; restrict streaming (WebRTC) ports to a trusted LAN/VPN
  and use SSH for all other management.
- Same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` everywhere (`25` /
  `rmw_fastrtps_cpp` by default). The Isaac side sets these itself via
  `launch-isaac`; the Humble side must export them before `launch-humble`.

---

## 6. Smoke-test the migrated pair

```bash
cd /srv/tinker-sim/6.0.1
export TINKER_WS=/home/tinker/tk25_ws
export ROS_DOMAIN_ID=25 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TINKER_ACCEPT_OMNIVERSE_EULA=Y

# terminal 1: Isaac
./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \
  --scenario find-and-approach-person --seed 7

# terminal 2: Humble boundary
./scripts/launch-humble
```

A full integrated OMPL qualification gate run:

```bash
SUITE_DIR=outputs/integrated/integrated-ompl-seed-7
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage D
```

---

## 7. Hardcoded absolute paths and the cleanest-deploy rule

The tree is *relocatable* (launch wrappers derive the project root from their
location; `TINKER_SIM_ROOT` / `TINKER_WS` are the explicit overrides), **but**
there are hardcoded `/home/tinker/...` references baked into committed files
(2026-08-10 scan):

| Path | Where it appears |
|------|------------------|
| `/home/tinker/tinker-sim/6.0.1` | 32 refs — config, tests, integration contracts, validation |
| `/home/tinker/tk25_ws` (and `.../src/tk25_manipulation`) | 9+6 refs — source-lock json, provenance, launch configs |
| `/home/tinker/tk25_ws/install/setup.bash` | 4 refs — launch/config files |
| `/home/tinker/tk25_ws/tkbuild` | 2 refs |

**Rule:** the cleanest deployment is to use the **same absolute paths on the
target** — `/home/tinker/tinker-sim/6.0.1` and `/home/tinker/tk25_ws`, same
user `tinker`. If you must relocate:

1. Restore the bundle to the new `<SIM>` path — runtime derivation handles it.
2. Use `TINKER_WS` for the external workspace; fix the committed
   `/home/tinker/tk25_ws/install/setup.bash` references in the sim tree's
   config/launch files.
3. Re-capture the external-workspace source lock (`workspace-lock`) and re-export
   the tinker2 artifact (`artifact-export`) so `workspace-verify` + the
   per-artifact locks agree with the new path — otherwise qualification fails
   closed.

---

## 8. What is NOT migrated by the bundle (do these manually)

- **`.venv` (28 GB)** — rebuilt by `uv sync --frozen --offline` from the bundled
  cache; never copy it.
- **`outputs/` (5.1 GB)** — qualification artifacts; copy separately if wanted.
- **Robot calibration** — `simulation/calibration/tinker2-missing.json` must be
  cleared / replaced, or qualification fails explicitly by design.
- **Controller gains / sensor & base noise** — synchronized robot calibration is
  a per-machine step; do not carry them blind.
- **AnyGrasp license (if used by the external workspace)** — node-locked; budget
  re-registration lead time.
- **FoundationStereo TRT engines (tk26_vision)** — GPU/TRT-locked, must be
  re-exported on the target, not copied.
- **Isaac asset cache** — *is* carried by the bundle; if you want a shared
  read-only copy instead, set `TINKER_ISAAC_ASSET_CACHE=/srv/tinker-sim-cache/...`
  in `.deployment.env` (optional override only).
- **External `tk25_ws` source** — never bundled; clone + `bootstrap.sh` +
  `restore_venvs.sh` per the workspace's ENV_SETUP.md, then `workspace-verify`.

---

## 9. Reference commands (source-of-truth README sections)

- Online bootstrap: README `## Online bootstrap`
- Offline provisioning / restore: README `## Offline provisioning`
- Portable deployment inputs + source-lock schema: README
  `### Portable deployment inputs`
- Humble boundary + DDS profiles: README `## ROS 2 Humble boundary`
- OMPL overlay build: README `## Integrated OMPL qualification CLI`
  (bounded build: `MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay`)
- Environment facts: `deployment.json`, `pyproject.toml`, `.deployment.env`
