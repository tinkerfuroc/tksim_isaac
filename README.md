# Tinker whole-robot simulation deployment

This is a standalone deployment project for x86_64 Tinker simulation servers.
It neither imports nor modifies the surrounding ROS workspace. For production,
copy this directory to a dedicated location such as
`/srv/tinker-sim/6.0.1`; do not overlay it on a sourced ROS workspace.

## Qualified baseline

| Component | Pinned value |
| --- | --- |
| Host | Ubuntu 22.04, x86_64, GLIBC 2.35 or newer |
| NVIDIA driver | 595.58.03 or newer |
| Python | CPython 3.12.13 |
| uv | 0.10.8 (0.10 lock format) |
| Isaac Sim | 6.0.1.0 |
| Isaac Lab | v3.0.0-beta2.patch1, commit `ffff603eafc6b74264a5261cc0183d6a65390d78` |
| Isaac Lab PhysX extension | 1.1.3 from the same pinned checkout |
| Isaac Lab Newton interface shim | 0.13.6 from the same pinned checkout |
| PyTorch | 2.11.0, CUDA 12.8 wheel |
| TorchVision / TorchAudio | 0.26.0 / 2.11.0, CUDA 12.8 wheels |
| Pillow | 12.2.0 |
| IsaacSim ROS workspaces | tag `IsaacSim-6.0.1`, commit `dd3eeede7912755996a18f4884285d9f50843f79` |
| ROS simulation interfaces | Humble 1.4.0, vendored Debian artifact |
| Topic-based ros2_control | Humble 0.2.0, vendored Debian artifact |

Provisioning requires at least 100 GB free disk, 32 GB RAM, and an RTX GPU
with 16 GB VRAM. Sixty-four GB RAM is the supported whole-robot target.
Drivers older than 595.58.03 produce an explicit experimental warning and are
not release-qualified.

`uv.lock` is authoritative for Python. The static `release-manifest.json` records the
lock, project, uv executable, managed Python, and Isaac Lab Git tree hashes.
`artifacts/provenance/ros-debs.json` is authoritative for the isolated Humble
runtime additions.
Each completed deployment also writes a machine-readable report under
`reports/`.

## Online bootstrap

Install uv 0.10.8 from its official release, start a fresh shell that has not
sourced `/opt/ros/humble`, then:

```bash
cd /srv/tinker-sim/6.0.1
cp deployment.env.example .deployment.env
# Read NVIDIA's Omniverse EULA, then set TINKER_ACCEPT_OMNIVERSE_EULA=Y.
set -a
source .deployment.env
set +a

./scripts/tinker-sim preflight
./scripts/bootstrap --mode online
```

EULA acceptance is never set by a script or repository default. Bootstrap
fails until the deployment configuration explicitly contains
`TINKER_ACCEPT_OMNIVERSE_EULA=Y`. After that explicit gate passes, bootstrap
sets Isaac Kit's required `OMNI_KIT_ACCEPT_EULA=YES` only for its child
processes.

Bootstrap is idempotent. It:

1. validates the OS, architecture, GLIBC, disk, RAM, GPU, VRAM, driver, and
   registered NVENC encoder;
2. installs CPython 3.12.13 into the project-local managed-Python directory;
3. fetches and verifies the exact Isaac Lab commit without patching it;
4. runs only `uv sync --frozen` and reconstructs a temporary environment with
   `uv sync --frozen --offline` to audit cache completeness;
5. runs Isaac's compatibility checker, 10,000 headless PhysX steps, requires
   non-empty RTX camera and LiDAR output, performs a real NVENC encode, and
   starts the headless WebRTC experience;
6. warms core, ROS, RTX-sensor, and URDF extension caches; and
7. vendors the exact ROS Debian artifacts and pins NVIDIA's IsaacSim ROS
   workspace without installing either into the host; and
8. writes a deployment report containing runtime hashes, package versions,
   ROS domain settings, and test results.

`--skip-preflight`, `--skip-validation`, and `--skip-prewarm` exist for tooling
tests only. A deployment made with any of them is not release-qualified.

No system CUDA toolkit is read or installed. CUDA runtime libraries come from
the Isaac Sim and PyTorch wheels.

## ROS 2 Humble boundary

There are two process environments:

- Isaac runs from this uv environment with Python 3.12 and Isaac's internal
  Humble libraries.
- Tinker gateways, Nav2, MoveIt/cuMotion, vision, decision, audio, VLA, FJT,
  and hardware-facade nodes run under system Humble with Python 3.10.

`scripts/launch-isaac` refuses to start if `PYTHONPATH`,
`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
`ROS_PACKAGE_PATH`, or `LD_LIBRARY_PATH` contains `/opt/ros/` or
`python3.10`. It then locates Isaac's internal
`isaacsim.ros2.* / humble / lib` directory and sets the shared ROS domain and
Fast DDS implementation. The default `--dds-profile local` leaves Fast DDS
shared memory enabled. `--dds-profile lan` selects the committed UDP-only
profile and must be used by every machine in that run.

Only the standard interfaces listed in `deployment.json` cross the Isaac DDS
boundary. NVIDIA's `isaacsim.ros2.sim_control` owns the standard
`simulation_interfaces` lifecycle API. The external overlay defines only
evaluator truth messages and hardware/task adapters; it must never be installed
in this uv environment.

Navigation is the first live module integration.  It reuses the existing
Tinker Humble Nav2/localization implementation without modifying the source
workspace, with a hardware-parity base facade, wheel-derived odometry, Livox
scan adapter, AMCL initialization, TF ownership guard, and public simulation
control gateway.  The exact build, two-terminal launch, interfaces, readiness
checks, and live results are documented in [`integration/NAVIGATION.md`](integration/NAVIGATION.md).

Example launch profiles:

```bash
./scripts/launch-isaac --sensor-profile physics-only --scenario empty
./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \
  --scenario find-and-approach-person --seed 7
./scripts/launch-isaac --sensor-profile streaming --dds-profile lan
```

Streaming uses Isaac's headless WebRTC experience, explicitly disables GPU
physics, and uses a local process lock to
enforce one viewer per simulator instance. Restrict its ports to a trusted LAN
or VPN. Use SSH for installation, process management, logs, tests, and bags.

## Offline provisioning

Run a complete online bootstrap first. Add generated robot USDs and their
checksums to `artifacts/asset-manifest.json` using the tracked portable template
at `config/asset-manifest.example.json`; do not reference the surrounding ROS
workspace. Both generated-USD and warmed asset groups must be non-empty, present
on disk, and hash-correct. Whole-robot bundles also require a verified immutable
`artifacts/robot/tinker2/current.json` generation. The only intentional
artifact-free mode is the explicitly selected `bundle-restore --profile
physics_only` validation profile; whole-robot creation and restore fail closed
when the verified generation is absent.
Then:

```bash
./scripts/create-offline-bundle artifacts/tinker-sim-6.0.1.tar.gz
```

Bundle creation first creates a fresh temporary environment using
`uv sync --frozen --offline`. This proves that no package is missing from the
cache. It refuses to package an unverified Isaac Lab tree or an unwarmed Isaac
cache. The deterministic archive contains:

- the lock and environment definition;
- the exact uv executable and managed Python;
- the populated, versioned uv cache;
- the pinned Isaac Lab checkout;
- warmed Isaac caches and generated assets;
- this deployment tooling and ROS contract workspace; and
- a per-file SHA-256 manifest.

On an offline server:

```bash
mkdir -p /srv/tinker-sim/6.0.1
./scripts/restore-offline-bundle \
  /media/bundle/tinker-sim-6.0.1.tar.gz /srv/tinker-sim/6.0.1
cd /srv/tinker-sim/6.0.1
export PATH="$PWD/bin:$PATH"
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/bootstrap --mode offline
```

Restore requires an empty destination and verifies every file before use.
Offline bootstrap runs `uv sync --frozen --offline` and never falls back to
the network.

## Miniconda recovery only

Miniconda may create a Python 3.12.13 environment if uv's managed Python cannot
be bootstrapped. It is not a second dependency definition:

```bash
./scripts/tinker-sim conda-export artifacts/requirements-from-uv-lock.txt
conda create -n tinker-isaac-recovery python=3.12.13
conda activate tinker-isaac-recovery
python -m pip install -r artifacts/requirements-from-uv-lock.txt
```

Always regenerate the requirements file from the committed lock. Never edit
or maintain it independently.

## Simulation contract and rollout

`contracts/simulation.yaml` defines the standard `simulation_interfaces`
surface and evaluator-only `/sim/truth/*` topics. There are deliberately no
custom `/sim/control/*` or `/sim/scenario/*` aliases.
`simulation/tinker_sim_core` provides:

- a Gazebo/Isaac backend protocol;
- deterministic single-owner command arbitration;
- hardware-parity versus hidden-truth separation; and
- postcondition scoring for person approach, pick/deliver/place, and reception
  seat assignment.

The parity profile is the only input to navigation, perception,
manipulation, decision, and VLA software. Only the evaluator may receive
truth. The scenario evaluators reject a task server's claimed success when the
world postcondition is false.

The Tinker 2 USD/URDF/map export, navigation gateway, external FJT controller,
xArm/gripper/pan-tilt facades, deterministic scenario orchestration, and audio
fixtures are implemented as content-addressed artifacts under `artifacts/`.
The artifact exporter derives only the canonical URDF metadata needed by the
strict planning contract: it adds the zero `world -> base_link` mount, names
the existing `base_link -> link_base` mount, and records a state-only
`drive_joint` entry. The USD is copied byte-for-byte and remains the simulator
physics artifact. The canonical URDF bytes and canonicalizer algorithm version
are part of the artifact identity; the manifest, immutable per-artifact
source lock, and `current.json` record the complete generation and all payload
hashes. Publication fsyncs files/directories, atomically claims the immutable
artifact directory, and replaces `current.json` as the sole commit point; a
crash before that replacement can leave only an ignored orphan generation and
cannot mix a pointer with a mutable root lock. Export publication is serialized
by a nonblocking inter-process lock; a normal concurrent exporter fails without
recovering the active exporter's staging directory. The exporter reads the
external workspace once, derives the immutable lock in memory, and only accepts
an already captured shared lock when it exactly matches; a failed export never
creates or rewrites that shared lock. `workspace-lock` is the separate explicit
atomic capture operation.

### Portable deployment inputs

The simulator checkout is relocatable. Set `TINKER_SIM_ROOT` when invoking a
launch wrapper from an installed/copy deployment; launch files otherwise derive
the project root from their installed/source package location. Set `TINKER_WS`
to the external Humble workspace, or pass `tinker_workspace:=...` explicitly.
There is no host-specific workspace default. `scripts/launch-humble` fails
clearly when `TINKER_WS` is absent. The external workspace is read-only runtime
input and is never bundled as an absolute provenance path.

The source lock schema is portable and exact: it contains robot `tinker2`, a
SHA-256 source identity derived from the fixed source contract plus the ordered
relative path/size/SHA-256 records, and no fake repository revision. The
immutable per-artifact lock is authoritative for that generation; manifest
canonicalization, USD provenance, source records, payload hashes, and the
content-addressed identity are cross-checked against it by every consumer.
Use the project-managed interpreter, for example
`TINKER_WS=/path/to/tk25_ws ./.venv/bin/python tools/deploy.py artifact-export`,
rather than editing an artifact directory in place.

Vision, manipulation, decision, and VLA vertical slices still require live
qualification and must not be represented as release-qualified. Controller
gains and sensor/base noise still require synchronized robot calibration;
qualification fails explicitly while
`simulation/calibration/tinker2-missing.json` remains uncalibrated.

### OMPL overlay acceptance contract

`integration/ompl-overlay-contract.json` is the deterministic acceptance
contract for the reviewed OMPL overlay (Tasks 3-7): canonical model-bundle
producer/preflight, integrated eight-joint state contract, atomic fixture
PlanningScene adapter, staged integrated OMPL overlay with typed integrated
readiness, and the deterministic OMPL plan-only smoke.  The contract is
canonical JSON (schema version 1, sorted keys, minimal separators) with exact
repository/commit identities, the production overlay
(`mobile_bringup` / `manipulation_planning_task_only.launch.py`, 18-argument
contract, literal-false `use_cumotion_*` compatibility values), the provider
manifest (raw + canonical identities, distinct persistent/one-shot/controller/
publisher classes), the model-bundle schema/artifact hashes and full eight-link
touch set, the typed action/service/topic contract (including
`/isaac_joint_commands` depth 50 and the external future
`/tinker_integrated_gate_executor` publisher ownership), fixture/scenario
identities, Task 6/7 evidence, and build commands (`tkbuild` /
`scripts/build-humble-overlay`, never raw colcon).  It does not itself prove
live OMPL or authorize cuMotion.  `tests/test_provenance.py` recomputes every
derived hash/contract from the real source and fails on mutations.  The two
repository-local source-lock files are Task 9 only.  See
`integration/MANIPULATION.md` for the operator workflow.

## Developer verification

The non-GPU tests run with the Ubuntu system Python:

```bash
python3 -m unittest discover -s tests -v
uv lock --check
./scripts/build-humble-overlay
```

The focused OMPL-overlay provenance suite runs under the simulator venv.  ROS
plugin discovery may auto-load the Humble `launch_pytest` plugin, which can fail
collection with `ModuleNotFoundError: No module named 'lark'` on hosts that do
not provide that dependency; disabling plugin autoload is the defensive
reproducible invocation (it does not claim collection necessarily fails on any
specific host):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/python -m pytest -q tests/test_provenance.py
```

This does not hide the pre-existing pinned-uv environment failure
(installed `uv 0.12.0` vs pinned `uv 0.10.8`), which is reported visibly.

Release qualification additionally requires a clean server, the full online
and offline bootstrap paths, ROS cross-process contract tests, RTX camera and
LiDAR initialization, and a 60-second continuous WebRTC connection using
NVIDIA's native Ubuntu Omniverse Streaming Client (never a downloaded browser).

References:

- [Isaac Sim Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS integration](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Lab installation](https://isaac-sim.github.io/IsaacLab/develop/source/setup/installation/index.html)

## Changelog

- 2026-08-02 (integrated qualification Task 2 fix round 2): Strengthened the
  semantic source checks.  `validation/integrated_static_contracts.py` replaces
  comment-stripped string-presence C++ scans with a deliberately scoped
  lexical/structural layer: comments, string/character and raw-string literals
  are sanitized (preserving braces/newlines), conditional-preprocessor
  directives inside load-bearing function bodies are rejected (so `#if 0` /
  `#ifdef` dead-code anchors cannot satisfy a check), and each inspected
  function (result builders, `GraspNode::~GraspNode`,
  `GraspNode::clean_planning_scene`, `GraspNode::move_straight`,
  `run_post_close_pick`, `coordinator_main`, `~MotionRuntime`) is located by
  its actual signature and brace-matched.  The Sim/Hardware `execute_lift`
  boolean is bound to its own `ExecutionProfile` branch, the destructor
  deadline/shutdown/join/state-validity reset are bound to the destructor body,
  and each task result builder must write exactly its `.action` schema fields.
  The authoritative overlay `model_bundle.production_source_commits` is bound
  to immutable Git blobs (40-hex commit existence, blob digest, manipulation
  ancestry vs `implementation_head`; external `tk25_basic` via exact recorded
  commit/blob only) and drives both the model-artifact binding and the
  pinned-prerequisite source-identity check.  `fixture-ownership` now requires
  the overlay `scenarios` key set to equal the configured C/D/E set exactly.
  `source_lock_manifest.py` closes the observer gaps: qualification-tooling
  requires clean mode with empty evidence, `authorization.report_path` must
  resolve (no symlink/canonical escape) to a regular file inside the
  repository, `_normalize_posix` rejects `..` and canonicalizes absolute
  repository paths, the stale mtime-fallback docstring is removed, and the
  output manifest is written/fsynced/replaced exactly once from a single
  pre-write timestamp.

- 2026-08-02 (integrated qualification Task 2 fix round 1): Bound Gate B to
  real artifacts.  `validation/integrated_static_contracts.py` no longer reads
  the fabricated `qualification/records/*` layer: every production source
  (SRDF `_xarm7_macro.srdf.xacro`, `config/xarm7` and `config/xarm_gripper`
  `controllers.yaml`, the pick_and_place C++ runtime and result writers, the
  `.action` schemas, and the selected launch) is inspected as an immutable Git
  blob at the production `implementation_head` from the produced source-lock
  manifest.  The model-fingerprint check recomputes
  `structural_fingerprint == sha256(canonical bundle contract)`, binds every
  recorded model artifact SHA-256 and the `production_source_commits` blobs,
  and proves the ordered `xarm_gripper` members plus `link_tcp` end-effector
  parent.  `fixture-ownership` inspects exactly the 17 configured scenarios
  (C/D/E) and derives owned ids from `planning_scene.objects` in declared
  order versus `integrated.expected_scene.owned_ids`.  The transport-contract
  check reads `typed_contract.runtime_contract_sha256` from its sibling
  location, recomputes the canonical full runtime mapping, and preserves the
  exact public one-key mapping.  Provider checks recompute the
  `provider_manifest_sha256` canonical self-hash and raw bytes and reconcile
  the provider executable set with the overlay.  `source_lock_manifest.py`
  gained closed policy/authorization schemas, the resolved lock-commit scope
  rule (`{policy_path, docs/acceptance.md}`), the qualification HEAD exact-match
  requirement, authorization report existence/predate plus bogus/valid
  authorization-commit checks, canonical repository-relative `policy_path`,
  required ISO `started_at`, and a stable output-freshness boolean.  The
  static checker emits canonical finite JSON with fsync + atomic replace for
  `static-contract.json`, `model-fingerprint.json`, and
  `source-identities.json`, and propagates `evidence-invalid` /
  `verified-fail` / `verified-pass` per F4.  The real-root post-fix Gate B
  capture remains `evidence-invalid` solely because the future
  qualification-tooling policy is intentionally absent; eight of the nine
  static checks pass against the real simulator/production roots and only
  `source-identities` fails on that absent policy.

- 2026-08-02 (integrated qualification Task 2): Added the offline Gate B
  static-contract closure.  `validation/source_lock_manifest.py` observes the
  three immutable source-lock authorizations
  (`simulator_overlay` / `production` / `qualification_tooling`) with
  non-self-referential Git-history resolution, exact `LC_ALL=C` raw
  status/diff/index/untracked evidence, a canonical path-sorted untracked
  manifest, an atomic fsync+`os.replace` output manifest, and attempt
  freshness.  `validation/integrated_static_contracts.py` runs the nine
  semantic static checks (model, controllers, providers, fixture ownership,
  action lifecycle, scene/collision safety, source identities, transport
  contract) against the produced three-entry manifest.
  `simulation/qualification/integrated-ompl.json` now names exactly the three
  `source_lock_policies`.  The real qualification-tooling policy is
  intentionally absent during Task 2, so a real-root Gate B capture honestly
  returns `evidence-invalid` until the post-Task-10 lock-only phase; no
  existing source-lock policy was created or modified.
- 2026-08-02 (fix round 2): Separated static and runtime OMPL acceptance
  evidence.  The 16-check preflight is now self-contained: the real unmodified
  `preflight_manifest` runs against a reconstructed Task 3-compatible project
  root (committed source + pinned git objects + a legacy `current.json` selector
  pointing at the reproduced canonical URDF), so `ready=true` and the stable
  preflight hash reproduce with no gitignored `outputs/`/`artifacts/`
  dependency.  The real `current.json` is a separate provisioned-host runtime
  readiness diagnostic (stale selection fails; absent selection is reported
  `not_provisioned`, not a static failure).  Top-level `evidence.preflight` now
  carries the load-bearing `stable_manifest_sha256`/`stable_preflight_sha256`
  and nests the raw hashes under `host_snapshot`.  Task 7 action-client scope is
  AST-grounded in pinned Task 7 source (exactly one MoveGroup client on
  `/move_action`); task_range boundary subjects + exact Task 3-7 commit list are
  recorded and recomputed; fixture status publication is recomputed from Task 5
  source; every artifact `path_relative` follows the recorded source policy; a
  temporary copy-install/wheel test proves the symlinked contract + scenarios
  install as real byte-identical files; the pinned v1 canonicalizer is loaded by
  deterministic temporary-package materialization (no `exec` string surgery);
  joint-limit acceptance is semantic + toolchain-aware; and a clean-checkout
  regression seam (`git clone` of the tracked tree) requires every static test to
  pass with no gitignored trees.
- 2026-08-02 (fix round 3): Locked the clean-checkout acceptance coverage.  The
  regression seam now collects the exact 64-node `Task8OMPLOverlayProvenanceTest`
  class in the clone (canonical SHA-256 over the sorted node list, so a deleted
  or added test fails even if the count is preserved) and executes it under a
  machine-readable JUnit XML, requiring total=64, failures=0, errors=0, skipped=4,
  the exact four host-runtime diagnostic skip names, and their reason categories —
  an unrelated static assertion broadened into a skip is rejected.  The
  reconstructed Task 3 resolver (`tools/tinker_sim_deploy`) is now materialized
  from immutable git objects at the recorded simulator implementation identity
  (never copied from the live working tree, so local edits cannot influence
  acceptance), and a pinned-blob mismatch/missing file fails closed.  The
  fixture status field contract is asserted against an independent 12-field
  literal (not re-derived from the source function), and a stale
  `current.json` selector is proven to fail the preflight `artifact_identity`
  check.  The fix-1 baseline is documented as 2 failed + 1 skipped.
- 2026-08-02 (fix round 4): Isolated the pinned-resolver verification.  Because
  the test module pre-imports `tinker_sim_deploy` from the live working tree,
  Python's module cache masked the earlier in-process dirty live-tree probe
  (the proof did not actually execute the pinned materialized resolver).  The
  reconstruction proof now runs in a fresh isolated subprocess (`-I`, no
  inherited `tinker_sim_deploy*` cache) that loads `tinker_sim_deploy.runtime`
  from the materialized temp root, records its resolved `__file__`, runs the
  real Task 3 preflight, and emits one machine-readable JSON with
  ready/16-check/exact-stable-hash evidence; a temp decoy working-tree package
  proves the pinned path wins path precedence (positive) and is load-bearing
  (negative).  No test writes a tracked active-checkout file.  The clean-checkout
  seam's collection/JUnit acceptance checks are factored into one deterministic
  validator used by both the real clone output and realistic mutated fixtures
  (delete, rename+add, unrelated skip, removed skip, wrong reason, duplicate
  testcase, failure/error count, multiple suites), and the JUnit structure is
  tightened to exactly one `<testsuite>` with 64 unique `<testcase>` entries.
- 2026-08-02 (fix round 1): Made the OMPL-overlay acceptance evidence
  reproducible.  The production-overlay scope is split into the exact production
  allow-lists (from `launch_contract_helpers.py`, including
  `re`/`importlib`/`rviz2`) and a simulator-overlay provider set derived from
  the provider manifest; the provenance suite now recomputes the 18-argument
  contract, literal booleans, strict-sim keys, allow-lists, handoff, and
  action-client scope from immutable production git objects.  The model-bundle
  evidence is reproducible on a clean checkout: the canonical simulator full
  URDF source + provenance descriptor are committed under
  `integration/model-bundle-r2/simulator_full_urdf/`, and stable manifest /
  preflight hashes exclude host-absolute paths and `elapsed_ms`.  The
  acceptance contract and three qualification scenarios are installed under
  `share/tinker_sim_bridge/{integration,scenarios}/` with a deterministic
  launch package-share fallback.  `tests/test_provenance.py` fails (never
  skips) when required source evidence is absent, and the documented focused
  pytest invocation uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.  The two
  source-lock files remain Task 9 only.
- 2026-08-02: Added the deterministic OMPL overlay acceptance contract
  (`integration/ompl-overlay-contract.json`) packaging the reviewed Tasks 3-7
  interfaces: exact repository/commit identities (simulator `f34de5f`;
  production runtime hardening `f3e2ce4..df702a5` and Task 2 launch
  `f7fea50..39d96a1` from actual git history), production overlay and
  literal-false `use_cumotion_*` compatibility values, provider-manifest raw +
  canonical identities, model-bundle schema/artifact hashes and full eight-link
  touch set, typed action/service/topic contract, fixture/scenario identities,
  Task 6/7 evidence, and `tkbuild`/`build-humble-overlay` build commands.
  `tests/test_provenance.py` recomputes every derived hash/contract.  The two
  source-lock files remain Task 9 only.
- 2026-07-30: Added deterministic exporter-side canonical URDF derivation with
  full-SHA immutable generation identity, exact portable source-lock identity,
  serialized crash-consistent publication, physical arm-joint validation,
  relocatable Humble launch inputs, shared runtime validation, and bundle
  admission checks. The USD physics bytes remain unchanged.
