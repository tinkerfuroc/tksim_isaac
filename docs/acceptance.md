# Release acceptance checklist

## Environment and provenance

- Fresh online bootstrap succeeds without skip flags.
- Fresh offline bootstrap succeeds from an empty destination.
- A second bootstrap makes no dependency or source changes.
- Two servers report identical package versions and release hashes.
- Isaac Lab `HEAD`, Git tree, and clean status match `release-manifest.json`.
- Isaac `sys.path` contains no `/opt/ros/humble` or Python 3.10 package path.

## GPU and streaming

- Compatibility checker exits successfully.
- Headless PhysX completes 10,000 steps.
- RTX camera and LiDAR each produce a GPU-backed frame/buffer.
- NVENC hardware encode succeeds.
- A fresh temporary environment synchronizes with `--frozen --offline` from
  the populated versioned cache.
- One viewer connects through trusted LAN/VPN and a second viewer is rejected.
- NVIDIA's native Ubuntu Omniverse Streaming Client remains connected for at
  least 60 continuous seconds, then client and simulator terminate cleanly.

## ROS boundary

- `/clock`, TF, `cmd_vel`, joint trajectory, camera, and LiDAR data cross the
  Python 3.12/Python 3.10 DDS boundary.
- External FJT and custom Tinker actions control the hardware facade.
- NVIDIA's standard `simulation_interfaces` services and `/simulate_steps`
  action are present; legacy `/sim/control/*` and `/sim/scenario/*` aliases are
  absent.
- The command gateway is the only `/isaac_joint_commands` publisher.
- Each dynamic TF edge has exactly one owner.
- No non-evaluator node can subscribe to `/sim/truth/*`.

## Simulation

- Gazebo and Isaac pass the same base contract suite.
- Parity odometry is scored against hidden truth.
- Arm tracking, collision, effort/contact, safety stops, and retained objects
  are scored.
- A claimed task success fails whenever a hidden world postcondition fails.
- Person approach, pick/deliver/place, and reception/seat assignment each pass
  deterministic seeded scenarios.

## OMPL overlay acceptance contract (Task 8)

The deterministic acceptance contract at `integration/ompl-overlay-contract.json`
packages the reviewed Tasks 3-7 interfaces.  It does not itself prove live OMPL
or authorize cuMotion; live qualification remains a separate gate.

- The simulator implementation identity is commit
  `f34de5f4cd472e2dbb50d65eb53e089bb1c84891`; production runtime hardening is
  recorded from actual git history as the range
  `f3e2ce4f6e00b23f9b35fef14555ff48d8993058..df702a573f971bb3e2008789adc882c09567de7a`
  and the Task 2 production launch as
  `f7fea50b5e15ba22deb9d2ec401097056519bf97..39d96a176904c0b7966b11333c5517b3b54b6ae3`,
  not a mutable concurrent HEAD.
- The production overlay is `mobile_bringup` /
  `manipulation_planning_task_only.launch.py` with the exact 18-argument
  contract and literal-false `use_cumotion_*` compatibility values.
- ROS policy is Humble, `ROS_DOMAIN_ID=25`, `rmw_fastrtps_cpp`, with the
  `local`/`lan` Fast DDS profiles.
- Every typed action/service/topic is recorded with exact
  type/source/cardinality/stamp/QoS policy, including `/isaac_joint_commands`
  depth 50 and the external future `/tinker_integrated_gate_executor`
  publisher ownership of `/sim/safety/operator`.
- The public `scenario-runner.json` report carries the one-key
  `{"execution_profile": "sim_ompl"}` mapping; the full runtime readiness
  contract is carried separately as `runtime_contract_sha256`.
- The provider manifest, model-bundle schema/artifact hashes, semantic
  kinematics/model contract, full eight-link SRDF touch set, fixture
  `sim_fixture/*` ownership and `target_handoff="pick_and_place/object_mesh"`,
  and the three plan-only scenarios are recorded verbatim.
- Production overlay scope is split: `production_overlay.production_allowlists`
  records the exact production launch import/node/executable/controller
  allow-lists (including `re`/`importlib`/`rviz2`) from
  `launch_contract_helpers.py`, while
  `production_overlay.simulator_overlay_provider_set` derives the simulator
  overlay provider packages/executables from the committed provider manifest.
- The static acceptance evidence is reproducible on a clean tracked-only
  checkout with no gitignored `outputs/`/`artifacts/` dependency:
  - the canonical simulator full URDF's source and provenance descriptor are
    committed (`integration/model-bundle-r2/simulator_full_urdf/`), and the
    v1 canonicalizer is loaded from the pinned git object;
  - the model bundle, synthesized joint limits, and production artifacts are
    reconstructed from committed/pinned inputs, and
    `model_bundle.stable_manifest_sha256` /
    `model_bundle.preflight_report.stable_sha256` hash deterministic
    projections that exclude host-absolute paths and `elapsed_ms`;
  - the 16-check preflight runs the real unmodified `preflight_manifest`
    against a self-contained reconstructed Task 3-compatible project root
    (including a legacy `current.json` selecting the reproduced canonical
    URDF), requiring `ready=true`, all 16 checks including
    `artifact_identity`, and the stable preflight hash.
- The real `artifacts/robot/tinker2/current.json` is a separate
  provisioned-host runtime-readiness diagnostic: when provisioned it must
  select bytes equal to the reproducible derivation (stale selection fails);
  when absent the host is reported `not_provisioned` (not runtime-ready) as a
  typed diagnostic without failing or silently skipping the static suite.
  Top-level `evidence.preflight` carries the load-bearing
  `stable_manifest_sha256` / `stable_preflight_sha256` and nests the raw
  host-scoped hashes under `evidence.preflight.host_snapshot`.
- The acceptance contract and the three qualification scenarios are installed
  under `share/tinker_sim_bridge/integration/` and
  `share/tinker_sim_bridge/scenarios/` (byte-identical to source under both
  the real colcon prefix and a temporary copy-install/wheel build), and the
  integrated launch resolves a scenario through a deterministic package-share
  fallback, rejecting byte disagreement when both sources exist.
- Build commands use `tkbuild` and `scripts/build-humble-overlay` only, never
  raw colcon.
- Both repository-local source-lock files are Task 9 only.
- `tests/test_provenance.py` recomputes every derived hash/contract from
  immutable git objects and committed source and fails on mutations (argument
  order/count, literal compatibility booleans, strict-sim keys, production
  import/node/executable allow-lists, simulator provider set, handoff, Task 7
  action-client scope, task-range boundary/commits, fixture status
  publication, artifact path policy, model-bundle source evidence, top-level
  stable hashes, stale current selection).  A clean-checkout regression seam
  (`git clone` of the tracked tree) collects the exact 64-node
  `Task8OMPLOverlayProvenanceTest` class (canonical node-set SHA-256, so a
  deleted/added test fails even at a preserved count) and executes it under a
  machine-readable JUnit XML, requiring total=64, failures=0, errors=0,
  skipped=4, the exact four host-runtime diagnostic skips, and their reason
  categories — a static assertion broadened into a skip is rejected.  The
  collection/JUnit acceptance checks are factored into one deterministic
  validator that is applied both to the real clone output and to realistic
  mutated fixtures (delete one node, rename+add substitution, an unrelated
  skip, a removed expected skip, a wrong skip reason, a duplicate testcase, a
  failure/error count, and multiple suites), and the JUnit structure is
  tightened to exactly one `<testsuite>` with 64 unique `<testcase>` entries.
  The reconstructed `tools/tinker_sim_deploy` resolver is materialized from
  immutable git objects at the recorded simulator implementation identity
  (never the live working tree); because the test module pre-imports
  `tinker_sim_deploy` from the live checkout, the pinned-resolver proof runs in
  a fresh isolated subprocess (`-I`, no inherited module cache) that records
  the loaded `tinker_sim_deploy.runtime.__file__` from the materialized temp
  root and runs the real Task 3 preflight, with a temp decoy working-tree
  package proving the pinned path wins precedence (positive) and is load-bearing
  (negative).  No test writes a tracked active-checkout file.  The fixture
  status field contract is asserted against an independent 12-field literal.
  The pre-existing uv
  environment provenance failure
  (installed `uv 0.12.0` vs pinned `uv 0.10.8`) is an environment failure, not
  a code failure.  The focused invocation uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
  (ROS plugin discovery may auto-load `launch_pytest`, which can fail on hosts
  without the `lark` module; this is the defensive reproducible invocation).

## Simulator repository-local source lock (Task 9)

This section accompanies `integration/source-locks.json` in the simulator
repository at `/home/tinker/tinker-sim/6.0.1`. Together they form the simulator
repository-local authorization policy for the post-implementation OMPL source
lock.

### What this policy records

- The implementation parent (`implementation_head`) is the exact commit
  `490f907831d9f6f06242e0d151ac014547973d6e`, the simulator HEAD immediately
  before this lock-only commit.
- `mode` is `"clean"`: at lock time the tracked tree was clean, so the captured
  `status_bytes` and `diff_bytes` are both empty
  (`{"encoding": "base64", "data": ""}`, SHA-256 of `b""` =
  `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`) and the
  untracked manifest is exactly `[]` (SHA-256 of the canonical compact list
  bytes `b"[]"` =
  `4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945`).
- The exact capture argv arrays, each run under `LC_ALL=C`:
  `["env", "LC_ALL=C", "git", "status", "--porcelain=v1", "-z",
  "--untracked-files=all"]` and
  `["env", "LC_ALL=C", "git", "diff", "--binary", "--no-ext-diff"]`.
- The policy commit is derived by resolution, never embedded: a Git commit
  cannot embed its own object ID, so the policy records `implementation_head`
  and `policy_commit_resolution = "commit_containing_policy_path"`. The actual
  resolved hash is verified after commit and recorded in the Task 9 simulator
  report ledger, never inside the policy file.

### Evidence normalization contract

Observation and qualification must reproduce these conventions exactly or fail
closed.

- **Git mode normalization.** `untracked_manifest[].mode` uses canonical Git
  index mode semantics: regular file with owner-executable clear = `100644`,
  owner-executable set = `100755`, symlink = `120000`; directories and devices
  are rejected. Regular-file bytes are read without following symlinks; a
  symlink's target text is hashed as bytes. A host mode such as `100664`
  (group-writable) normalizes to `100644`.
- **Untracked manifest digest.** `untracked_manifest_sha256` is SHA-256 over the
  UTF-8 bytes of the entire manifest list serialized as compact sorted-key JSON
  (`json.dumps(manifest, sort_keys=True, separators=(",", ":"))`) with no
  trailing newline. For clean mode the exact list is `[]`.
- **Status/diff bytes.** Observation runs the exact recorded argv and compares
  raw bytes; any Git, configuration, or version drift that changes the bytes
  fails closed.

### Policy commit resolution (schema transition)

`integration/source-locks.json` pre-existed this OMPL authorization: it was
introduced at the 6.0.1 baseline commit `63913b1` with the deployment-workspace
lock fields only. A raw path-history count therefore equals two after this lock
commit and must never be misread as a false count of one. The unique policy
commit is the **schema-transition commit** that introduces the repository-local
OMPL authorization fields (`repository`, `implementation_head`,
`policy_commit_resolution`) and whose parent blob lacks them. Verification
requires the conjunction:

- exactly one commit in checked-out history introduces the OMPL authorization
  schema for `integration/source-locks.json` (blob/schema transition, not raw
  path-touch count);
- that commit's first parent equals `implementation_head`
  (`490f907831d9f6f06242e0d151ac014547973d6e`);
- the resolved commit is an ancestor of the checked-out HEAD (it need not remain
  HEAD after a later docs-only review fix);
- the `HEAD:<policy_path>` blob equals the working-tree policy blob;
- repository/root/path fields match the simulator repository;
- duplicate, later rewrite, cross-repository, missing, ambiguous, or
  self-referential records fail.

### Qualification contract

Later qualification must:

- load exactly one simulator policy (`integration/source-locks.json`) and reject
  missing, duplicate, ambiguous, cross-repository, or self-referential records;
- resolve the unique schema-transition commit and require its first parent to
  equal `implementation_head`;
- compare current `status`, `diff`, and untracked bytes byte-for-byte against
  this policy, reproducing the exact capture argv and normalization above;
- treat this policy as pre-attempt authorization only: qualification may observe
  it but must never create or update it.

### Scope and limits

- The pre-existing deployment-workspace locks (`schema_version`,
  `workspace_policy`, `isaacsim_ros_workspaces`, `tinker_cumotion`,
  `tinker_isaac_ros_common`) are preserved with their scalar values unchanged
  and remain separate from the OMPL repository authorization.
- This lock records state only; it does not modify any unrelated tracked or
  untracked path.
- Live OMPL remains unproven by this lock.
- cuMotion remains unauthorized by this lock.
