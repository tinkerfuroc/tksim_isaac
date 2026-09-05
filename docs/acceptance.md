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
- `authorization.commit` is `null` by design: no earlier authorization commit
  exists. The uniquely resolved schema-transition commit
  (`ab8cf7e...`) is itself the pre-attempt authorization, so `null` does not
  mean unauthenticated — authorization is derived from the resolved transition,
  not from a prior separate authorization commit. The phase is
  `task-9b-simulator-repository-lock-only` and the report ledger path is
  `.superpowers/sdd/2026-07-29-simulator-ompl-moveit-overlay/task-9-simulator-report.md`.
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
- **Staged changes fail closed.** The status capture runs
  `git status --porcelain=v1 -z --untracked-files=all`, whose raw porcelain
  payload covers the staged (index) column in addition to unstaged and
  untracked entries, whereas `git diff --binary --no-ext-diff` is unstaged-only.
  Clean mode requires the status bytes, the diff bytes, and the untracked
  manifest all to be exact empty, so any staged path — even one whose unstaged
  diff alone would be empty — fails the policy comparison.

### Policy commit resolution (schema transition)

Despite the common enum `policy_commit_resolution="commit_containing_policy_path"`,
resolution is machine-checkable and deterministic. The simulator resolution
algorithm is:

1. `required_fields` = the exact set `{"repository", "implementation_head",
   "policy_commit_resolution"}`.
2. Enumerate every commit `c` that touches `integration/source-locks.json` in
   checked-out history, i.e. the set `git log --format=%H -- <policy_path>`.
3. A commit `c` is the schema-transition commit iff its policy blob
   `git show c:<policy_path>` contains all three `required_fields` AND its
   first-parent policy blob `git show c^:<policy_path>` lacks at least one.
   A root commit (no parent) can never qualify because it has no first-parent
   blob.
4. The resolved transition must be unique: exactly one such commit exists in
   checked-out history, namely `ab8cf7e9645b1e019aba81e2c7923177ba13d1ac`.
5. The resolved transition must satisfy: first parent == `implementation_head`
   (`490f907831d9f6f06242e0d151ac014547973d6e`), and it is an ancestor of the
   checked-out HEAD (it need not remain HEAD after a later docs-only review
   fix).

`integration/source-locks.json` pre-existed this OMPL authorization: it was
introduced at the 6.0.1 baseline commit `63913b1` (a root commit) with the
deployment-workspace lock fields only. The raw path history after this lock
commit is therefore exactly 2 and the schema-transition count is exactly 1 — the
raw count must never be misread as a false count of one. The raw path history
count is 2 at lock time and may only remain unchanged: any later policy-path
touch/rewrite is rejection.

The transition's blob identity is load-bearing. Verification additionally
requires the conjunction:

- exactly one commit in checked-out history introduces the OMPL authorization
  schema for `integration/source-locks.json` (blob/schema transition, not raw
  path-touch count);
- that commit's first parent equals `implementation_head`
  (`490f907831d9f6f06242e0d151ac014547973d6e`);
- the resolved commit is an ancestor of the checked-out HEAD (it need not remain
  HEAD after a later docs-only review fix);
- the three-way blob identity holds: bytes of
  `git show <transition_commit>:<policy_path>` equal bytes of
  `git show HEAD:<policy_path>` equal the working-tree policy bytes. Any later
  committed policy rewrite therefore fails closed even if it retains the same
  fields; a docs-only commit that leaves the policy bytes untouched remains
  allowed;
- repository/root/path fields match the simulator repository;
- duplicate, later rewrite, cross-repository, missing, ambiguous, or
  self-referential records fail.

### Qualification contract

Later qualification must:

- load exactly one simulator policy (`integration/source-locks.json`) and reject
  missing, duplicate, ambiguous, cross-repository, or self-referential records;
- resolve the unique schema-transition commit and require its first parent to
  equal `implementation_head`;
- assert the three-way blob identity
  (`<transition>:<policy_path>` == `HEAD:<policy_path>` == working-tree bytes)
  and the exact raw path-history count of 2 unchanged from lock time, rejecting
  any later policy-path touch or rewrite;
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

## Integrated OMPL qualification CLI (Task 10)

`validation/integrated_qualification.py` is the offline integrated OMPL
qualification orchestrator.  It runs the six-gate Stage-A core suite, the
offline Stage-B static closure, the live Stage-C/D/E scenario attempts, and the
offline Stage-F evidence rebuild/verify.  Task 10's own tests and this
documentation make no live Gate F/OMPL/cuMotion claim.

Use one consistent suite path variable:

```bash
SUITE_DIR=outputs/integrated/integrated-ompl-seed-7
```

`SUITE_DIR` is passed as the runner's `--attempt-root`.  It is the exact
integrated suite directory; nothing is silently appended below it.  A repeated
qualification run must choose a fresh `--attempt-root`; never merge a new run
into an old suite.  The six-gate Stage-A core suite runs in the sibling root
`<SUITE_DIR>-core/` (here `outputs/integrated/integrated-ompl-seed-7-core/`),
outside the integrated Gate-F index.

Gate A (six-gate core suite):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage A
```

Gate B (offline static closure; `--offline` is an explicit compatibility flag
for the already-offline B implementation and must not make any live stage
offline or bypass checks):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage B --offline
```

Gate C (three OMPL plan-only scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage C
```

Gate D (six execute scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage D
```

Gate E (eight pick-place scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage E
```

Gate F (standalone offline rebuild/verify):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage F
```

All stages (`--stage all`):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage all
```

The standalone Gates A through F above and `--stage all` are alternatives.
`--stage all` must use a fresh suite path and must not be run against a suite
that already has write-once A-E stage records.

Independent verifier replay against the selected immutable C/D/E attempt
directory.  Every C/D/E scenario runs in a newly created immutable attempt
directory named `STAGE-<scenario>-<invocation>-<counter>`.  The selection
below finds the exact single immutable matching attempt under `$SUITE_DIR`,
fails unless exactly one match is found, and binds it to `ATTEMPT_DIR`:

```bash
ATTEMPT_DIR="$(find "$SUITE_DIR" -maxdepth 1 -type d \
  -name 'C-qualification-moveit-plan-joint-*' | sort)"
test "$(printf '%s\n' "$ATTEMPT_DIR" | sed '/^$/d' | wc -l)" -eq 1
./.venv/bin/python validation/integrated_gate_verifier.py \
  --scenario qualification-moveit-plan-joint \
  --attempt-dir "$ATTEMPT_DIR" \
  --config simulation/qualification/integrated-ompl.json
```

Integrated contact-sheet regeneration against `$SUITE_DIR`:

```bash
./.venv/bin/python validation/integrated_contact_sheets.py --suite-dir "$SUITE_DIR"
```

Standalone evidence-index rebuild and Gate-F validation (writes
`evidence-index.json` and `qualification-summary.json`):

```bash
./.venv/bin/python validation/integrated_evidence_index.py \
  --suite-dir "$SUITE_DIR" --summary "$SUITE_DIR/qualification-summary.json" \
  --validate
```

Failed-attempt retention/rerun rule: never delete or reuse a failed, stale, or
old attempt or suite.  Every C/D/E scenario runs in a freshly created immutable
attempt directory; repeated allocation yields distinct preserved paths.  To
rerun, choose a fresh suite path (a fresh `--attempt-root`); never merge a new
run into an old suite.

Bounded build command (never raw colcon).  The wrapper ignores CLI args and
internally executes colcon with `--parallel-workers 2`:

```bash
MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay
```

Truthful scope:

- The runtime config has three source-lock roles
  (`simulator_overlay` / `production` / `qualification_tooling`).  The
  qualification-tooling source-lock role is created only after Task 10 is
  review-clean, in a separate lock-only commit, and only before live attempts.
- No live Gate F/OMPL/cuMotion claim comes from Task 10's offline tests.
- `_image_stats` thresholds still require live RTX calibration.
- cuMotion remains prohibited until Task 37's live OMPL qualification passes.

## Changelog

- 2026-08-05 (integrated qualification Task 10 — "document integrated
  qualification CLI and docs acceptance"): Documented the offline integrated
  OMPL qualification CLI with one consistent `SUITE_DIR` variable and exact
  standalone command blocks for Gates A-F (Gate B with the explicit `--offline`
  compatibility flag), `--stage all`, the deterministic single-match verifier
  replay (`ATTEMPT_DIR` selection that fails unless exactly one immutable
  matching attempt exists), the integrated contact-sheet regeneration, and the
  bounded `MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay` build command
  (the wrapper ignores CLI args and internally executes colcon with
  `--parallel-workers 2`).  Standalone Gates A-F and `--stage all` are
  documented as alternatives; `--stage all` must use a fresh suite path and
  must not be run against a suite that already has write-once A-E stage
  records.  The evidence-index command writes `qualification-summary.json` via
  `--summary`.  The documentation states the fresh-suite retention rule (never
  delete/reuse; choose a fresh `--attempt-root`), the three source-lock roles
  with the qualification-tooling role created only after review-clean Task 10,
  and the live-only caveats (no live Gate F/OMPL/cuMotion claim from Task 10
  offline tests, `_image_stats` still requires live RTX calibration, and
  cuMotion remains prohibited until Task 37 live OMPL passes).  New
  `tests/test_integrated_acceptance_docs.py` asserts the exact command
  blocks/paths, fresh-suite retention wording, bounded build command,
  three-lock sequence, live-only caveats, and cuMotion prohibition verbatim in
  `docs/acceptance.md`, `integration/MANIPULATION.md`, and `README.md`.  No
  build, no live Isaac/ROS/GPU/cuMotion, and no source-lock file changed; the
  future qualification-tooling source-lock role remains absent until Task 36.
