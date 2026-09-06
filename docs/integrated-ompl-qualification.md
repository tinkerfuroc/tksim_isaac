# Integrated OMPL qualification

Operator reference for the reviewed OMPL overlay's acceptance contract and
the Gate A-F qualification CLI. Moved out of the README to keep that file
to a quick-start; this is the detailed reference.

## OMPL overlay acceptance contract

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

## Integrated OMPL qualification CLI

`validation/integrated_qualification.py` orchestrates the integrated OMPL
qualification Gates A-F.  It is an offline orchestration layer over the
six-gate core suite, the offline static closure, and the live C/D/E scenario
attempts, with an offline Gate-F evidence rebuild/verify.  Task 10's own tests
and this documentation make no live Gate F/OMPL/cuMotion claim.

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
