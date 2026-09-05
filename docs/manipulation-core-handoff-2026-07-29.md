# Tinker manipulation core handoff

Date: 2026-07-29

## Status

The manipulation-core implementation is substantial but the milestone is
**not qualified**. Do not report it complete until all six built-in gates pass
in one fresh suite, independent verification passes, teardown is clean, and
both contact sheets have been reviewed.

The shortest path is to integrate and test the mirror-only safety/evidence
patch, rerun the focused safety gate, then run the remaining gates and the
full suite. MoveIt, cuMotion, vision, decision, and VLA remain out of scope.

## Working trees

- Authoritative runtime tree: `/home/tinker/tinker-sim/6.0.1`
- Writable integration mirror:
  `/home/tinker/tk25_ws/tinker-sim-6.0.1-work`
- Main progress report:
  `/home/tinker/tinker-sim/6.0.1/docs/progress-report-2026-07-28.md`

This project is not a Git worktree. Compare mirror files directly against the
authoritative tree before syncing. Do not overwrite unrelated authoritative
changes.

## Delivered in the authoritative tree

- Built-in executors and independent verifiers for:
  `free-space-fjt`, `safety-stop`, `free-gripper`,
  `obstructed-gripper`, `arm-collision`, and `retention`.
- Raw physics truth schema/evaluator, exact frame correlation and bounded
  drain.
- Standard `FollowJointTrajectory` and gripper action paths through the
  command gateway.
- Launch readiness handshake, deterministic qualification fixtures, pedestal,
  dynamic cube placement, and fixed qualification cameras.
- Fail-closed command baseline resynchronization after safety clear.
- Explicit gravity-compensated measured-state PD safety hold.
- Strict rosbag startup/finalization, graph ownership checks, process/GPU
  ownership accounting, and bounded cleanup.
- Journal-bound visual checkpoints and separate user/agent contact sheets.

Last established non-live baseline before the current mirror patch:

- Generic suite: `309 passed, 3 skipped, 1 warning, 6 subtests passed`.
- Sourced Humble manipulation partition: `73 passed, 1 skipped`.
- Humble overlay build completed for `tinker_sim_interfaces` and
  `tinker_sim_bridge`.

## Latest live attempt

Preserve this directory:

`/tmp/tinker-manipulation-core-20260729-safety/20260729T083938.516076Z-2198516-34992310b5`

Command used:

```bash
cd /home/tinker/tinker-sim/6.0.1
./.venv/bin/python validation/manipulation_qualification.py \
  --root /home/tinker/tinker-sim/6.0.1 \
  --attempt-root /tmp/tinker-manipulation-core-20260729-safety \
  --gate safety-stop \
  --base-domain-id 186 \
  --readiness-timeout 60
```

Useful outcomes:

- GPU was available at launch.
- The built-in safety executor completed with no executor exception.
- Safety asserted at simulation time `17.65`.
- Arm velocity first became compliant at `17.666666667`.
- Safety cleared at `18.183333333`; the post-clear checkpoint was
  `18.216666667`.
- The trajectory action ended unsuccessfully as required: status `6`,
  result success `false`.
- Post-gate health was ready.
- Raw/evaluator drain was exact: `3743` frames each, no mismatches or evaluator
  errors.
- Isaac, Humble, and rosbag each exited `0` after planned SIGINT.
- Cleanup found no owned PID/GPU survivor and no unexplained GPU memory.
  Baseline and final GPU memory were both `474 MiB`.

Why the attempt failed:

- The independent verifier did not have a full 0.5 second stopped evidence
  window or 1 second post-clear evidence window.
- The safety gate had no objects or contacts, so finalized rosbag counts for
  `/sim/truth/object_state` and `/sim/truth/contacts` were legitimately zero,
  but the authoritative validator required every approved topic to be
  positive.
- The physical safety hold stopped within two frames, but joint 1 later
  saturated at the nominal `50 Nm` limit and drifted during the required hold.
  Measured maximum velocity over the hold was approximately `0.1594 rad/s`;
  maximum drift was approximately `0.10379 rad`.

The attempt itself was fully cleaned up. Its `resource-cleanup.json` and
`termination.json` are the source of truth for that statement.

## Mirror-only patch

The mirror contains changes that are not yet in the authoritative tree:

- `simulation/tinker_sim_isaac/backend.py`
  uses the configured finite `100 Nm` emergency ceiling for every arm joint
  instead of clipping the safety hold to nominal actuator limits.
- `tests/test_manipulation_runtime.py`
  covers positive/negative clipping, all seven arm joints, and restoration of
  each nominal limit after clear.
- `validation/manipulation_gate_executor.py`
  retains 0.65 seconds of simulated stopped evidence after velocity compliance
  and 1.1 seconds after safety clear.
- `tests/test_manipulation_gate_executor.py`
  covers both bounded simulated-time windows.
- `validation/manipulation_qualification.py`,
  `simulation/qualification/manipulation-core.json`, and
  `tests/test_qualification_manifest.py`
  add gate-aware rosbag minimum counts. Object/contact counts may be zero only
  for gates where those streams are physically inapplicable; required streams
  remain fail-closed.
- The manipulation qualification checksum layer has been removed at user
  request. Runtime safety, process/GPU cleanup, evidence parsing, and
  fail-closed physical verification remain in place.

One Luna worker reported `310 passed, 3 skipped, 1 warning, 6 subtests` after
the executor-window change. The combined backend, gate-aware rosbag, and
checksum-removal state subsequently passed `190/190` focused non-live unit
tests. A full suite was started but intentionally stopped at the user's
request during handoff; it is not a completed verification result.

Review the actual delta:

```bash
cd /home/tinker/tk25_ws/tinker-sim-6.0.1-work
diff -qr /home/tinker/tinker-sim/6.0.1 . \
  --exclude .venv --exclude __pycache__ --exclude .pytest_cache
```

## Checksum removal

At the user's request, remove only the manipulation qualification SHA-256
generation/verification layer and its generated checksum/index artifacts.
Keep general dependency, deployment, asset, and workspace-lock integrity
mechanisms outside manipulation qualification.

Do not weaken:

- physical gate thresholds;
- raw/evaluator correlation;
- rosbag topic/QoS/count validation;
- source file presence checks;
- action-result and world-state verification;
- command and raw-truth ownership checks;
- process/GPU ownership accounting or cleanup.

Legacy `manifest.sha256`, `evidence.sha256`, `suite-evidence.sha256`, and
hash-only `evidence-index.json` files under `/tmp/tinker-*` were deleted.
The failed attempt directories and all non-hash evidence were retained.

No live qualification was run as part of this handoff.

## Next steps

1. Review the entire mirror/authoritative delta. Keep the changes above
   narrowly scoped and remove any partial worker edits that do not match this
   handoff.
2. Run the generic test suite in the mirror:

```bash
cd /home/tinker/tk25_ws/tinker-sim-6.0.1-work
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/python -m pytest
```

3. Run the sourced Humble manipulation partition and rebuild the overlay using
   the existing project wrappers. Do not treat the expected
   `simulation_interfaces` skip under system Humble as a blocker.
4. Sync only reviewed changed files into
   `/home/tinker/tinker-sim/6.0.1`, then rebuild the Humble overlay there.
5. Confirm the GPU has no unrelated compute allocation. Run the focused
   `safety-stop` command above with a new attempt root/domain ID.
6. Inspect the independent gate verdict, gate window, raw truth, rosbag final,
   post-gate health, termination, and resource cleanup. Do not delete a failed
   attempt.
7. If safety passes, run focused gates in this order:
   `free-space-fjt`, `free-gripper`, `obstructed-gripper`,
   `arm-collision`, `retention`.
8. Fix minor nonblocking issues directly. Stop only for a core workflow or
   safety failure, a memory/process leak, or invalid evidence.
9. Run `--gate all` with a fresh suite root. Qualification requires every gate
   and the overall suite to be `verified-pass`.
10. Review both generated contact sheets. Regenerate them from the accepted
    attempt/suite if any checkpoint is missing, stale, unreadable, or
    mismatched.

## Qualification acceptance

The milestone is complete only when one preserved fresh suite demonstrates:

- all six gates are `verified-pass`;
- every required action and physical predicate is independently recomputed;
- safety velocity, hold drift, unsuccessful action termination, clear
  behavior, and no automatic resume all pass;
- gate-aware rosbag topic counts and QoS pass;
- raw/evaluator drain is exact with no errors;
- post-gate Isaac, Humble, and rosbag health pass;
- planned termination is clean with no forced descendant cleanup;
- no attempt-owned process, GPU process, or unexplained GPU memory survives;
- user and agent contact sheets contain all required, legible checkpoints.

## Visual review

Current diagnostic sheets are useful for comparison but are not qualification
proof:

- [User contact sheet](../reports/manipulation-core-20260728/contact-sheet-user.png)
- [Agent contact sheet](../reports/manipulation-core-20260728/contact-sheet-agent.png)
- [Safety diagnostic](../reports/manipulation-core-20260728/safety-stop-diagnostic.png)

The two suite sheets deliberately show `EVIDENCE-INVALID`; the safety
diagnostic predates the current mirror patch. Replace them only with artifacts
from a fresh verified run, and keep the failed diagnostics for traceability.

## Avoid rework

The launch ownership model, fixture placement, recorder startup contract,
truth-drain ownership, contact-sheet pipeline, and teardown accounting have
already been exercised live. Revisit them only if new evidence identifies a
breaking failure or leak. The immediate blocker is focused safety
qualification, followed by the remaining gates and the full suite.
