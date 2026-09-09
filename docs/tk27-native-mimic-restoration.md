# Native gripper mimic restoration

The complete tk27 cycle at tk27 `08be31c` and simulator `1a19fae` executes Pick,
lift, transfer, Place, release, and settled placement, but strict carried
retention fails at 9.57417 degrees. The unchanged angular bound is five degrees.
Both pads remain in contact, with approximately 10-14 N reported normal force.
Translation drift accumulates even during the pre-transfer hold.

Increasing velocity iterations from one to eight, or position iterations from
32 to 128, does not resolve stable held-lift qualification. Neither diagnostic
override is promoted. Evidence resides in tk27 outputs
`native-full-batch-cycle-20260910-15`, `-16`, and `-17` respectively.

## Imported constraint metadata

The canonical imported USD explicitly authors this metadata on each follower:

```
apiSchemas = [PhysicsJointStateAPI:angular, PhysxJointAPI, PhysxMimicJointAPI:rotX]
physxMimicJoint:rotX:gearing = -1
physxMimicJoint:rotX:offset = 0
physxMimicJoint:rotX:naturalFrequency = 25
physxMimicJoint:rotX:dampingRatio = 0.005
```

Its reference relationship targets the drive joint. `GetAppliedSchemas()` in
the offline USD interpreter omits unregistered PhysX schemas, whereas
`GetMetadata("apiSchemas")` retains the explicit declaration. Earlier comments
that the importer dropped mimic coupling are therefore incorrect. A soft
constraint also cannot be disproved merely by moving the master while holding
a follower with an independent drive.

PhysX documents `q_A + gearing*q_B + offset = 0`; zero natural frequency selects
a hard constraint. See the
[PhysX articulation API](https://nvidia-omniverse.github.io/PhysX/physx/latest/_api_build/classPxArticulationReducedCoordinate.html)
and the [mimic-joint setup guide](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/simulation_setup/articulations.html).

The validation implementation retains authored references, gearing, and offset,
sets native mimic compliance to zero before the initial PhysX parse, and makes
the five follower actuators passive. The master drive retains its 2.5 N*m
effort ceiling. No object mass, friction, contact geometry, or acceptance bound
is changed. Constraint reaction forces replace the independent follower motors.

The USD regression proves these authored properties and preservation of the
master reference and effort field. Native launch 20 reports all five rigid
mimic constraints with passive followers. Its free-space close/reopen probe
records 138 measured samples with maximum follower/master error 0.00009033 rad.
Both commands succeed, proving actual coupling while follower motors are
passive. Evidence: tk27 `outputs/native-mimic-free-space-20260910-01.json`.

The obsolete software mirror and its two implementation-specific tests are
removed. The resulting native suite reports 203 passed and seven subtests in
the USD/Python 3.12 environment, plus six real ROS transport/reset tests passed
in Humble/Python 3.10: 209 unique cases total. The original failed attempts
remain preserved.

The full-cycle attempt fails before motion because a red RGB silhouette carries
background depth, expanding the observed target into the open-jaw clearance
budget. This input failure does not evaluate loaded mimic retention. The saved
request and exact point attribution are recorded in tk27 outputs
`native-full-batch-cycle-20260910-18`, `native-grasp-open-jaw-attribution-20260910-39.json`,
and `native-grasp-extrema-attribution-20260910-40.json`. Loaded retention remains
a separate pending verification gate.

## Loaded hard-constraint result and critical compliance

Native launch 21 reaches a 123.09 mm lift with the native rigid linkage, but the
arm controller aborts at its original goal tolerance: joint index 4 has a
0.012890 rad error against 0.010000 rad. Loaded mimic error reaches 0.01066 rad
and measured linkage velocity oscillates. Maximum recorded object drift is
8.14 mm and 2.01 degrees. This is a failed attempt, preserved under tk27
`outputs/native-full-batch-cycle-20260910-19`.

The next validation setting uses the native constraint's finite compliance,
natural frequency 120 s^-1 and damping ratio 1.0. At 120 Hz, the timestep times
natural frequency is one. This follows the contact-coupled mimic recommendation
in the [PhysX articulation stability guide](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/articulation_stability.html).
Followers remain passive and the master effort ceiling remains unchanged.
The authored-property test first fails at the previous rigid settings and then
passes with the critically damped constraint. Native loaded qualification is
still required; this source change alone does not establish a stable grasp.

The critical-compliance trial (native launch 22, tk27 cycle output `-20`) does
not lift the object: maximum measured lift is below one micrometer, despite
controller completion. The constraint is too compliant for this loaded linkage.
The 120/1 setting is reverted; its failed evidence is preserved. The following
comparison restores rigid native coupling and changes only TGS position
iterations from 32 to 128, retaining one velocity iteration and every contact,
effort, and execution tolerance.

## Strict complete-cycle pass

Native launch 23, with native source `04aa37d`, tk27 `aa6a80d`, rigid native
mimic coupling, and 128 position / one velocity iteration, passes every gate
of the unchanged full-cycle evaluator. The output is tk27
`outputs/native-full-batch-cycle-20260910-21/qualification.json`.

Pick, the 1.3083-second stable post-pick hold, transport, Place, release, and
settled placement all pass. Continuous carried retention has 962 samples,
zero violations, maximum translation drift 3.3784 mm, and maximum angular
drift 1.5745 degrees. No forbidden contacts are recorded; all 18,865 evidence
records fit within capacity without drops. All 344 generated candidates are
retained before admission. This establishes one positive seed-0 cycle; other
seeds, constrained scenes, cancellation, and lifecycle remain separate gates.

The qualified tk27 native launcher now supplies 128 position iterations by
default while respecting an explicit environment override. This default belongs
to tk27 simulation startup; the generic simulator default remains unchanged.
The earlier 128-iteration failure used software follower motors and is not
equivalent to this rigid native linkage configuration.
