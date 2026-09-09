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
