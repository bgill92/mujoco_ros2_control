# MuJoCo ROS 2 Control

Simulation-based robot control stack: runs MuJoCo physics behind a ros2_control hardware interface, with ROS 2 plugins extending the simulator.

## Language

### Vacuum gripper (planned plugin)

**Vacuum gripper**:
One plugin instance owning one suction pickup: a pair of bodies (gripper side and part side) joined by a weld equality constraint, plus a latched vacuum state.
_Avoid_: suction cup, vacuum pad (that is the MJCF body), gripper (too generic)

**Vacuum on / vacuum off**:
The latched gripper state set by the activate/release triggers. Vacuum on means the gripper *would* hold; it holds only while contact persists.
_Avoid_: enabled, active (ambiguous with weld active)

**Weld active**:
The weld equality constraint is engaged in the current physics step. True iff vacuum on AND the two gripper bodies are in contact.
_Avoid_: latched, engaged (use "weld active" or "weld engaged" consistently; prefer "weld active" in API names)

**Engage**:
The transition where contact appears while vacuum is on: the weld's relative pose is set to the bodies' current relative pose and the constraint is activated.
_Avoid_: attach, grab

**Release**:
Either trigger (vacuum off) or contact loss, whichever first; the weld is deactivated and the part falls.
_Avoid_: drop, detach

**In contact**:
At least one active contact point exists between any geom of the gripper body and any geom of the part body in the current mjData.
_Avoid_: touching, sealed
