# VacuumGripperPlugin — Implementation Spec

**Status**: Handoff spec. All design decisions are settled (wayfinder map `mujoco_ros2_control-tpr`,
research tickets `tpr.4`/`tpr.5` closed). An implementation session should be able to build the
plugin + msgs + demo from this document **without further design decisions**. If something here is
ambiguous, that is a spec bug — flag it, don't silently pick.

**Terminology**: see [`CONTEXT.md`](../../../CONTEXT.md) (vacuum gripper, vacuum on/off, weld active,
engage, release, in contact). Use those terms in code, logs, and docs.

## 1. Overview

One plugin instance owns one suction pickup: a pair of MJCF bodies (gripper side + part side)
joined by a **weld equality constraint** (`mjEQ_WELD`) that is declared in MJCF, inactive, and
activated at runtime by the plugin. The gripper latches a *vacuum* state via a ROS 2 trigger,
which is rejected unless the two bodies are in contact. The weld engages on the next physics
step and then **stays active until explicit release or ResetWorld** — there is no auto-release:
the weld constraint keeps the part pinned (a part dragged away while latched is pulled back to
the held pose by the constraint itself). ResetWorld clears everything.

One instance = one (gripper body, part body, weld eq) triple. Multiple grippers = multiple
instances (see demo, §6).

## 2. Files to create / modify

| File | Action |
|---|---|
| `mujoco_ros2_control_msgs/srv/WeldState.srv` | new |
| `mujoco_ros2_control_msgs/CMakeLists.txt` | add `srv/WeldState.srv` to `srv_files` |
| `mujoco_ros2_control_plugins/src/vacuum_gripper_plugin.hpp` | new |
| `mujoco_ros2_control_plugins/src/vacuum_gripper_plugin.cpp` | new |
| `mujoco_ros2_control_plugins/CMakeLists.txt` | add `.cpp` to the `mujoco_ros2_control_plugins_impl` shared-library sources |
| `mujoco_ros2_control_plugins/mujoco_ros2_control_plugins.xml` | register the class |
| `mujoco_ros2_control_plugins/doc/plugins.rst` | new plugin section (interface, params, MJCF convention) |
| `mujoco_ros2_control_plugins/README.md` | table row |
| `mujoco_ros2_control_demos/demo_resources/vacuum_gripper/vacuum_robot.urdf` | new (xacro) |
| `mujoco_ros2_control_demos/demo_resources/vacuum_gripper/vacuum_robot.xml` | new (robot MJCF) |
| `mujoco_ros2_control_demos/demo_resources/vacuum_gripper/vacuum_scene.xml` | new (scene MJCF incl. weld eqs) |
| `mujoco_ros2_control_demos/config/controllers_vacuum_gripper.yaml` | new |
| `mujoco_ros2_control_demos/config/mujoco_ros2_control_plugins_vacuum_gripper.yaml` | new |
| `mujoco_ros2_control_demos/launch/06_vacuum_gripper_plugin.launch.py` | new |
| `mujoco_ros2_control_demos/package.xml` | ensure `mujoco_ros2_control_msgs` dep (for launch/verification if it calls services from Python; already present for other demos — verify) |

No changes to `mujoco_ros2_control` core are needed. `std_srvs` is already a dependency of the
plugins package. No new dependencies anywhere.

## 3. New service type

`mujoco_ros2_control_msgs/srv/WeldState.srv` — query service, empty request:

```
---
bool weld_active
bool vacuum_enabled
string message
```

- `weld_active`: the plugin's own weld eq is active in the latest physics step.
- `vacuum_enabled`: the latched vacuum state.
- `message`: human-readable one-liner (e.g. `weld active`, `vacuum on, not in contact`, `inactive`).

## 4. Plugin: `VacuumGripperPlugin`

Class `mujoco_ros2_control_plugins::VacuumGripperPlugin`, extends
`MuJoCoROS2ControlPluginBase` (`mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp`).
Override `init()`, `pre_step()`, `cleanup()`. Do **not** override `update()` (nothing needs the
control-thread hook).

Registration in `mujoco_ros2_control_plugins.xml`:

```xml
<class name="mujoco_ros2_control_plugins/VacuumGripperPlugin"
       type="mujoco_ros2_control_plugins::VacuumGripperPlugin"
       base_class_type="mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase">
  <description>...</description>
</class>
```

### 4.1 Parameters

Declared/read on the instance node in `init()`, under `mujoco_plugins.<instance>.`:

| Param | Type | Default | Meaning |
|---|---|---|---|
| `type` | string | — | `mujoco_ros2_control_plugins/VacuumGripperPlugin` (read by the loader, not the plugin) |
| `gripper_body` | string | **required, no default** | MJCF body name, gripper side (e.g. the suction pad) |
| `part_body` | string | **required, no default** | MJCF body name, part side (must be a free-joint body in practice) |
| `eq_name` | string | `vacuum_weld` | MJCF `<weld>` equality element name this instance owns |

Declare with `declare_parameter` + default only for `eq_name`; `gripper_body`/`part_body` are
mandatory (use `has_parameter`/declare without default and fail `init()` if absent — mirror how
other plugins validate, log the exact error).

### 4.2 `init()` — id resolution and validation

Resolve once, store as ints (model is immutable for the simulation lifetime):

1. `gripper_body_id_ = mj_name2id(model, mjOBJ_BODY, gripper_body.c_str())`
2. `part_body_id_ = mj_name2id(model, mjOBJ_BODY, part_body.c_str())`
3. `eq_id_ = mj_name2id(model, mjOBJ_EQUALITY, eq_name.c_str())`

Fail (`RCLCPP_ERROR` with all three names, `return false`) if any is `MJ_ID_UNKNOWN`, or:

- `model->eq_type[eq_id_] != mjEQ_WELD`
- `model->eq_objtype[eq_id_]` is not body-typed on both objects (`mjOBJ_BODY` for
  `eq_obj1id`/`eq_obj2id`)
- `{model->eq_obj1id[eq_id_], model->eq_obj2id[eq_id_]} != {gripper_body_id_, part_body_id_}`
  (order-insensitive set equality)
- the two body ids are equal

Do **not** store the `data` pointer passed to `init()` for later writes — `pre_step()` receives
the live pointer each call. (Init-time `init(..., const mjModel* model, mjData* data)`: keep
`model_` (const) for lookups; that's all.)

Then create the three services on the instance node (they surface as
`<parent_ns>/<instance>/<service>`, e.g. `/simulator/vacuum_part1/activate`):

| Service | Type |
|---|---|
| `activate` | `std_srvs/srv/Trigger` |
| `release` | `std_srvs/srv/Trigger` |
| `weld_state` | `mujoco_ros2_control_msgs/srv/WeldState` |

### 4.3 Plugin state

```cpp
std::atomic_bool vacuum_enabled_{ false };  // latched vacuum (set by activate, cleared by release/reset)
std::atomic_bool in_contact_{ false };      // refreshed every pre_step from live contacts
std::atomic_bool weld_active_{ false };     // mirrors data->eq_active[eq_id_]
std::atomic_bool pending_activate_{ false };// set by activate callback, consumed in pre_step
// ids: gripper_body_id_, part_body_id_, eq_id_ (int, resolved in init)
double prev_step_time_{ -1.0 };             // reset detection
```

Service callbacks run on ROS executor threads; `pre_step()` runs on the physics thread. **All
mjModel/mjData mutation happens exclusively in `pre_step()`**; callbacks touch only the atomics.
No mutexes, no blocking, no locks in `pre_step()` (the physics loop must not stall).

### 4.4 Service contracts

**`~/activate`** (std_srvs/Trigger, empty request):

- If `in_contact_` is false → `success=false`,
  `message="activate ignored: bodies '<gripper_body>' and '<part_body>' are not in contact"`.
  **No state change** (no pending/latching state exists by design).
- Else → `pending_activate_ = true`; `success=true`,
  `message="vacuum latched; weld engages on the next physics step"`.

Note: `in_contact_` is at most one physics step stale (it's refreshed in `pre_step`). The weld can
never activate without *live* contact, because actual activation only ever happens in `pre_step`
against the live contact list. In the (rare) race where contact is lost in the one step between
callback and drain, the vacuum latches and the weld simply waits for re-contact.

**`~/release`** (std_srvs/Trigger):

- Always `success=true`. Sets `vacuum_enabled_ = false` immediately (clears any pending activate
  too: `pending_activate_ = false`).
- `message`: `"vacuum released"` if it was latched, else `"no vacuum latched"`.
- `pre_step` deactivates the eq on the next step.

**`~/weld_state`** (WeldState):

- `weld_active = weld_active_.load()`, `vacuum_enabled = vacuum_enabled_.load()`.
- `message`: `"weld active"` / `"vacuum on, not in contact"` / `"inactive"`.

### 4.5 `pre_step()` algorithm (physics thread, every step)

```cpp
void VacuumGripperPlugin::pre_step(mjData* data)
{
  const mjModel* model = data->model;

  // 1) World-reset detection: simulation time going backwards means mj_resetData ran
  //    (ResetWorld service). eq_active is restored to eq_active0 (=0, MJCF active="false")
  //    by the reset itself; we must also clear the latched vacuum.
  if (prev_step_time_ >= 0.0 && data->time < prev_step_time_)
  {
    vacuum_enabled_ = false;
    pending_activate_ = false;
  }
  prev_step_time_ = data->time;

  // 2) Contact scan on live data. In contact iff there is a contact entry with dist <= 0
  //    (actual touch/penetration, NOT the proximity margin) between any geom of the gripper
  //    body and any geom of the part body.
  bool in_contact = false;
  for (int i = 0; i < data->ncon && !in_contact; ++i)
  {
    if (data->contact[i].dist > 0.0)
    {
      continue;
    }
    const int a = model->geom_bodyid[data->contact[i].geom[0]];
    const int b = model->geom_bodyid[data->contact[i].geom[1]];
    in_contact = (a == gripper_body_id_ && b == part_body_id_) ||
                 (a == part_body_id_ && b == gripper_body_id_);
  }
  in_contact_ = in_contact;

  // 3) Drain activate requests queued by the service callback.
  if (pending_activate_.exchange(false))
  {
    vacuum_enabled_ = true;
  }

  // 4) Weld bookkeeping. The weld engages when the vacuum is latched and the bodies are
  //    in contact, and then stays active until ~/release or a world reset (no auto-release).
  if (!vacuum_enabled_)
  {
    if (data->eq_active[eq_id_])
    {
      data->eq_active[eq_id_] = 0;   // released or reset
    }
  }
  else if (in_contact && !data->eq_active[eq_id_])
  {
    engage(data, model);             // no-snap eq_data write, then activate (see §4.6)
  }
  weld_active_ = data->eq_active[eq_id_];
}
```

> **Implementation note (deviation).** The sketch above used contact loss to break the hold.
> That was observed to be unreliable: while the weld is holding, the part rides rigidly at
> the engage-time center distance, but the surface contact pair can drop out of MuJoCo's
> contact margin mid-ride (and reappear a few steps later), so contact-loss release drops a
> *held* part during the lift. The implementation therefore has **no auto-release**: once
> engaged, the weld stays active until `release` or a world reset, and the weld constraint
> itself keeps the part pinned (a part dragged away while latched is pulled back to the held
> pose).

Resulting state machine:

| From | Event | To |
|---|---|---|
| any | `activate`, in contact | vacuum on; weld engages same/next step |
| any | `activate`, NOT in contact | rejected (`success=false`), nothing changes |
| any | `release` | vacuum off; weld off next step |
| any | ResetWorld | vacuum off, weld off (core restores `eq_active` to `eq_active0`; plugin cleared via `world_reset()` hook) |

### 4.6 Engage: no-snap `eq_data` recipe

Weld `eq_data` layout (stride `mjNEQDATA` == 11, verified against MuJoCo 3.8.1 engine sources —
research ticket `tpr.4`):

- `[0..2]` anchor in **body2** (`eq_obj2id`) local frame (the author's MJCF `anchor`, default zero = body2 origin)
- `[3..5]` anchor in **body1** (`eq_obj1id`) local frame
- `[6..9]` relative quaternion = `conj(xquat1) * xquat2` (w,x,y,z)
- `[10]` torquescale

The constraint is satisfied iff `xmat1*eqd[3..5] + xpos1 == xmat2*eqd[0..2] + xpos2` and the
quaternions match. To engage **without snapping the part**, recompute `[3..5]` and `[6..9]` so the
constraint holds at the *current* configuration:

```cpp
void VacuumGripperPlugin::engage(mjData* data, const mjModel* model)
{
  // Do NOT assume which of gripper/part is obj1 vs obj2 — use the model's mapping.
  const int b1 = model->eq_obj1id[eq_id_];
  const int b2 = model->eq_obj2id[eq_id_];
  mjtNum* eqd = data->eq_data + eq_id_ * mjNEQDATA;

  // Keep the author's anchor (b2-local, eqd[0..2]; zero = b2 origin).
  const mjtNum* x2 = data->xpos + b2 * 3;
  const mjtNum* m2 = data->xmat + b2 * 9;  // row-major 3x3 (b2 -> world)
  const mjtNum aw[3] = {x2[0] + m2[0] * eqd[0] + m2[1] * eqd[1] + m2[2] * eqd[2],
                        x2[1] + m2[3] * eqd[0] + m2[4] * eqd[1] + m2[5] * eqd[2],
                        x2[2] + m2[6] * eqd[0] + m2[7] * eqd[1] + m2[8] * eqd[2]};

  // Same anchor point expressed in b1-local: eqd[3..5] = m1^T * (aw - x1).
  const mjtNum* x1 = data->xpos + b1 * 3;
  const mjtNum* m1 = data->xmat + b1 * 9;
  const mjtNum d[3] = {aw[0] - x1[0], aw[1] - x1[1], aw[2] - x1[2]};
  eqd[3] = m1[0] * d[0] + m1[3] * d[1] + m1[6] * d[2];
  eqd[4] = m1[1] * d[0] + m1[4] * d[1] + m1[7] * d[2];
  eqd[5] = m1[2] * d[0] + m1[5] * d[1] + m1[8] * d[2];

  // eqd[6..9] = conj(q1) * q2,  q_i = data->xquat + b_i * 4  (w,x,y,z).
  // Use mju_mulQuat(mju_quatConjugate(q1), q2) or the explicit form:
  const mjtNum* q1 = data->xquat + b1 * 4;
  const mjtNum* q2 = data->xquat + b2 * 4;
  eqd[6] =  q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  eqd[7] =  q1[0] * q2[1] - q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2];
  eqd[8] =  q1[0] * q2[2] + q1[1] * q2[3] - q1[2] * q2[0] - q1[3] * q2[1];
  eqd[9] =  q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] - q1[3] * q2[0];
  // eqd[10] (torquescale): leave the author's MJCF value untouched.

  data->eq_active[eq_id_] = 1;
}
```

Writes to `eq_data`/`eq_active` in `pre_step` take effect on the very next `mj_step` (the engine
gates on `eq_active` per step).

`cleanup()`: nothing to tear down (services die with the node).

## 5. MJCF authoring convention

The weld eq is **declared in MJCF, inactive**; the plugin only ever toggles activation and
rewrites the relative pose at engage. Runtime construction of eq elements is out of scope.

```xml
<equality>
  <weld name="vacuum_weld_part1" body1="vacuum_pad" body2="part1" active="false"/>
</equality>
```

Rules:

- **Naming**: the eq is identified *by name* (`eq_name` param, default `vacuum_weld`). Give each
  eq a distinct name when multiple exist.
- **Convention**: `body1` = gripper-side body, `body2` = part-side body. The optional MJCF
  `anchor` attribute is in the **part (body2) frame**; omit it for the part body origin (the
  part's relative pose is then frozen around its origin — fine for rigid parts).
- **`active="false"` is mandatory** so `eq_active0 == 0`: world resets restore the inactive state,
  and the plugin is the only thing that ever activates it.
- **`torquescale`**: optional author knob (default 1); the plugin never touches it.
- **Body survival**: the gripper-side body (e.g. `vacuum_pad`) is usually attached by a *fixed*
  joint, so the MuJoCo compiler would fuse it into its parent and the name would disappear.
  Scenes using this plugin must set `<compiler fusestatic="false"/>` in the **root** scene file.
  The plugin's `init()` validation (§4.2) catches any name that didn't survive and fails loudly.
- **Contact geoms**: the pad and the part need colliding geoms on their touching faces (default
  `contype`/`conaffinity` are fine — just don't exclude the pair). The contact check uses *all*
  geoms of both bodies, so extra geoms are harmless.
- The part body should be a `<freejoint>` body so it falls when released.

## 6. Demo

New folder `mujoco_ros2_control_demos/demo_resources/vacuum_gripper/`. Follow existing demo
conventions (see `01_basic_robot.launch.py`, `demo_resources/robot/test_robot.urdf`,
`demo_resources/scenes/scene.xml`).

### 6.1 `vacuum_scene.xml` (root MJCF, loaded via the `mujoco_model` hardware param)

```xml
<mujoco model="vacuum_gripper_scene">
  <compiler fusestatic="false"/>
  <include file="vacuum_robot.xml"/>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.01" pos="0 0 0" material="groundplane"/>
    <!-- arm base mount at z=1.2, in +X -->
    <body name="base" pos="0 0 1.2"/>
    <!-- parts: free-joint boxes resting on the floor, top face at z=0.04 -->
    <body name="part1" pos="2.0 0 0.02">
      <freejoint/>
      <inertial mass="0.5" diaginertia="0.001 0.001 0.001"/>
      <geom type="box" size="0.05 0.05 0.02" rgba="1 0 0 1"/>
    </body>
    <body name="part2" pos="1.6 0 0.02">
      <freejoint/>
      <inertial mass="0.5" diaginertia="0.001 0.001 0.001"/>
      <geom type="box" size="0.05 0.05 0.02" rgba="0 1 0 1"/>
    </body>
  </worldbody>
  <equality>
    <weld name="vacuum_weld_part1" body1="vacuum_pad" body2="part1" active="false"/>
    <weld name="vacuum_weld_part2" body1="vacuum_pad" body2="part2" active="false"/>
  </equality>
</mujoco>
```

(Cosmetic extras — skybox/headlight/groundplane texture — may be copied from `scenes/scene.xml`.)

Geometry is chosen so the neutral pose (joint1 = joint2 = 0) hangs the pad bottom face at
z ≈ 1.02 directly above `part1`'s top face (z = 0.04): a plain descent picks part1 without any
reaching; part2 (x = 1.6) requires bending joint2. The two parts are 0.4 m apart in the arm's
planar workspace.

### 6.2 `vacuum_robot.xml` (robot MJCF, included by the scene)

Two-link planar arm, mirroring `demo_resources/robot/test_robot.xml` proportions:

- `upperarm`: revolute `joint1` (axis 0 1 0) at the base; box geom 1.0 × 0.1 × 0.1 centered at
  (0.5, 0, 0).
- `forearm`: revolute `joint2` (axis 0 1 0) at (1, 0, 0); same box geom.
- `vacuum_pad`: body at (1.0, 0, -0.15) relative to `forearm` (→ world (2.0, 0, 1.05) at
  neutral), **cylinder geom** `size="0.04 0.03"` (radius 0.04, half-length 0.03), local axis =
  world Z (no quat) so the flat faces point up/down; bottom face at z ≈ 1.02.
- No actuators in the MJCF — the arm is driven through ros2_control position interfaces
  (joint names `joint1`/`joint2` must match the URDF).

### 6.3 `vacuum_robot.urdf` (xacro, for ros2_control + robot_state_publisher)

Mirrors `test_robot.urdf` structure, reduced:

- Links: `world`, `base` (fixed at 0 0 1.2), `upperarm`, `forearm`, `vacuum_pad` (fixed below the
  forearm tip, same offsets as the MJCF).
- Joints: `joint1`, `joint2` — revolute, axis (0 1 0), limits ±3.14 rad.
- `<ros2_control>`: hardware tag with
  `<param name="mujoco_model">$(find mujoco_ros2_control_demos)/demo_resources/vacuum_gripper/vacuum_scene.xml</param>`
  and `<param name="headless">$(arg headless)</param>`; `joint1`/`joint2` each with a `position`
  command interface and position/velocity/effort/torque state interfaces.
- xacro args: `headless` (default `false`). No camera/lidar/sensors, no gripper fingers, no
  transmissions.

Keep URDF joint names identical to MJCF joint names (ros2_control maps by name).

### 6.4 `config/controllers_vacuum_gripper.yaml`

Copy the `controller_manager` (update_rate 100) + `joint_state_broadcaster` + `position_controller`
blocks from `config/controllers.yaml` (forward_command_controller, interface `position`,
joints `joint1` + `joint2`). Nothing else.

### 6.5 `config/mujoco_ros2_control_plugins_vacuum_gripper.yaml`

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      vacuum_part1:
        type: "mujoco_ros2_control_plugins/VacuumGripperPlugin"
        gripper_body: "vacuum_pad"
        part_body: "part1"
        eq_name: "vacuum_weld_part1"
      vacuum_part2:
        type: "mujoco_ros2_control_plugins/VacuumGripperPlugin"
        gripper_body: "vacuum_pad"
        part_body: "part2"
        eq_name: "vacuum_weld_part2"
      free_joint_state_publisher:
        type: "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin"
        body_names: ["part1", "part2"]
        publish_rate: 50.0
```

Two plugin instances demonstrate the one-instance-per-pair model and per-instance service
namespaces (`/simulator/vacuum_part1/...`, `/simulator/vacuum_part2/...`). The
FreeJointStatePublisher gives the verification script a deterministic part-pose topic.
(The pad can't be in contact with both parts at once, so cross-activation is impossible in
practice; keep the parts ≥ 0.3 m apart.)

### 6.6 `launch/06_vacuum_gripper_plugin.launch.py`

Follow `05_base_velocity_plugin.launch.py`:

- `headless` launch arg (default `false`).
- `robot_state_publisher` with the xacro'd `vacuum_robot.urdf`.
- `mujoco_ros2_control` `ros2_control_node` with `use_sim_time`, the controllers file, and the
  plugins file (node name `simulator`, as in demo 05).
- Spawners: `joint_state_broadcaster`, `position_controller`.
- Module docstring: resource list + copy-pasteable usage:

```bash
ros2 launch mujoco_ros2_control_demos 06_vacuum_gripper_plugin.launch.py headless:=true
# lower the pad onto part1 (commands are Float64MultiArray [joint1, joint2]):
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.05, 0.75]"
ros2 service call /simulator/vacuum_part1/activate std_srvs/srv/Trigger "{}"
ros2 service call /simulator/vacuum_part1/weld_state mujoco_ros2_control_msgs/srv/WeldState "{}"
# lift:
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]"
ros2 service call /simulator/vacuum_part1/release std_srvs/srv/Trigger "{}"
# watch part poses:
ros2 topic echo /simulator/free_joint_states
```

(Exact joint values for "on the part" are implementation-tuning; pick values that put the pad in
contact, and record them in the launch docstring.)

## 7. Verification (acceptance criteria for the implementation session)

All of the following must pass:

1. **Build**: `colcon build` (or at least `--packages-select mujoco_ros2_control_msgs
   mujoco_ros2_control_plugins mujoco_ros2_control_demos`) succeeds; linters pass.
2. **Headless launch smoke**: `ros2 launch mujoco_ros2_control_demos 06_vacuum_gripper_plugin.launch.py
   headless:=true` starts cleanly; both plugin instances log successful init;
   `ros2 service list | grep vacuum` shows 6 services (activate/release/weld_state × 2 instances).
3. **Negative activate**: before descending,
   `ros2 service call /simulator/vacuum_part1/activate std_srvs/srv/Trigger "{}"` →
   `success: false`, message mentions not-in-contact. `weld_state` → all false/inactive.
4. **Pick part1**: descend until contact (verify via `/simulator/free_joint_states` that part1 is
   still at floor z, and via repeated `weld_state` after activating):
   - `activate` → `success: true`
   - `weld_state` → `weld_active: true`, `vacuum_enabled: true`
   - command neutral/up joint targets → part1's z rises with the pad (assert
     `z_part1 > z_floor + 0.05` from the topic).
   - `release` → `weld_state` → `weld_active: false`; part1 falls back to floor z.
5. **Pick part2** (same sequence, instance `vacuum_part2`, reaching joint values) — proves
   per-instance namespacing and that instance 1 stays inactive.
6. **Set the part down**: the weld has no auto-release — a rigid weld pins the part, so
   setting it down in the demo flow requires an explicit `release` (then move the arm away
   and the part falls). The persist-until-release semantics are covered by the unit test
   `WeldStaysActiveWhenPartMovesAway` (part dragged away while latched: the weld stays
   active and the constraint pulls the part back to the held pose).
7. **ResetWorld**: with the weld active and the vacuum latched,
   `ros2 service call /mujoco_ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld`
   → success; then `weld_state` → `vacuum_enabled: false`, `weld_active: false`; parts back at
   initial poses. (The reset preserves simulation time for ROS clock continuity, so the plugin
   is notified through the `world_reset()` hook — see implementation deviations below.)

**Implementation deviations (documented by the implementation session):**

1. **Part positions 1.6 / 1.3 m, not 2.0 / 1.6 m**: with the arm base at z = 1.2 m and two
   1.0 m links, x = 2.0 is unreachable (verified numerically). The two parts remain 0.3 m
   apart, as §6.5 requires.
2. **Service/topic FQNs**: plugin sub-nodes expose their services at the top level under the
   instance key (`/vacuum_part1/activate`, `/part_state_publisher/free_joint_states`), not
   under a `/simulator` prefix as sketched above.
3. **No auto-release**: the contact-loss release of the §4.5 sketch drops held parts mid-lift,
   because the surface contact pair can fall out of MuJoCo's contact margin while the part
   rides rigidly on the weld. The implementation therefore never auto-releases: once engaged,
   the weld stays active until `release` or a world reset, and the constraint itself keeps the
   part pinned (see the note after the `pre_step` listing in §4.5).
4. **Core changes (beyond §8's "no core changes" scope, required for acceptance step 7)**:
   - `MujocoSimulation::reset_world_state` now also restores `mjData::eq_active` from
     `mjModel::eq_active0` — the qpos/qvel/ctrl restore left runtime-activated equality
     constraints (plugin welds) active across a world reset.
   - A new `world_reset(mjData*)` hook on `MuJoCoROS2ControlPluginBase` (default no-op) is
     invoked by the system interface after every reset; `VacuumGripperPlugin` overrides it to
     clear the latched vacuum. Resets deliberately preserve simulation time, so
     time-rewind detection in `pre_step` cannot see them (kept only as a defensive fallback).
5. **MuJoCo 3.8.1 API**: `eq_data`/`eq_active0`/`eq_active` live where §9 says, but `mjData`
   has no `model` pointer (plugins keep the `const mjModel*` from `init`), `body_qposadr` is
   gone (use `jnt_qposadr[body_jntadr[...]]`), and `inertial` elements require a `pos`
   attribute. See the `mujoco_ros2_control-lwj.1` ticket comments for the full fact list.

A small throwaway script (rclpy or `ros2` CLI + sleeps) driving steps 3–7 is the expected vehicle;
commit it if it is more than ~50 lines, as
`mujoco_ros2_control_demos/test_vacuum_gripper_pick.py`, otherwise keep the commands in the launch
docstring / `doc/tutorials.rst`.

Also update `mujoco_ros2_control_demos/doc/tutorials.rst` (new Tutorial 6 section) and
`mujoco_ros2_control_plugins/doc/plugins.rst` + README table row (user-facing plugin docs).

## 8. Out of scope

- Anything beyond one plugin class + one srv + the demo above.
- Runtime creation/modification of eq elements (MJCF-declared only).
- One instance managing multiple grippers or multiple parts.
- Vacuum force modeling, leak/noise dynamics, seal quality — the weld is binary.
- Changes to `mujoco_ros2_control` core (loader, simulation interface).

## 9. Verified MuJoCo facts (from research ticket `tpr.4`, MuJoCo 3.8.1)

- Weld eq type `mjEQ_WELD`; enumerate via `model->eq_type[i]`, `eq_obj1id`, `eq_obj2id`.
  `eq_active0` holds the MJCF `active` initial value.
- `eq_data` layout and no-snap recipe: §4.6 (verified against `src/engine/engine_setconst.c`
  `mj_equalityAnchors` / `setconst`).
- Runtime toggle: write `mjData::eq_active[i]` (mjtByte) in `pre_step`; engine gate
  `if (!d->eq_active[i]) continue;`. No public enable/disable API in 3.8.1.
- Contacts: `mjData::ncon` / `contact[i]` (`dist`, `geom[2]`); geom→body via
  `mjModel::geom_bodyid`.
- Reset: `eq_active` is `mjSTATE_USER` state — `mj_resetData` restores it from `eq_active0`.
- Threading: plugin `pre_step` runs on the physics thread immediately before `mj_step`;
  `init` gets `const mjModel*`; `mjData::model` gives read access to the model in `pre_step`.
