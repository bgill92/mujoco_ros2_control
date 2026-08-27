// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__VACUUM_GRIPPER_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__VACUUM_GRIPPER_PLUGIN_HPP_

#include <atomic>
#include <string>

#include <mujoco_ros2_control_msgs/srv/weld_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Plugin that implements one vacuum suction pickup.
 *
 * One instance owns one suction pickup: a pair of MJCF bodies (gripper side + part side)
 * joined by a weld equality constraint (`mjEQ_WELD`) that is declared in MJCF with
 * `active="false"` and activated at runtime by this plugin.
 *
 * The gripper latches a *vacuum* state via the `~/activate` trigger (rejected unless the
 * bodies are in contact). The weld engages on the next physics step and then **stays
 * active until `~/release` or a world reset** — there is no auto-release: the weld
 * constraint keeps the part pinned, and a part dragged away while latched is pulled back
 * to the held pose by the constraint itself. `~/reset_world` (the core ResetWorld service)
 * clears everything.
 *
 * Services (namespaced per instance, e.g. `/mujoco_ros2_control_node/vacuum_part1/...`):
 * - `~/activate`   (std_srvs/srv/Trigger) — latch the vacuum. Rejected with `success=false`
 *   unless the bodies are in contact at the latest physics step.
 * - `~/release`    (std_srvs/srv/Trigger) — clear the latched vacuum (weld drops next step).
 * - `~/weld_state` (mujoco_ros2_control_msgs/srv/WeldState) — query weld/vacuum status.
 *
 * MJCF authoring convention (see doc/plugins.rst):
 * - `<weld name="<eq_name>" body1="<gripper_body>" body2="<part_body>" active="false"/>`
 *   with `body1` = gripper-side body, `body2` = part-side body (free-joint body in practice).
 * - `active="false"` is mandatory: the plugin is the only thing that ever activates the eq,
 *   and world resets must restore the inactive state.
 * - A fixed (joint-less) gripper body (e.g. the suction pad) must survive compilation as a
 *   named body: MuJoCo >= 3.x defaults to fusestatic="false" and never fuses eq-referenced
 *   bodies, but set `<compiler fusestatic="false"/>` in the root scene for portability to
 *   older MuJoCo versions (whose default fuses such bodies away).
 *
 * Threading: all mjModel/mjData mutation happens exclusively in `pre_step()` (physics
 * thread). Service callbacks (executor threads) touch only atomics. No locks in `pre_step()`.
 */
class VacuumGripperPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  VacuumGripperPlugin() = default;
  ~VacuumGripperPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void pre_step(mjData* data) override;
  void world_reset(mjData* data) override;
  void cleanup() override;

private:
  using WeldState = mujoco_ros2_control_msgs::srv::WeldState;
  using Trigger = std_srvs::srv::Trigger;

  /// Re-engage the weld eq at the current configuration (no-snap eq_data rewrite, see spec §4.6).
  void engage(mjData* data, const mjModel* model);

  // Service callbacks — run in ROS executor threads, touch only atomics.
  void handleActivate(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handleRelease(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handleWeldState(const WeldState::Request::SharedPtr request, WeldState::Response::SharedPtr response);

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("VacuumGripperPlugin")};
  rclcpp::Service<Trigger>::SharedPtr activate_service_;
  rclcpp::Service<Trigger>::SharedPtr release_service_;
  rclcpp::Service<WeldState>::SharedPtr weld_state_service_;

  // Model pointer (const, valid for the simulation lifetime). Lookups only.
  const mjModel* model_{nullptr};

  // Body/eq ids, resolved once in init() (model is immutable for the simulation lifetime).
  int gripper_body_id_{ -1 };
  int part_body_id_{ -1 };
  int eq_id_{ -1 };

  // Body names, kept for log/service messages.
  std::string gripper_body_name_;
  std::string part_body_name_;
  std::string eq_name_;

  // Cross-thread state (atomics only; see class doc).
  std::atomic_bool vacuum_enabled_{false};  ///< latched vacuum (activate / release / reset)
  std::atomic_bool in_contact_{false};      ///< refreshed every pre_step from live contacts
  std::atomic_bool weld_active_{false};     ///< mirrors data->eq_active[eq_id_]
  std::atomic_bool pending_activate_{false};  ///< set by activate callback, drained in pre_step

  double prev_step_time_{-1.0};  ///< defensive reset detection (time backwards); primary path
                                 ///< is the world_reset() hook (resets preserve sim time)
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__VACUUM_GRIPPER_PLUGIN_HPP_
