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

#include "vacuum_gripper_plugin.hpp"

#include <string>

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>

namespace
{
// The hardware node declares the `mujoco_plugins.<plugin_key>.*` overrides on itself and
// hands plugins a sub-node; the sub-node addresses those parameters by full name (same
// convention as BaseVelocityPlugin). With an empty sub-namespace (unit tests) the name is
// used as-is.
std::string namespacedParamName(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  const std::string sub_ns = node->get_sub_namespace();
  return sub_ns.empty() ? name : ("mujoco_plugins." + sub_ns + "." + name);
}
}  // namespace

namespace mujoco_ros2_control_plugins
{

bool VacuumGripperPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  model_ = model;

  // Required parameters. Declaring with an empty default means a missing parameter reads as
  // "", which fails the checks below (an empty body name is invalid anyway).
  const std::string gripper_body_name = namespacedParamName(node_, "gripper_body");
  const std::string part_body_name = namespacedParamName(node_, "part_body");
  const std::string eq_name_name = namespacedParamName(node_, "eq_name");
  if (!node_->has_parameter(gripper_body_name))
  {
    node_->declare_parameter(gripper_body_name, std::string());
  }
  if (!node_->has_parameter(part_body_name))
  {
    node_->declare_parameter(part_body_name, std::string());
  }
  if (!node_->has_parameter(eq_name_name))
  {
    node_->declare_parameter(eq_name_name, std::string("vacuum_weld"));
  }

  gripper_body_name_ = node_->get_parameter(gripper_body_name).as_string();
  part_body_name_ = node_->get_parameter(part_body_name).as_string();
  if (gripper_body_name_.empty())
  {
    RCLCPP_ERROR(
      logger_, "Required parameter 'gripper_body' (MJCF body name, gripper side) is missing.");
    return false;
  }
  if (part_body_name_.empty())
  {
    RCLCPP_ERROR(logger_, "Required parameter 'part_body' (MJCF body name, part side) is missing.");
    return false;
  }
  eq_name_ = node_->get_parameter(eq_name_name).as_string();

  // Resolve ids once; the model is immutable for the simulation lifetime. mj_name2id returns
  // -1 for unknown names.
  gripper_body_id_ = mj_name2id(model, mjOBJ_BODY, gripper_body_name_.c_str());
  part_body_id_ = mj_name2id(model, mjOBJ_BODY, part_body_name_.c_str());
  eq_id_ = mj_name2id(model, mjOBJ_EQUALITY, eq_name_.c_str());

  if (gripper_body_id_ == -1)
  {
    RCLCPP_ERROR(logger_,
                 "gripper_body '%s' not found in the MJCF model. If the body is fixed "
                 "(joint-less), check that it survived compilation: set <compiler "
                 "fusestatic=\"false\"/> in the root scene file (required on MuJoCo < 3.x, "
                 "whose default fuses static bodies away).",
                 gripper_body_name_.c_str());
    return false;
  }
  if (part_body_id_ == -1)
  {
    RCLCPP_ERROR(logger_, "part_body '%s' not found in the MJCF model.", part_body_name_.c_str());
    return false;
  }
  if (eq_id_ == -1)
  {
    RCLCPP_ERROR(logger_, "eq_name '%s' not found among the model's equality elements.",
                 eq_name_.c_str());
    return false;
  }
  if (gripper_body_id_ == part_body_id_)
  {
    RCLCPP_ERROR(logger_, "gripper_body '%s' and part_body '%s' resolve to the same body.",
                 gripper_body_name_.c_str(), part_body_name_.c_str());
    return false;
  }
  if (model->eq_type[eq_id_] != mjEQ_WELD)
  {
    RCLCPP_ERROR(logger_,
                 "eq_name '%s' is not a weld equality constraint (type %d, expected %d).",
                 eq_name_.c_str(), static_cast<int>(model->eq_type[eq_id_]),
                 static_cast<int>(mjEQ_WELD));
    return false;
  }
  if (model->eq_objtype[eq_id_] != mjOBJ_BODY)
  {
    RCLCPP_ERROR(logger_, "eq_name '%s' is not a body-body equality constraint.", eq_name_.c_str());
    return false;
  }
  const int obj1 = model->eq_obj1id[eq_id_];
  const int obj2 = model->eq_obj2id[eq_id_];
  if (!((obj1 == gripper_body_id_ && obj2 == part_body_id_) ||
        (obj1 == part_body_id_ && obj2 == gripper_body_id_)))
  {
    RCLCPP_ERROR(
      logger_,
      "eq_name '%s' does not join gripper_body '%s' and part_body '%s' (it joins body ids %d and "
      "%d).",
      eq_name_.c_str(), gripper_body_name_.c_str(), part_body_name_.c_str(), obj1, obj2);
    return false;
  }

  activate_service_ =
    node_->create_service<Trigger>("activate",
                                   std::bind(&VacuumGripperPlugin::handleActivate, this,
                                             std::placeholders::_1, std::placeholders::_2));
  release_service_ =
    node_->create_service<Trigger>("release",
                                   std::bind(&VacuumGripperPlugin::handleRelease, this,
                                             std::placeholders::_1, std::placeholders::_2));
  weld_state_service_ =
    node_->create_service<WeldState>("weld_state",
                                     std::bind(&VacuumGripperPlugin::handleWeldState, this,
                                               std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    logger_,
    "VacuumGripperPlugin initialised: gripper_body='%s' (id %d), part_body='%s' (id %d), "
    "eq_name='%s' (id %d). Services: '%s', '%s', '%s'.",
    gripper_body_name_.c_str(), gripper_body_id_, part_body_name_.c_str(), part_body_id_,
    eq_name_.c_str(), eq_id_, activate_service_->get_service_name(),
    release_service_->get_service_name(), weld_state_service_->get_service_name());

  return true;
}

void VacuumGripperPlugin::pre_step(mjData* data)
{
  const mjModel* model = model_;

  // 1) Defensive world-reset detection: if simulation time ever goes backwards (a reset
  //    path that did NOT go through world_reset()), clear the latched vacuum. The primary
  //    reset path is the world_reset() hook — ResetWorld preserves sim time (ROS clock
  //    continuity), so this branch is normally cold. eq_active itself is restored to
  //    eq_active0 by the core on every reset.
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
  //    in contact, and then STAYS active until ~/release or a world reset: there is no
  //    auto-release. The weld constraint keeps the part pinned — a part dragged away
  //    while latched is pulled back to the held pose by the constraint itself.
  if (!vacuum_enabled_)
  {
    if (data->eq_active[eq_id_])
    {
      data->eq_active[eq_id_] = 0;  // released or reset
    }
  }
  else if (in_contact && !data->eq_active[eq_id_])
  {
    engage(data, model);  // no-snap eq_data write, then activate
  }
  weld_active_ = data->eq_active[eq_id_];
}

void VacuumGripperPlugin::engage(mjData* data, const mjModel* model)
{
  // Do NOT assume which of gripper/part is obj1 vs obj2 — use the model's mapping.
  const int b1 = model->eq_obj1id[eq_id_];
  const int b2 = model->eq_obj2id[eq_id_];
  // Note: eq_data lives on the MODEL (MuJoCo >= 3.x); it is the constraint target and is not
  // touched by mj_resetData, so it survives world resets. engage() re-derives every field it
  // owns from the current pose on each call, so stale values can never leak through.
  mjtNum* eqd = model->eq_data + eq_id_ * mjNEQDATA;

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
  const mjtNum* q1 = data->xquat + b1 * 4;
  const mjtNum* q2 = data->xquat + b2 * 4;
  eqd[6] = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  eqd[7] = q1[0] * q2[1] - q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2];
  eqd[8] = q1[0] * q2[2] + q1[1] * q2[3] - q1[2] * q2[0] - q1[3] * q2[1];
  eqd[9] = q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] - q1[3] * q2[0];
  // eqd[10] (torquescale): leave the author's MJCF value untouched.

  data->eq_active[eq_id_] = 1;
}

void VacuumGripperPlugin::handleActivate(
  const Trigger::Request::SharedPtr /*request*/, Trigger::Response::SharedPtr response)
{
  if (!in_contact_.load())
  {
    response->success = false;
    response->message = "activate ignored: bodies '" + gripper_body_name_ + "' and '" +
                        part_body_name_ + "' are not in contact";
    return;
  }
  pending_activate_ = true;
  response->success = true;
  response->message = "vacuum latched; weld engages on the next physics step";
}

void VacuumGripperPlugin::handleRelease(
  const Trigger::Request::SharedPtr /*request*/, Trigger::Response::SharedPtr response)
{
  const bool was_latched = vacuum_enabled_.exchange(false);
  pending_activate_ = false;
  response->success = true;
  response->message = was_latched ? "vacuum released" : "no vacuum latched";
}

void VacuumGripperPlugin::handleWeldState(
  const WeldState::Request::SharedPtr /*request*/, WeldState::Response::SharedPtr response)
{
  const bool weld = weld_active_.load();
  const bool vacuum = vacuum_enabled_.load();
  response->weld_active = weld;
  response->vacuum_enabled = vacuum;
  if (weld)
  {
    response->message = "weld active";
  }
  else if (vacuum)
  {
    response->message = "vacuum on, not in contact";
  }
  else
  {
    response->message = "inactive";
  }
}

void VacuumGripperPlugin::world_reset(mjData* /*data*/)
{
  // The core has restored qpos/qvel/ctrl and eq_active to their MJCF/initial state; drop
  // our latched vacuum so the plugin starts from the authored state (weld off, no vacuum).
  vacuum_enabled_ = false;
  pending_activate_ = false;
}

void VacuumGripperPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "VacuumGripperPlugin cleanup.");
  activate_service_.reset();
  release_service_.reset();
  weld_state_service_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::VacuumGripperPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
