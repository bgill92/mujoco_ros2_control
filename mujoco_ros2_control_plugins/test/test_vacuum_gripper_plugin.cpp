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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_msgs/srv/weld_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "vacuum_gripper_plugin.hpp"

namespace
{
// A fixed "gripper" cylinder (top face at z = 1.05) and a free-joint "part" cylinder whose
// origin can be placed touching/penetrating the gripper. Extra bodies/eqs exercise the init
// validation paths (mismatched pair, wrong eq type).
constexpr const char* kMjcf = R"(
<mujoco model="vacuum_gripper_test">
  <compiler fusestatic="false"/>
  <worldbody>
    <body name="gripper" pos="0 0 1">
      <geom type="cylinder" size="0.1 0.05"/>
    </body>
    <body name="part" pos="0 0 1.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="cylinder" size="0.08 0.05"/>
    </body>
    <body name="other" pos="0.5 0 1">
      <geom type="sphere" size="0.05"/>
    </body>
    <body name="slider" pos="-1 0 1">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.05"/>
    </body>
  </worldbody>
  <equality>
    <weld name="vacuum_weld" body1="gripper" body2="part" active="false"/>
    <weld name="vacuum_weld_other" body1="gripper" body2="other" active="false"/>
    <joint name="joint_eq_test" joint1="j1" polycoef="0 1 0 0 0"/>
  </equality>
</mujoco>
)";

// Part origin z for the bottom face to penetrate the gripper top face (z = 1.05) by 1 mm.
constexpr double kContactPartZ = 1.05 + 0.05 - 0.001;
// Part origin z far below the gripper: no contact.
constexpr double kClearPartZ = 0.5;
}  // namespace

class VacuumGripperPluginTest : public ::testing::Test
{
protected:
  using Trigger = std_srvs::srv::Trigger;
  using WeldState = mujoco_ros2_control_msgs::srv::WeldState;

  static void SetUpTestCase()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("vacuum_gripper_test_node");
    plugin_node_ = node_->create_sub_node("vacuum_plugin");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 2);
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    char error[1024] = { 0 };
    mjSpec* spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;

    model_ = mj_compile(spec, nullptr);
    if (model_ == nullptr)
    {
      const char* ce = mjs_getError(spec);
      mj_deleteSpec(spec);
      FAIL() << (ce ? ce : "mj_compile failed");
    }
    mj_deleteSpec(spec);

    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
    mj_forward(model_, data_);

    gripper_body_id_ = mj_name2id(model_, mjOBJ_BODY, "gripper");
    part_body_id_ = mj_name2id(model_, mjOBJ_BODY, "part");
    eq_id_ = mj_name2id(model_, mjOBJ_EQUALITY, "vacuum_weld");
    ASSERT_NE(gripper_body_id_, -1);
    ASSERT_NE(part_body_id_, -1);
    ASSERT_NE(eq_id_, -1);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    executor_.reset();
    plugin_node_.reset();
    node_.reset();
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
  }

  /// Place the part's free-joint origin at the given z (identity orientation) and refresh
  /// forward kinematics + contacts without stepping the simulation.
  void placePartAt(double z)
  {
    const int part_qadr = model_->jnt_qposadr[model_->body_jntadr[part_body_id_]];
    data_->qpos[part_qadr + 2] = z;
    mj_forward(model_, data_);
  }

  // Mirrors the running system: the plugin reads parameters under the full
  // `mujoco_plugins.<sub_namespace>.*` name, and plugin_node_'s sub-namespace is
  // "vacuum_plugin".
  static constexpr const char* kParamPrefix = "mujoco_plugins.vacuum_plugin.";

  /// Declare-or-set a plugin parameter (a test may init several plugins on the same node).
  void setParam(const std::string& name, const std::string& value)
  {
    const std::string full = std::string(kParamPrefix) + name;
    if (plugin_node_->has_parameter(full))
    {
      plugin_node_->set_parameter(rclcpp::Parameter(full, value));
    }
    else
    {
      plugin_node_->declare_parameter(full, value);
    }
  }

  bool initPlugin(const std::string& gripper, const std::string& part, const std::string& eq_name)
  {
    if (gripper != "unset")
    {
      setParam("gripper_body", gripper);
    }
    if (part != "unset")
    {
      setParam("part_body", part);
    }
    if (eq_name != "unset")
    {
      setParam("eq_name", eq_name);
    }
    mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
    const bool ok = plugin.init(plugin_node_, model_, data_);
    plugin.cleanup();
    return ok;
  }

  bool callActivate(Trigger::Response::SharedPtr& response)
  {
    return callService<Trigger>("activate", response);
  }

  bool callRelease(Trigger::Response::SharedPtr& response)
  {
    return callService<Trigger>("release", response);
  }

  bool callWeldState(WeldState::Response::SharedPtr& response)
  {
    auto client = plugin_node_->create_client<WeldState>("weld_state");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return false;
    }
    auto future = client->async_send_request(std::make_shared<WeldState::Request>());
    if (!waitFor(future))
    {
      return false;
    }
    response = future.get();
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  int gripper_body_id_{ -1 };
  int part_body_id_{ -1 };
  int eq_id_{ -1 };

private:
  template<typename SrvT>
  bool callService(const std::string& name, typename SrvT::Response::SharedPtr& response)
  {
    auto client = plugin_node_->create_client<SrvT>(name);
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return false;
    }
    auto future = client->async_send_request(std::make_shared<typename SrvT::Request>());
    if (!waitFor(future))
    {
      return false;
    }
    response = future.get();
    return true;
  }

  template<typename FutureT>
  bool waitFor(FutureT& future)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline)
    {
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// init() — id resolution and validation
// ---------------------------------------------------------------------------

TEST_F(VacuumGripperPluginTest, InitSucceeds)
{
  EXPECT_TRUE(initPlugin("gripper", "part", "vacuum_weld"));
}

TEST_F(VacuumGripperPluginTest, InitFailsWithoutRequiredParams)
{
  EXPECT_FALSE(initPlugin("unset", "unset", "unset"));
}

TEST_F(VacuumGripperPluginTest, InitFailsOnUnknownGripperBody)
{
  EXPECT_FALSE(initPlugin("no_such_body", "part", "vacuum_weld"));
}

TEST_F(VacuumGripperPluginTest, InitFailsOnUnknownPartBody)
{
  EXPECT_FALSE(initPlugin("gripper", "no_such_body", "vacuum_weld"));
}

TEST_F(VacuumGripperPluginTest, InitFailsOnUnknownEqName)
{
  EXPECT_FALSE(initPlugin("gripper", "part", "no_such_eq"));
}

TEST_F(VacuumGripperPluginTest, InitFailsOnMismatchedBodyPair)
{
  // vacuum_weld_other joins gripper+other, but the plugin is configured for gripper+part.
  EXPECT_FALSE(initPlugin("gripper", "part", "vacuum_weld_other"));
  // The matching configuration for that eq must succeed.
  EXPECT_TRUE(initPlugin("gripper", "other", "vacuum_weld_other"));
}

TEST_F(VacuumGripperPluginTest, InitFailsOnWrongEqType)
{
  // joint_eq_test is a single-joint equality, not a weld.
  EXPECT_FALSE(initPlugin("gripper", "part", "joint_eq_test"));
}

// ---------------------------------------------------------------------------
// Service contracts + state machine (pre_step driven by hand)
// ---------------------------------------------------------------------------

TEST_F(VacuumGripperPluginTest, ActivateRejectedWhenNotInContact)
{
  placePartAt(kClearPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);  // refresh in_contact_ (no contact at this pose)

  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  EXPECT_FALSE(resp->success);
  EXPECT_NE(resp->message.find("not in contact"), std::string::npos) << resp->message;

  WeldState::Response::SharedPtr ws;
  ASSERT_TRUE(callWeldState(ws));
  EXPECT_FALSE(ws->weld_active);
  EXPECT_FALSE(ws->vacuum_enabled);
  EXPECT_EQ(ws->message, "inactive");

  // No state changed at all: the eq must not be active.
  EXPECT_EQ(data_->eq_active[eq_id_], 0);

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, WeldEngagesOnVacuumAndContact)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);

  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  EXPECT_TRUE(resp->success) << resp->message;

  plugin.pre_step(data_);  // drains pending_activate_, engages the weld
  EXPECT_EQ(data_->eq_active[eq_id_], 1);

  WeldState::Response::SharedPtr ws;
  ASSERT_TRUE(callWeldState(ws));
  EXPECT_TRUE(ws->weld_active);
  EXPECT_TRUE(ws->vacuum_enabled);
  EXPECT_EQ(ws->message, "weld active");

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, EngageWritesEqDataHoldingCurrentPose)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);
  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  ASSERT_TRUE(resp->success);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 1);

  // No-snap: the rewritten eq_data must make the constraint hold at the CURRENT
  // configuration — the anchor points (world frame) computed from both bodies must coincide.
  const mjModel* model = model_;
  mjtNum* eqd = model_->eq_data + eq_id_ * mjNEQDATA;
  const int b1 = model->eq_obj1id[eq_id_];
  const int b2 = model->eq_obj2id[eq_id_];
  const mjtNum* x1 = data_->xpos + b1 * 3;
  const mjtNum* m1 = data_->xmat + b1 * 9;
  const mjtNum* x2 = data_->xpos + b2 * 3;
  const mjtNum* m2 = data_->xmat + b2 * 9;
  const mjtNum p1[3] = {x1[0] + m1[0] * eqd[3] + m1[1] * eqd[4] + m1[2] * eqd[5],
                        x1[1] + m1[3] * eqd[3] + m1[4] * eqd[4] + m1[5] * eqd[5],
                        x1[2] + m1[6] * eqd[3] + m1[7] * eqd[4] + m1[8] * eqd[5]};
  const mjtNum p2[3] = {x2[0] + m2[0] * eqd[0] + m2[1] * eqd[1] + m2[2] * eqd[2],
                        x2[1] + m2[3] * eqd[0] + m2[4] * eqd[1] + m2[5] * eqd[2],
                        x2[2] + m2[6] * eqd[0] + m2[7] * eqd[1] + m2[8] * eqd[2]};
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(p1[i], p2[i], 1e-9) << "anchor mismatch in axis " << i;
  }

  // The relative quaternion must equal conj(q1) * q2 at the current configuration.
  const mjtNum* q1 = data_->xquat + b1 * 4;
  const mjtNum* q2 = data_->xquat + b2 * 4;
  const mjtNum rel[4] = {q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3],
                         q1[0] * q2[1] - q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2],
                         q1[0] * q2[2] + q1[1] * q2[3] - q1[2] * q2[0] - q1[3] * q2[1],
                         q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] - q1[3] * q2[0]};
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(eqd[6 + i], rel[i], 1e-9) << "relative quaternion mismatch in component " << i;
  }

  // Stepping the simulation with the weld active must not snap the part: its position
  // should stay where it was when the weld engaged.
  const int qadr = model_->jnt_qposadr[model_->body_jntadr[part_body_id_]];
  const mjtNum before[3] = {data_->qpos[qadr], data_->qpos[qadr + 1], data_->qpos[qadr + 2]};
  for (int i = 0; i < 50; ++i)
  {
    plugin.pre_step(data_);
    mj_step(model_, data_);
  }
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(data_->qpos[qadr + i], before[i], 0.02) << "part snapped in axis " << i;
  }

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, ContactLossDeactivatesWeldButKeepsVacuum)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);
  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  ASSERT_TRUE(resp->success);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 1);

  // Part swings clear of the pad while the vacuum stays latched.
  placePartAt(kClearPartZ);
  plugin.pre_step(data_);
  EXPECT_EQ(data_->eq_active[eq_id_], 0) << "weld must drop on contact loss";

  WeldState::Response::SharedPtr ws;
  ASSERT_TRUE(callWeldState(ws));
  EXPECT_FALSE(ws->weld_active);
  EXPECT_TRUE(ws->vacuum_enabled) << "vacuum must stay latched on contact loss";
  EXPECT_EQ(ws->message, "vacuum on, not in contact");

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, RecontactReengagesWithoutNewActivate)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);
  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  ASSERT_TRUE(resp->success);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 1);

  placePartAt(kClearPartZ);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 0);

  // Vacuum is still latched: re-establishing contact re-engages the weld.
  placePartAt(kContactPartZ);
  plugin.pre_step(data_);
  EXPECT_EQ(data_->eq_active[eq_id_], 1);

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, ReleaseDeactivatesWeld)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);
  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  ASSERT_TRUE(resp->success);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 1);

  ASSERT_TRUE(callRelease(resp));
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(resp->message, "vacuum released");

  plugin.pre_step(data_);
  EXPECT_EQ(data_->eq_active[eq_id_], 0);

  WeldState::Response::SharedPtr ws;
  ASSERT_TRUE(callWeldState(ws));
  EXPECT_FALSE(ws->weld_active);
  EXPECT_FALSE(ws->vacuum_enabled);
  EXPECT_EQ(ws->message, "inactive");

  // Releasing again reports that no vacuum was latched.
  ASSERT_TRUE(callRelease(resp));
  EXPECT_EQ(resp->message, "no vacuum latched");

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, ResetClearsLatchedVacuum)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);
  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  ASSERT_TRUE(resp->success);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 1);

  // Simulate ResetWorld: mj_resetData restores qpos/eq_active from their initial values
  // and rewinds data->time.
  mj_resetData(model_, data_);
  data_->time = data_->time - 1.0;  // force the backwards-time reset detection
  mj_forward(model_, data_);

  plugin.pre_step(data_);

  WeldState::Response::SharedPtr ws;
  ASSERT_TRUE(callWeldState(ws));
  EXPECT_FALSE(ws->vacuum_enabled) << "reset must clear the latched vacuum";
  EXPECT_FALSE(ws->weld_active);
  EXPECT_EQ(ws->message, "inactive");

  plugin.cleanup();
}

TEST_F(VacuumGripperPluginTest, WorldResetHookClearsLatchedVacuum)
{
  placePartAt(kContactPartZ);
  mujoco_ros2_control_plugins::VacuumGripperPlugin plugin;
  setParam("gripper_body", "gripper");
  setParam("part_body", "part");
  setParam("eq_name", "vacuum_weld");
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  plugin.pre_step(data_);
  Trigger::Response::SharedPtr resp;
  ASSERT_TRUE(callActivate(resp));
  ASSERT_TRUE(resp->success);
  plugin.pre_step(data_);
  ASSERT_EQ(data_->eq_active[eq_id_], 1);

  // Simulate the core's ResetWorld path: eq_active restored to the MJCF default (0) and
  // sim time PRESERVED (no rewind) — the plugin learns of the reset only via the hook.
  data_->eq_active[eq_id_] = 0;
  plugin.world_reset(data_);

  plugin.pre_step(data_);

  WeldState::Response::SharedPtr ws;
  ASSERT_TRUE(callWeldState(ws));
  EXPECT_FALSE(ws->vacuum_enabled) << "world_reset must clear the latched vacuum";
  EXPECT_FALSE(ws->weld_active);
  EXPECT_EQ(ws->message, "inactive");

  plugin.cleanup();
}
