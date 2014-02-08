/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  Copyright (c) 2014, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Acorn Pooley, Ioan Sucan */

#include <gtest/gtest.h>
#include <moveit/robot_interaction/locked_robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>



static const char *URDF_STR =
  "<?xml version=\"1.0\" ?>"
  "<robot name=\"one_robot\">"
  "<link name=\"base_link\">"
  "  <inertial>"
  "    <mass value=\"2.81\"/>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
  "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
  "  </inertial>"
  "  <collision name=\"my_collision\">"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </collision>"
  "  <visual>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </visual>"
  "</link>"
  "<joint name=\"joint_a\" type=\"continuous\">"
  "   <axis xyz=\"0 0 1\"/>"
  "   <parent link=\"base_link\"/>"
  "   <child link=\"link_a\"/>"
  "   <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
  "</joint>"
  "<link name=\"link_a\">"
  "  <inertial>"
  "    <mass value=\"1.0\"/>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
  "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
  "  </inertial>"
  "  <collision>"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </collision>"
  "  <visual>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </visual>"
  "</link>"
  "<joint name=\"joint_b\" type=\"fixed\">"
  "  <parent link=\"link_a\"/>"
  "  <child link=\"link_b\"/>"
  "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
  "</joint>"
  "<link name=\"link_b\">"
  "  <inertial>"
  "    <mass value=\"1.0\"/>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
  "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
  "  </inertial>"
  "  <collision>"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </collision>"
  "  <visual>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </visual>"
  "</link>"
  "  <joint name=\"joint_c\" type=\"prismatic\">"
  "    <axis xyz=\"1 0 0\"/>"
  "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.09\" velocity=\"0.2\"/>"
  "    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" soft_lower_limit=\"0.0\" soft_upper_limit=\"0.089\"/>"
  "    <parent link=\"link_b\"/>"
  "    <child link=\"link_c\"/>"
  "    <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
  "  </joint>"
  "<link name=\"link_c\">"
  "  <inertial>"
  "    <mass value=\"1.0\"/>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 .0\"/>"
  "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
  "  </inertial>"
  "  <collision>"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </collision>"
  "  <visual>"
  "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </visual>"
  "</link>"
  "  <joint name=\"mim_f\" type=\"prismatic\">"
  "    <axis xyz=\"1 0 0\"/>"
  "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
  "    <parent link=\"link_c\"/>"
  "    <child link=\"link_d\"/>"
  "    <origin rpy=\" 0.0 0.1 0.0 \" xyz=\"0.1 0.1 0 \"/>"
  "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
  "  </joint>"
  "  <joint name=\"joint_f\" type=\"prismatic\">"
  "    <axis xyz=\"1 0 0\"/>"
  "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
  "    <parent link=\"link_d\"/>"
  "    <child link=\"link_e\"/>"
  "    <origin rpy=\" 0.0 0.1 0.0 \" xyz=\"0.1 0.1 0 \"/>"
  "  </joint>"
  "<link name=\"link_d\">"
  "  <collision>"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </collision>"
  "  <visual>"
  "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </visual>"
  "</link>"
  "<link name=\"link_e\">"
  "  <collision>"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </collision>"
  "  <visual>"
  "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
  "    <geometry>"
  "      <box size=\"1 2 1\" />"
  "    </geometry>"
  "  </visual>"
  "</link>"
  "</robot>";

static const char *SRDF_STR =
  "<?xml version=\"1.0\" ?>"
  "<robot name=\"one_robot\">"
  "<virtual_joint name=\"base_joint\" child_link=\"base_link\" parent_frame=\"odom_combined\" type=\"planar\"/>"
  "<group name=\"base_from_joints\">"
  "<joint name=\"base_joint\"/>"
  "<joint name=\"joint_a\"/>"
  "<joint name=\"joint_c\"/>"
  "</group>"
  "<group name=\"mim_joints\">"
  "<joint name=\"joint_f\"/>"
  "<joint name=\"mim_f\"/>"
  "</group>"
  "<group name=\"base_with_subgroups\">"
  "<group name=\"base_from_base_to_tip\"/>"
  "<joint name=\"joint_c\"/>"
  "</group>"
  "<group name=\"base_from_base_to_tip\">"
  "<chain base_link=\"base_link\" tip_link=\"link_b\"/>"
  "<joint name=\"base_joint\"/>"
  "</group>"
  "</robot>";

static moveit::core::RobotModelPtr getModel()
{
  static moveit::core::RobotModelPtr model;
  if (!model)
  {
    boost::shared_ptr<urdf::ModelInterface> urdf(urdf::parseURDF(URDF_STR));
    boost::shared_ptr<srdf::Model> srdf(new srdf::Model());
    srdf->initString(*urdf, SRDF_STR);
    model.reset(new moveit::core::RobotModel(urdf, srdf));
  }
  return model;
}

TEST(LockedRobotState, load)
{
  moveit::core::RobotModelPtr model = getModel();

  robot_interaction::LockedRobotState ls1(model);

  moveit::core::RobotState state2(model);
  state2.setToDefaultValues();
  robot_interaction::LockedRobotState ls2(state2);
  
  moveit::core::RobotStatePtr state3(new moveit::core::RobotState(model));
  state3->setToDefaultValues();
  robot_interaction::LockedRobotState ls3(state3);
  
  robot_interaction::LockedRobotStatePtr ls4(
                    new robot_interaction::LockedRobotState(model));
}

enum {
  JOINT_A = 3,
  JOINT_C = 4,
  MIM_F   = 5,
  JOINT_F = 6
};

class MyInfo
{
public:
  MyInfo()
    : quit_(false)
  {}

  void setThreadFunc(robot_interaction::LockedRobotState* locked_state,
                        int* counter,
                        double offset);

  void checkThreadFunc(robot_interaction::LockedRobotState* locked_state,
                       int* counter);

  void waitThreadFunc(robot_interaction::LockedRobotState* locked_state,
                      int** counters,
                      int max);

  void checkState(robot_interaction::LockedRobotState &locked_state);

  boost::mutex cnt_lock_;
  boost::condition_variable state_available_condition_;
  bool quit_;
};

// Check the state.  It should always be valid.
void MyInfo::checkState(robot_interaction::LockedRobotState &locked_state)
{
  robot_state::RobotStateConstPtr s = locked_state.getState();
  
  robot_state::RobotState cp1(*s);

  // take some time
  cnt_lock_.lock();
  cnt_lock_.unlock();
  cnt_lock_.lock();
  cnt_lock_.unlock();
  cnt_lock_.lock();
  cnt_lock_.unlock();

  // check mim_f == joint_f
  EXPECT_EQ(s->getVariablePositions()[MIM_F],
            s->getVariablePositions()[JOINT_F] * 1.5 + 0.1);

  robot_state::RobotState cp2(*s);

  EXPECT_NE(cp1.getVariablePositions(), cp2.getVariablePositions());
  EXPECT_NE(cp1.getVariablePositions(), s->getVariablePositions());

  int cnt = cp1.getVariableCount();
  for (int i = 0 ; i < cnt ; ++i)
  {
    EXPECT_EQ(cp1.getVariablePositions()[i], 
              cp2.getVariablePositions()[i]);
    EXPECT_EQ(cp1.getVariablePositions()[i], 
              s->getVariablePositions()[i]);
  }

  // check mim_f == joint_f
  EXPECT_EQ(s->getVariablePositions()[MIM_F],
            s->getVariablePositions()[JOINT_F] * 1.5 + 0.1);
}

// spin, checking the state
void MyInfo::checkThreadFunc(
    robot_interaction::LockedRobotState* locked_state,
    int* counter)
{
  bool go = true;
  while(go)
  {
    for (int loops = 0 ; loops < 100 ; ++loops)
    {
      checkState(*locked_state);
    }

    cnt_lock_.lock();
    go = !quit_;
    ++*counter;
    cnt_lock_.unlock();
  }
}

// spin, setting the state to different values
void MyInfo::setThreadFunc(
    robot_interaction::LockedRobotState* locked_state,
    int* counter,
    double offset)
{
  bool go = true;
  while(go)
  {
    double val = offset;
    for (int loops = 0 ; loops < 100 ; ++loops)
    {
      val += 0.0001;
      robot_state::RobotState cp1(*locked_state->getState());
      
      cp1.setVariablePosition(JOINT_A, val + 0.00001);
      cp1.setVariablePosition(JOINT_C, val + 0.00002);
      cp1.setVariablePosition(JOINT_F, val + 0.00003);

      locked_state->setState(cp1);
    }

    cnt_lock_.lock();
    go = !quit_;
    ++*counter;
    cnt_lock_.unlock();

    checkState(*locked_state);

    val += 0.000001;
  }
}

// spin until all counters reach at least max
void MyInfo::waitThreadFunc(
            robot_interaction::LockedRobotState* locked_state,
            int** counters,
            int max)
{
  bool go = true;
  while(go)
  {
    go = false;
    cnt_lock_.lock();
    for (int i = 0 ; counters[i] ; ++i)
    {
      if (counters[i][0] < max)
        go = true;
    }
    cnt_lock_.unlock();

    checkState(*locked_state);
    checkState(*locked_state);
  }
  cnt_lock_.lock();
  quit_ = true;
  cnt_lock_.unlock();
}

TEST(LockedRobotState, set2)
{
  MyInfo info;

  moveit::core::RobotModelPtr model = getModel();
  robot_interaction::LockedRobotState ls1(model);

#if 0
  for (int i = 0 ; i < ls1.getState()->getVariableCount() ; ++i)
  {
    printf(" var[%3d] = %s\n",i, ls1.getState()->getVariableNames()[i].c_str());
  }
#endif


  int cnt1 = 0;
  int cnt2 = 0;
  int cnt3 = 0;
  int *counters[] = { &cnt1, &cnt2, &cnt3, NULL };

  boost::thread t1(&MyInfo::setThreadFunc,
                   &info,
                   &ls1,
                   &cnt1,
                   .1);

  boost::thread t2(&MyInfo::setThreadFunc,
                   &info,
                   &ls1,
                   &cnt2,
                   .2);

  boost::thread t3(&MyInfo::checkThreadFunc,
                   &info,
                   &ls1,
                   &cnt3);

  boost::thread t4(&MyInfo::waitThreadFunc,
                   &info,
                   &ls1,
                   counters,
                   1000);

  t1.join();
  t2.join();
  t3.join();
  t4.join();

#if 0
  printf("counts: %d %d %d\n",cnt1,cnt2,cnt3);
#endif
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int arg;

  return RUN_ALL_TESTS();
}
