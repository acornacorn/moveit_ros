/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Author: Acorn Pooley */

#include <moveit/robot_interaction/imarker_ik.h>
#include <moveit/robot_interaction/kinematic_options.h>
#include <moveit/robot_interaction/robot_imarker_handler.h>


//#include <moveit/robot_interaction/interactive_marker_helpers.h>
//#include <moveit/transforms/transforms.h>
//#include <interactive_markers/interactive_marker_server.h>
//#include <interactive_markers/menu_handler.h>
//#include <eigen_conversions/eigen_msg.h>
//#include <tf_conversions/tf_eigen.h>
//#include <boost/lexical_cast.hpp>
//#include <boost/math/constants/constants.hpp>
//#include <algorithm>
//#include <limits>
//
//#include <Eigen/Core>
//#include <Eigen/Geometry>

robot_interaction::IMarkerIK::IMarkerIK(
      RobotIMarkerHandler& handler)
: IMarker(handler)
{}

robot_interaction::IMarkerIK::IMarkerIK(
      RobotIMarkerHandler& handler,
      const RobotInteractionContextPtr& context)
: IMarker(handler, context)
{}

robot_interaction::KinematicOptions
robot_interaction::IMarkerIK::getKinematicOptions() const
{
  boost::unique_lock<boost::mutex> lock(lock_);
  return kinematic_options_ ? *kinematic_options_ :
                              getContext().getKinematicOptions();
}

void robot_interaction::IMarkerIK::setKinematicOptions(
      const KinematicOptions& options)
{
  boost::unique_lock<boost::mutex> lock(lock_);
  kinematic_options_.reset(new KinematicOptions(options));
}

bool robot_interaction::IMarkerIK::setStateFromIK(
      const std::string& group,
      const std::string& tip,
      const geometry_msgs::Pose &pose)
{
  bool result;
  KinematicOptions kinematic_options = getKinematicOptions();

#if 0
  getHandler().modifyState(boost::bind(
                  &robot_interaction::IMarkerIK::internalSetStateFromIK,
                  this,
                  _1));
#endif

  // call internalSetStateFromIK() to modify the (locked) RobotState
  getHandler().modifyState(boost::bind(
                  &robot_interaction::IMarkerIK::internalSetStateFromIK,
                  this,
                  _1,
                  &group,
                  &tip,
                  &pose,
                  &kinematic_options,
                  &result));
  return result;
}

// This does not use any members. Called with lock_ unlocked.
// TODO: make this static?
void robot_interaction::IMarkerIK::internalSetStateFromIK(
      robot_state::RobotState* state,
      const std::string* group,
      const std::string* tip,
      const geometry_msgs::Pose* pose,
      const KinematicOptions* kinematic_options,
      bool* result)
{
  const robot_model::JointModelGroup *jmg = state->getJointModelGroup(*group);
  *result = state->setFromIK(jmg,
                            *pose,
                            *tip,
                            kinematic_options->max_attempts,
                            kinematic_options->timeout_seconds,
                            kinematic_options->state_validity_callback,
                            kinematic_options->options);
}
