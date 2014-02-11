/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2013, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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

/* Author: Acorn Pooley, Ioan Sucan, Adam Leeper */

#include <moveit/robot_interaction/robot_imarker_handler.h>

//#include <moveit/robot_interaction/robot_interaction.h>
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

robot_interaction::RobotIMarkerHandler::RobotIMarkerHandler(
      const robot_state::RobotState &state,
      const RobotInteractionContextPtr& context)
: LockedRobotState(state)
, context_(context)
, next_id_(0)
{}

robot_interaction::RobotIMarkerHandler::RobotIMarkerHandler(
      const robot_model::RobotModelConstPtr &model,
      const RobotInteractionContextPtr& context)
: LockedRobotState(model)
, context_(context)
, next_id_(0)
{}

robot_interaction::RobotIMarkerHandler::~RobotIMarkerHandler()
{}

int robot_interaction::RobotIMarkerHandler::add(
      const IMarkerPtr& marker)
{
}

void robot_interaction::RobotIMarkerHandler::remove(
      int marker_id)
{
}

void robot_interaction::RobotIMarkerHandler::clear()
{
}

void robot_interaction::RobotIMarkerHandler::setStateChangedCallback(
      StateChangedFn& callback)
{
  state_changed_callback_ = callback;
}

void robot_interaction::RobotIMarkerHandler::stateChanged()
{
  if (state_changed_callback_)
    state_changed_callback_(*this);
}



