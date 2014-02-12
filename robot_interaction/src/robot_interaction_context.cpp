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

#include <moveit/robot_interaction/robot_interaction_context.h>
#include <moveit/robot_interaction/kinematic_options.h>

#include <interactive_markers/interactive_marker_server.h>

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

const std::string
robot_interaction::RobotInteractionContext::INTERACTIVE_MARKER_TOPIC =
                          "robot_interaction_interactive_marker_topic";


robot_interaction::RobotInteractionContext::RobotInteractionContext(
      const TransformerPtr& tf,
      const InteractiveMarkerServerPtr& marker_server)
: tf_(tf)
, int_marker_server_(marker_server)
{ }

robot_interaction::RobotInteractionContext::RobotInteractionContext(
      const TransformerPtr& tf,
      const std::string& marker_server_ns)
: tf_(tf)
{
  std::string topic = marker_server_ns.empty() ?
                      INTERACTIVE_MARKER_TOPIC :
                      marker_server_ns + "/" + INTERACTIVE_MARKER_TOPIC;
  int_marker_server_.reset(new interactive_markers::InteractiveMarkerServer(
                                                                    topic));
}

robot_interaction::RobotInteractionContext::~RobotInteractionContext()
{}

void robot_interaction::RobotInteractionContext::setKinematicOptions(
      const KinematicOptions& kinematic_options)
{
  //boost::unique_lock<boost::recursive_mutex> lock(lock_);
  kinematic_options_ = kinematic_options;
}

robot_interaction::KinematicOptions
robot_interaction::RobotInteractionContext::getKinematicOptions() const
{
  //boost::unique_lock<boost::recursive_mutex> lock(lock_);
  return kinematic_options_;
}

