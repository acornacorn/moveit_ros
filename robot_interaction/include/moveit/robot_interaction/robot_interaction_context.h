/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef MOVEIT__ROBOT_INTERACTION__ROBOT_INTERACTION_CONTEXT_H
#define MOVEIT__ROBOT_INTERACTION__ROBOT_INTERACTION_CONTEXT_H

//#include <visualization_msgs/InteractiveMarkerFeedback.h>
//#include <visualization_msgs/InteractiveMarker.h>
//#include <interactive_markers/menu_handler.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/macros/class_forward.h>
//#include <boost/function.hpp>
#include <moveit/robot_interaction/kinematic_options.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/node_handle.h>
//#include <tf/tf.h>
//#include <moveit/robot_interaction/kinematic_options.h>

namespace interactive_markers
{
class InteractiveMarkerServer;
}
namespace tf
{
class Transformer;
}

namespace robot_interaction
{

// Context for interaction with a robot.
class RobotInteractionContext
{
public:
  typedef boost::shared_ptr<tf::Transformer> TransformerPtr;
  typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer>
                                             InteractiveMarkerServerPtr;

  /// Construct with existing InteractiveMarkerServer
  // \param marker_server existing InteractiveMarkerServer to use.
  // \param tf (optional) existing transformer (if not supplied one is created)
  RobotInteractionContext(const TransformerPtr& tf,
                          const InteractiveMarkerServerPtr& marker_server);

  /// Construct
  // \param marker_server_ns (optional) interactive marker topic namespace
  // \param tf (optional) existing transformer (if not supplied one is created)
  RobotInteractionContext(const TransformerPtr& tf,
                          const std::string& marker_server_ns = "");

  virtual ~RobotInteractionContext();

  /// Set options for use in IK calculations
  void setKinematicOptions(const KinematicOptions& kinematic_options);
  KinematicOptions getKinematicOptions() const;


private:
  /// The topic name on which the internal Interactive Marker Server operates
  static const std::string INTERACTIVE_MARKER_TOPIC;

  // Server to maintain interactive markers and communicate with Rviz.
  // DO NOT lock the lock_ member when accessing this.
  InteractiveMarkerServerPtr int_marker_server_;

  // Node handle
  ros::NodeHandle nh_;
  TransformerPtr tf_;
  

  // lock_ must be locked for access to most members of this class.
  //   Exceptions: lock_ need not be locked to access:
  //     int_marker_server_
  //     nh_
  //     tf_
  mutable boost::recursive_mutex lock_;

  // Parameters for calling RobotState::setFromIK()
  // PROTECTED BY lock_
  KinematicOptions kinematic_options_;
};

typedef boost::shared_ptr<RobotInteractionContext> RobotInteractionContextPtr;
typedef boost::shared_ptr<const RobotInteractionContext> RobotInteractionContextConstPtr;


} // namespace robot_interaction

#endif
