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

#ifndef MOVEIT__ROBOT_INTERACTION__IMARKER_IK_H
#define MOVEIT__ROBOT_INTERACTION__IMARKER_IK_H

#include <moveit/robot_interaction/imarker.h>

namespace robot_interaction
{

/// This is a subclass of IMarker that implements IK
class IMarkerIK : public IMarker
{
public:
  IMarkerIK(RobotIMarkerHandler& handler);
  IMarkerIK(RobotIMarkerHandler& handler,
            const RobotInteractionContextPtr& context);

  // Set handler's state using IK.
  bool setStateFromIK(const std::string& group,
                      const std::string& tip,
                      const geometry_msgs::Pose &pose);

  /// Set kinematic options specific to this IMarkerIK.
  /// If this is not called the KinematicOptions from RobotInteractionContext
  /// will be used.
  void setKinematicOptions(const KinematicOptions& options);

  /// Get kinematic options.
  /// If getKinematicOptions() was called, returns those options.  Otherwise
  /// returns options from the RobotInteractionContext.
  KinematicOptions getKinematicOptions() const;

private:
  // Helper function for setStateFromIK()
  void internalSetStateFromIK(robot_state::RobotState* state,
                              const std::string* group,
                              const std::string* tip,
                              const geometry_msgs::Pose* pose,
                              const KinematicOptions* kinematic_options,
                              bool* result);

  void internalSetStateFromIK2(robot_state::RobotState* state);
  
  // Parameters for RobotState::setFromIK().
  // PROTECTED BY IMarker::lock_
  KinematicOptionsConstPtr kinematic_options_;
};

} // namespace robot_interaction

#endif
