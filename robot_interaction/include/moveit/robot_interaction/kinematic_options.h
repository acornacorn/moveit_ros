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

#ifndef MOVEIT__ROBOT_INTERACTION__KINEMATIC_OPTIONS_H
#define MOVEIT__ROBOT_INTERACTION__KINEMATIC_OPTIONS_H

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>

namespace robot_interaction
{

// Options for inverse kinematics calculations.
struct KinematicOptions
{
  /// Constructor - set to reasonable default values
  KinematicOptions();

  /// max time an IK attempt can take before we give up.
  double timeout_seconds;

  /// how many attempts before we give up.
  unsigned int max_attempts;

  /// other options
  kinematics::KinematicsQueryOptions options;

  /// This is called to determine if the state is valid
  robot_state::GroupStateValidityCallbackFn state_validity_callback;
};

typedef boost::shared_ptr<KinematicOptions> KinematicOptionsPtr;
typedef boost::shared_ptr<const KinematicOptions> KinematicOptionsConstPtr;

}

inline robot_interaction::KinematicOptions::KinematicOptions()
: timeout_seconds(0.0) // 0.0 = use default timeout
, max_attempts(0)      // 0 = use default max attempts
{ }

#endif
