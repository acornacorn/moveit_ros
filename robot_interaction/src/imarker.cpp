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

#include <moveit/robot_interaction/imarker.h>
#include <moveit/robot_interaction/robot_imarker_handler.h>
#include <moveit/robot_interaction/kinematic_options.h>

#include <interactive_markers/menu_handler.h>

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

robot_interaction::IMarker::IMarker(
      RobotIMarkerHandler& handler,
      const std::string& name)
: handler_(handler)
, context_(handler.getContext())
, name_(name)
, marker_name_(handler.getName() + ":" + name)
, destroyed_(false)
{
  handler_.IMarkerContainer::insert(shared_from_this());
}

robot_interaction::IMarker::~IMarker()
{}

void robot_interaction::IMarker::destroy()
{
  {
    boost::mutex::scoped_lock lock(lock_);
    destroyed_ = true;
  }
  handler_.IMarkerContainer::erase(shared_from_this());
}

void robot_interaction::IMarker::feedbackFunction(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // TODO: do we need to throttle this if messages come too fast?

  boost::mutex::scoped_lock lock(lock_);
  if (destroyed_)
    return;

  // modify the state in the handler (in place) by calling back to the
  // doProcessFeedback() method.
  handler_.LockedRobotState::modifyState(boost::bind(
                &IMarker::doProcessFeedback,
                this,
                feedback,
                _1));
}

void robot_interaction::IMarker::getMarkerMsg(
      visualization_msgs::InteractiveMarker& marker)
{
}

robot_interaction::MenuHandlerPtr
robot_interaction::IMarker::getMenuHandler()
{
  return MenuHandlerPtr();
}

robot_interaction::KinematicOptions
robot_interaction::IMarker::getKinematicOptions() const
{
  boost::mutex::scoped_lock lock(lock_);
  return kinematic_options_ ? *kinematic_options_ :
                              getContext().getKinematicOptions();
}

void robot_interaction::IMarker::setKinematicOptions(
      const KinematicOptions& options)
{
  boost::mutex::scoped_lock lock(lock_);
  kinematic_options_.reset(new KinematicOptions(options));
}


