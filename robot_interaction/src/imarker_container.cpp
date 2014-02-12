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

#include <moveit/robot_interaction/imarker_container.h>
#include <moveit/robot_interaction/imarker.h>

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

robot_interaction::IMarkerContainer::IMarkerContainer(
      InteractiveMarkerServerPtr server)
: int_marker_server_(server)
{}

robot_interaction::IMarkerContainer::~IMarkerContainer()
{
  // remove IMarkers while int_marker_server_ is still valid
  clear();
}

int robot_interaction::RobotIMarkerHandler::insert(
      const IMarkerPtr& imarker)
{
  visualization_msgs::InteractiveMarker marker;
  interactive_markers::FeedbackCallback callback;
  imarker->getMarkerMsg(marker);
  imarker->getMarkerFeedbackCallback(callback);
  marker.name = imarker.getNarkerName();

  // insert the marker into the server.
  // Using the shared pointer imarker forces the server to keep a reference to
  // the IMarker until the IMarker has been removed from the server.
  //
  // The marker will not actually be broadcast until
  // int_marker_server_->applyChanges() is called later.
  int_marker_server_->insert(marker, boost::bind(&IMarker::feedbackFunction,
                                                 imarker,
                                                 _1));

  boost::unique_lock<boost::mutex> lock(lock_);
  imarkers_[imarker.getNarkerName()] = imarker;
}

void robot_interaction::RobotIMarkerHandler::erase(
      const IMarkerPtr& imarker)
{
  {
    boost::unique_lock<boost::mutex> lock(lock_);
    imarkers_.erase(imarker.getNarkerName());
  }

  // This will usually delete the IMarker since the server holds a reference to
  // it in the callback closure.
  //
  // The removal (and deletion) does not actually occur until
  // int_marker_server_->applyChanges() is called later.
  int_marker_server_->erase(imarker.getNarkerName());
}

void robot_interaction::RobotIMarkerHandler::clear()
{
  boost::unique_lock<boost::mutex> lock(lock_);
  while (!imarkers_.empty())
  {
    std::map<std::string, IMarkerPtr>::iterator it = imarkers_.begin();
    IMarker& imarker = *it->second;
    imarkers_.erase(it);

    // This schedules the IMarker to be deleted next time
    // int_marker_server_->applyChanges() is called.
    int_marker_server_->erase(imarker.getNarkerName());
  }
}

void robot_interaction::RobotIMarkerHandler::setPose(
      const IMarkerPtr& imarker,
      const geometry_msgs::Pose& pose)
{
  int_marker_server_->setPose(imarker->getNarkerName(), pose);
}
