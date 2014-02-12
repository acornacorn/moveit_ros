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

#ifndef MOVEIT__ROBOT_INTERACTION__IMARKER_CONTAINER_H
#define MOVEIT__ROBOT_INTERACTION__IMARKER_CONTAINER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

//#include <interactive_markers/menu_handler.h>
//#include <moveit/macros/class_forward.h>
//#include <boost/function.hpp>
//#include <boost/thread.hpp>
//#include <tf/tf.h>
//#include <moveit/robot_interaction/locked_robot_state.h>
//#include <moveit/robot_interaction/robot_interaction_context.h>

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace robot_interaction
{

class IMarker;
typedef boost::shared_ptr<IMarker> IMarkerPtr;
typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer>
                                             InteractiveMarkerServerPtr;



// Manage a set of IMarkers.
//
// Adding an
//
// Any number of IMarker objects can be added.  Each IMarker object represents
// one interactive marker which can be manipulated to adjust the robot state.
class IMarkerContainer
{
public:
  IMarkerContainer(InteractiveMarkerServerPtr server);
  virtual ~IMarkerContainer();

  // remove all IMarkers
  void clear();

  // get the context
  RobotInteractionContextPtr getContext() const { return context_; }

private:
  friend class IMarker;

  // add an IMarker. 
  // Called by IMarker constructor.
  // Also inserts the marker into the InteractiveMarkerServer.
  void insert(const IMarkerPtr& imarker);

  // add an IMarker.  Called by IMarker constructor.
  void setPose(const IMarkerPtr& imarker,
               const geometry_msgs::Pose& pose);

  // remove an IMarker.  Called by IMarker::destroy()
  void erase(const IMarkerPtr& imarker);

  // lock the container
  mutable boost::mutex imarkers_lock_;

  // Container of imarkers
  // PROTECTED BY imarkers_lock_
  std::map<std::string, IMarkerPtr> imarkers_;

  // marker server
  InteractiveMarkerServerPtr int_marker_server_;
};

} // namespace robot_interaction

#endif
