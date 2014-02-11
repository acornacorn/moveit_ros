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

#ifndef MOVEIT__ROBOT_INTERACTION__ROBOT_IMARKER_HANDLER_H
#define MOVEIT__ROBOT_INTERACTION__ROBOT_IMARKER_HANDLER_H

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
//#include <interactive_markers/menu_handler.h>
#include <moveit/robot_state/robot_state.h>
//#include <moveit/macros/class_forward.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
//#include <tf/tf.h>
#include <moveit/robot_interaction/locked_robot_state.h>
#include <moveit/robot_interaction/robot_interaction_context.h>

namespace robot_interaction
{

class IMarker;
typedef boost::shared_ptr<IMarker> IMarkerPtr;


// Manage interactive markers for controlling one robot state.
//
// Any number of IMarker objects can be added.  Each IMarker object represents
// one interactive marker which can be manipulated to adjust the robot state.
class RobotIMarkerHandler : public LockedRobotState
{
public:
  RobotIMarkerHandler(const robot_state::RobotState &state,
                     const RobotInteractionContextPtr& context);

  RobotIMarkerHandler(const robot_model::RobotModelConstPtr &model,
                     const RobotInteractionContextPtr& context);

  virtual ~RobotIMarkerHandler();

  /// Add one interactive marker.
  // This interaction is defined by the IMarker subclass.
  // \returns an id for removing this interaction
  int add(const IMarkerPtr& marker);

  /// Remove one interactive marker.
  /// @param marker_id the id returned when add was called.
  void remove(int marker_id);

  /// Remove all interactive markers.
  void clear();

  /// Get the default context for this handler.
  RobotInteractionContextPtr getContext() const;

  /// Function type for callback when RobotState changes.
  typedef boost::function<void(RobotIMarkerHandler& handler)> StateChangedFn;

  /// Set a function to be called whenever the RobotState changes
  void setStateChangedCallback(StateChangedFn& callback);

protected:
  // This is called (by LockedRobotState) when the internally maintained state
  // has changed.
  virtual void stateChanged();

private:
  RobotInteractionContextPtr context_;

  // Protects the markers_ map and next_id_
  mutable boost::mutex markers_mutex_;

  // List of active IMarkers, indexed by their integer id
  // PROTECTED BY markers_mutex_
  // NOTE: markers_mutex_ protects the map but not the IMarkers in the map.
  std::map<int,IMarkerPtr> markers_;

  // The next available IMarker id
  // PROTECTED BY markers_mutex_
  int next_id_;

  // Callback to indicate state has changed
  // TODO: protect access?
  StateChangedFn state_changed_callback_;
};

typedef boost::shared_ptr<RobotIMarkerHandler> RobotIMarkerHandlerPtr;
typedef boost::shared_ptr<const RobotIMarkerHandler> RobotIMarkerHandlerConstPtr;

} // namespace robot_interaction

inline robot_interaction::RobotInteractionContextPtr
robot_interaction::RobotIMarkerHandler::getContext() const
{
  return context_;
}


#endif
