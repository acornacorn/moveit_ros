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
class RobotIMarkerHandler : public LockedRobotState,
                            public IMarkerContainer
{
public:
  /// Function type for callback when RobotState changes.
  typedef boost::function<void(RobotIMarkerHandler& handler)> StateChangedFn;

  RobotIMarkerHandler(const robot_state::RobotState &state,
                      const RobotInteractionContextPtr& context,
                      const std::string& handler_name,
                      StateChangedFn state_changed_function);

  RobotIMarkerHandler(const robot_model::RobotModelConstPtr &model,
                      const RobotInteractionContextPtr& context,
                      const std::string& handler_name,
                      StateChangedFn state_changed_function);

  virtual ~RobotIMarkerHandler();

  // Return the RobotInteractionContext associated with this handler.
  RobotInteractionContextPtr getContext() const;

  /// Return the handler's name.
  const std::string& getName() const { return name_; }

  /// remove all IMarkers from this handler.
  void clear();

protected:
  // This is called (by LockedRobotState) when the internally maintained state
  // has changed.
  virtual void stateChanged();

private:
  // name of handler
  const std::string name_;

  // Callback to indicate state has changed
  const StateChangedFn state_changed_callback_;
};

typedef boost::shared_ptr<RobotIMarkerHandler> RobotIMarkerHandlerPtr;
typedef boost::shared_ptr<const RobotIMarkerHandler> RobotIMarkerHandlerConstPtr;

} // namespace robot_interaction

inline robot_interaction::RobotInteractionContextPtr
robot_interaction::RobotIMarkerHandler::getContext() const
{
  return IMarkerContainer::getContext();
}

inline void robot_interaction::RobotIMarkerHandler::clear()
{
  IMarkerContainer::clear();
}


#endif
