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

#ifndef MOVEIT__ROBOT_INTERACTION__IMARKER_H
#define MOVEIT__ROBOT_INTERACTION__IMARKER_H

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <moveit/robot_interaction/kinematic_options.h>


//#include <interactive_markers/menu_handler.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/macros/class_forward.h>
//#include <boost/function.hpp>
#include <boost/thread.hpp>
//#include <tf/tf.h>

namespace interactive_markers
{
class MenuHandler;
}

namespace robot_interaction
{

class KinematicContext;
class RobotIMarkerHandler;
class RobotInteractionContext;
typedef boost::shared_ptr<RobotInteractionContext> RobotInteractionContextPtr;

/// Represents one interactive marker for a RobotIMarkerHandler.
///
/// Subclasses should be implemented
//
class IMarker
{
public:
  IMarker(RobotIMarkerHandler& handler);
  IMarker(RobotIMarkerHandler& handler,
          const RobotInteractionContextPtr& context);
  virtual ~IMarker();

  /// Create the interactive marker.
  /// Subclasses should override this with code that creates the interactive marker.
  ///  @param state the current robot state.
  ///  @param marker the interactive marker that this function will fill in.
  virtual void constructMarker(const robot_state::RobotState &state,
                               visualization_msgs::InteractiveMarker& marker) = 0;

  // Shared pointer to a menu handler
  typedef boost::shared_ptr<interactive_markers::MenuHandler> MenuHandlerPtr;

  /// Get menu handler, or NULL if marker should have no menu.
  /// Subclasses should override this if they want the interactive marker to
  /// have a right-click context menu.
  virtual MenuHandlerPtr getMenuHandler();

  /// Process feedback from an interactive marker.
  /// Subclasses should override this with code that handles feedback.
  ///  @param feedback_msg the message that describes how the interactive marker moved.
  ///  @param state the state which should be modified according to how the marker moved.
  virtual void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback_msg,
                               robot_state::RobotState &state) = 0;

  /// Update the pose of the marker given a new RobotState
  /// Subclasses should override this with code that updates the pose of the
  /// interactive marker when the RobotState changes.
  ///  @param state (in) the new robot state.
  ///  @param pose (out) the new pose of the marker based on the state.
  virtual void updatePose(const robot_state::RobotState &state,
                          geometry_msgs::Pose& pose) = 0;

  // Get the handler managing this IMarker.
  RobotIMarkerHandler& getHandler();
  const RobotIMarkerHandler& getHandler() const;

  /// Get the context to use for this marker.
  /// Subclasses can use this to get information they need to implement the above functions
  const RobotInteractionContext& getContext() const;

private:
  RobotInteractionContextPtr context_;
  RobotIMarkerHandler& handler_;


protected:
  // locks access to mutable state (any subclass member variables that can
  // change after construction).
  mutable boost::mutex lock_;
};

typedef boost::shared_ptr<IMarker> IMarkerPtr;
typedef boost::shared_ptr<const IMarker> IMarkerConstPtr;


} // namespace robot_interaction


inline const robot_interaction::RobotInteractionContext& robot_interaction::IMarker::getContext() const
{
  return *context_;
}
inline robot_interaction::RobotIMarkerHandler& robot_interaction::IMarker::getHandler()
{
  return handler_;
}
inline const robot_interaction::RobotIMarkerHandler& robot_interaction::IMarker::getHandler() const
{
  return handler_;
}

#endif
