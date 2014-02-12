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

#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>


//#include <interactive_markers/menu_handler.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/macros/class_forward.h>
//#include <boost/function.hpp>
//#include <tf/tf.h>

namespace interactive_markers
{
class MenuHandler;
}

namespace robot_interaction
{

class RobotIMarkerHandler;
class RobotInteractionContext;
typedef boost::shared_ptr<RobotInteractionContext> RobotInteractionContextPtr;
typedef boost::shared_ptr<interactive_markers::MenuHandler> MenuHandlerPtr;
class KinematicOptions;
typedef boost::shared_ptr<const KinematicOptions> KinematicOptionsConstPtr;

/// Represents one interactive marker for a RobotIMarkerHandler.
///
/// Subclasses should be implemented
//
class IMarker : private boost::noncopyable,
                public boost::enable_shared_from_this<IMarker>
{
public:
  IMarker(RobotIMarkerHandler& handler,
          const std::string& name);
  virtual ~IMarker();

  //. Get the handler managing this IMarker.
  RobotIMarkerHandler& getHandler();
  const RobotIMarkerHandler& getHandler() const;

  /// Get the context to use for this marker.
  /// Subclasses can use this to get information they need to implement the above functions
  const RobotInteractionContext& getContext() const;

  /// get marker name for this interaction
  const std::string& getMarkerName() const;
  /// get name of this interaction (set in constructor)
  const std::string& getName() const;

  /// Set kinematic options specific to this IMarkerIK.
  /// If this is not called the KinematicOptions from RobotInteractionContext
  /// will be used.
  void setKinematicOptions(const KinematicOptions& options);

  /// Get kinematic options.
  /// If getKinematicOptions() was called, returns those options.  Otherwise
  /// returns options from the RobotInteractionContext.
  KinematicOptions getKinematicOptions() const;


  /// remove the marker from the InteractiveMarkerServer and destroy it.
  void destroy();

  // get the marker message
  void getMarkerMsg(visualization_msgs::InteractiveMarker& marker);

  // the feedback function called by the InteractiveMarkerServer
  void feedbackFunction(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // Called when the state has changed and we need to update the pose of the marker.
  void updatePose();

protected:
  /// Create the interactive marker.
  /// Subclasses should override this with code that creates the interactive marker.
  ///  @param state the current robot state.
  ///  @param marker the interactive marker that this function will fill in.
  /// This is called with lock_ already held.
  virtual void doConstructMarker(const robot_state::RobotState &state,
                                 visualization_msgs::InteractiveMarker& marker) = 0;

  /// Get menu handler, or NULL if marker should have no menu.
  /// Subclasses should override this if they want the interactive marker to
  /// have a right-click context menu.
  /// This is called with lock_ already held.
  virtual MenuHandlerPtr doGetMenuHandler();

  /// Process feedback from an interactive marker.
  /// Subclasses should override this with code that handles feedback.
  ///  @param feedback_msg the message that describes how the interactive marker moved.
  ///  @param state the state which should be modified according to how the marker moved.
  /// This is called with lock_ already held.
  virtual void doProcessFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback_msg,
                                 robot_state::RobotState &state) = 0;

  /// Update the pose of the marker given a new RobotState
  /// Subclasses should override this with code that updates the pose of the
  /// interactive marker when the RobotState changes.
  ///  @param state (in) the new robot state.
  ///  @param pose (out) the new pose of the marker based on the state.
  /// This is called with lock_ already held.
  virtual void doUpdatePose(const robot_state::RobotState &state,
                            geometry_msgs::Pose& pose) = 0;

  // locks access to mutable state
  mutable boost::mutex lock_;

private:
  RobotInteractionContextPtr context_;
  RobotIMarkerHandler& handler_;

  // interactivity name set in constructor.
  // Never changes -- no locking needed.
  const std::string name_;

  // marker name.
  // Never changes after construction -- no locking needed.
  const std::string marker_name_;

  // Parameters for RobotState::setFromIK().
  // PROTECTED BY lock_
  KinematicOptionsConstPtr kinematic_options_;

  // set to true when destroy() is called.
  // This prevents feedbackFunction() from running after destroy() is called.
  // PROTECTED BY lock_
  bool destroyed_;
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
