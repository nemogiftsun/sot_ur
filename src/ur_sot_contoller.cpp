#include "my_controller_pkg/my_controller_file.h"
#include <pluginlib/class_list_macros.h>

namespace sot_ur {


/// Controller initialization in non-realtime
bool URSotController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("MyController could not find joint named '%s'",
              joint_name.c_str());
    return false;
  }
  return true;
}


/// Controller startup in realtime
void URSotController::starting()
{
  sotDEBUGOUT(25) ;
  init_pos_ = joint_state_->position_;
  sotDEBUGOUT(25) ;
}


/// Controller update loop in realtime
void URSotController::update()
{
  sotDEBUGOUT(25) ;
  double desired_pos = init_pos_ + 15 * sin(ros::Time::now().toSec());
  double current_pos = joint_state_->position_;
  joint_state_->commanded_effort_ = -10 * (current_pos - desired_pos);
  sotDEBUGOUT(25) ;
}


/// Controller stopping in realtime
void URSotController::stopping()
{}
} // namespace

