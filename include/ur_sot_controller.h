#ifndef PR2_SOT_CONTROLLER_H
#define PR2_SOT_CONTROLLER_H

// pr2 controller includes
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

// sot includes

namespace sot_ur {

class URSotController : public  pr2_controller_interface::Controller
{
private:
  pr2_mechanism_model::JointState* joint_state_;
  double init_pos_;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};


}
