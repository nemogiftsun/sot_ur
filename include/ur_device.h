#ifndef UR_DEVICE_H
#define UR_DEVICE_H

#include <sot/core/device.hh>
#include <urdf_model/joint.h>
//#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/robot.h>
#include <ros/node_handle.h>


namespace sot_ur {

    typedef boost::shared_ptr<urdf::Joint> UrdfJointPtr;
    typedef boost::shared_ptr<pr2_mechanism_model::JointState> Pr2JointPtr;
    typedef std::pair<UrdfJointPtr,Pr2JointPtr> jointAssociation_t;
    typedef std::map<std::string, jointAssociation_t> jointMap_t;

    class UrDevice : public dynamicgraph::sot::Device {
        DYNAMIC_GRAPH_ENTITY_DECL();
    public:
        UrDevice(const std::string &name);
        virtual ~UrDevice();

        virtual bool init();

        virtual void setup(jointMap_t &jm);

        virtual void control(jointMap_t &jm);

    private:
        static const double TIMESTEP_DEFAULT;
        double timestep_;
    };
}

#endif
