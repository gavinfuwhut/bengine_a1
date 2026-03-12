#ifndef _JOINT_CONTROLLER_H_
#define _JOINT_CONTROLLER_H_

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "bengine_legged_msgs/MotorCmd.h"
#include "bengine_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>

#define PMSM      (0x0A)
#define PosStopF  (2.146E+9f)
#define VelStopF  (16000.0f)

namespace bengine_legged_controller{
    class BengineJointController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        private:
            hardware_interface::JointHandle joint;
            ros::Subscriber sub_cmd;
            control_toolbox::Pid pid_controller_;
            boost::scoped_ptr<realtime_tools::RealtimePublisher<bengine_legged_msgs::MotorState> > controller_state_publisher_ ;
        public:
            // bool start_up;
            std::string name_space;
            std::string joint_name;
            urdf::JointConstSharedPtr joint_urdf;
            realtime_tools::RealtimeBuffer<bengine_legged_msgs::MotorCmd> command;
            bengine_legged_msgs::MotorCmd lastCmd;
            bengine_legged_msgs::MotorState lastState;

            BengineJointController();
            ~BengineJointController();
            virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration& period);
            virtual void stopping();
            void setCommandCB(const bengine_legged_msgs::MotorCmdConstPtr& msg);
            void positionLimits(double &position);
            void velocityLimits(double &velocity);
            void effortLimits(double &effort);
            void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
            void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
            void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    };
}

#endif
