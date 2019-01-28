#ifndef GAUSS_HARDWARE_INTERFACE_H
#define GAUSS_HARDWARE_INTERFACE_H

#include <boost/shared_ptr.hpp>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <vector>

#include "gauss_driver/communication/communication_base.h"
#include "common/common.h"

extern GaussLogger g_roslogger_pub; 

class GaussHardwareInterface: public hardware_interface::RobotHW {

    public:

        GaussHardwareInterface(CommunicationBase* gauss_comm);
                                                  
        void read();

        void write();

        // custom
        void setCommandToCurrentPosition();

        std::vector<double> getPositionToWrite();
    
    private:

        ros::NodeHandle nh_;

        CommunicationBase* comm;

        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::PositionJointInterface joint_position_interface;
        
        double cmd[6] = { 0, 0.64, -1.39, 0, 0, 0};
        double pos[6] = { 0, 0.64, -1.39, 0, 0, 0};
        double vel[6] = {0};
        double eff[6] = {0};

};

#endif
