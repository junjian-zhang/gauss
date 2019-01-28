#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <thread>

#include <ros/ros.h>

#include "gauss_driver/communication/communication_base.h"
#include "gauss_driver/rpi/rpi_diagnostics.h"
#include "gauss_driver/change_hardware_version.h"

#include "gauss_msgs/SetInt.h"
#include "gauss_msgs/SetLeds.h"

#include "gauss_msgs/PingDxlTool.h"
#include "gauss_msgs/OpenGripper.h"
#include "gauss_msgs/CloseGripper.h"
#include "gauss_msgs/PullAirVacuumPump.h"
#include "gauss_msgs/PushAirVacuumPump.h"

#include "gauss_msgs/ChangeHardwareVersion.h"

#include "gauss_msgs/HardwareStatus.h"
#include "gauss_msgs/SoftwareVersion.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

class RosInterface {

    public:

        RosInterface(CommunicationBase* gauss_comm, RpiDiagnostics* rpi_diagnostics,
                bool *flag_reset_controllers, bool learning_mode_on, int hardware_version);

        void startServiceServers();
        void startPublishers();
        void startSubscribers();

    private:

        CommunicationBase* comm;
        RpiDiagnostics* rpi_diagnostics;
        ros::NodeHandle nh_;

        bool* flag_reset_controllers;
        int hardware_version;
        bool learning_mode_on;
        int calibration_needed;

        std::string rpi_image_version;
        std::string ros_gauss_version;
    
        // publishers

        ros::Publisher hardware_status_publisher;
        boost::shared_ptr<std::thread> publish_hardware_status_thread;

        ros::Publisher software_version_publisher;
        boost::shared_ptr<std::thread> publish_software_version_thread;

        ros::Publisher learning_mode_publisher;
        boost::shared_ptr<std::thread> publish_learning_mode_thread;

        // publish methods
        
        void publishHardwareStatus();
        void publishSoftwareVersion();
        void publishLearningMode();
        
        // services

        ros::ServiceServer calibrate_motors_server;
        ros::ServiceServer request_new_calibration_server;

        ros::ServiceServer activate_learning_mode_server;
        ros::ServiceServer activate_leds_server;

        ros::ServiceServer ping_and_set_dxl_tool_server;
        ros::ServiceServer open_gripper_server;
        ros::ServiceServer close_gripper_server;
        ros::ServiceServer pull_air_vacuum_pump_server;
        ros::ServiceServer push_air_vacuum_pump_server;

        ros::ServiceServer change_hardware_version_server;

        // callbacks
        
        bool callbackCalibrateMotors(gauss_msgs::SetInt::Request &req, gauss_msgs::SetInt::Response &res);
        bool callbackRequestNewCalibration(gauss_msgs::SetInt::Request &req, gauss_msgs::SetInt::Response &res);

        bool callbackActivateLearningMode(gauss_msgs::SetInt::Request &req, gauss_msgs::SetInt::Response &res);
        bool callbackActivateLeds(gauss_msgs::SetLeds::Request &req, gauss_msgs::SetLeds::Response &res);
        
        bool callbackPingAndSetDxlTool(gauss_msgs::PingDxlTool::Request &req, gauss_msgs::PingDxlTool::Response &res);

        bool callbackOpenGripper(gauss_msgs::OpenGripper::Request &req, gauss_msgs::OpenGripper::Response &res);
        bool callbackCloseGripper(gauss_msgs::CloseGripper::Request &req, gauss_msgs::CloseGripper::Response &res);

        bool callbackPullAirVacuumPump(gauss_msgs::PullAirVacuumPump::Request &req, gauss_msgs::PullAirVacuumPump::Response &res);
        bool callbackPushAirVacuumPump(gauss_msgs::PushAirVacuumPump::Request &req, gauss_msgs::PushAirVacuumPump::Response &res);

        bool callbackChangeHardwareVersion(gauss_msgs::ChangeHardwareVersion::Request &req,
                gauss_msgs::ChangeHardwareVersion::Response &res);

};

#endif
