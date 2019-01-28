#!/usr/bin/env python

import rospy

from joystick_interface import JoystickInterface

from sequence_manager import SequenceManager
from sequence_action_server import SequenceActionServer
from sequence_autorun import SequenceAutorun
from gauss_user_interface.gauss_ros_logger import RosLogger
#from matlab_manager import MatlabManager 

class UserInterface:

    def __init__(self):
        self.gauss_ros_logger = RosLogger()

        # Joystick
        self.joy = JoystickInterface()
        self.joy.disable_joy()
    
        # Sequence Manager
        sequences_dir = rospy.get_param("~sequences_dir")
        self.sequence_manager = SequenceManager(sequences_dir, self.gauss_ros_logger)

        # Sequence Action Server
        self.sequence_action_server = SequenceActionServer(self.sequence_manager, self.gauss_ros_logger)
        self.sequence_action_server.start()

        # Sequence Autorun
        self.sequence_autorun = SequenceAutorun(self.gauss_ros_logger)

        #Matlab node manager 
        #self.matlab_manager=MatlabManager()

        self.gauss_ros_logger.publish_log_status("INFO", "UserInterface start over")


    def shutdown(self):
        self.sequence_manager.shutdown()

if __name__ == '__main__':
    rospy.init_node('user_interface')
    ui = UserInterface()
    rospy.on_shutdown(ui.shutdown)
    rospy.spin()
