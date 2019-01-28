#!/usr/bin/env python

import rospy
import threading 

from gauss_rpi.rpi_ros_utils import send_shutdown_command 

from gauss_msgs.srv import SetInt


class ShutdownManager: 
    
    def callback_shutdown_rpi(self, req):
        if req.value==1:  
            send_shutdown_command_thread = threading.Timer(1.0,send_shutdown_command)
            send_shutdown_command_thread.start()
            return {'status': 200, 'message': 'Robot is shutting down'}
        return {'status': 400, 'message': 'Robot is not shutting down : try request value :1 to shutdown rpi'}

    def __init__(self):
        self.shutdown_rpi_sever=rospy.Service('/gauss/rpi/shutdown_rpi', SetInt, self.callback_shutdown_rpi) 
        rospy.loginfo("Shutdown Manager OK")


