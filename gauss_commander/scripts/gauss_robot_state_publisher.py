#!/usr/bin/env python

import rospy
import tf
from gauss_commander.moveit_utils import get_rpy_from_quaternion
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float64
from gauss_msgs.msg import RobotState
from geometry_msgs.msg import Quaternion

PI = 3.14159

class GaussRobotStatePublisher:

    def get_orientation_from_angles(self, r, p, y):
        quaternion = quaternion_from_euler(r, p, y)
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        return orientation

    def get_robot_pose(self, event):
        try:
            (pos, rot) = self.tf_listener.lookupTransform('base_link', 'flange', rospy.Time(0))  
            self.position = pos
            # self.rpy = get_rpy_from_quaternion(rot)
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            self.rpy[0] = roll
            self.rpy[1] = pitch
            self.rpy[2] = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("catnot get robot pose, TF fail")

    def publish_state(self, event):
        msg = RobotState()
        msg.position.x = self.position[0]
        msg.position.y = self.position[1]
        msg.position.z = self.position[2]
        msg.rpy.roll = self.rpy[0]
        msg.rpy.pitch = self.rpy[1]
        msg.rpy.yaw = self.rpy[2]
        self.gauss_robot_state_publisher.publish(msg)

    def __init__(self):
            
        # Tf listener (position + rpy) of end effector tool
        self.position = [0,0,0]
        self.rpy = [0,0,0]
        self.tf_listener = tf.TransformListener()

        # State publisher
        self.gauss_robot_state_publisher = rospy.Publisher(
                '/gauss/robot_state', RobotState, queue_size=5)

        # Get params from rosparams
        rate_tf_listener = rospy.get_param("/gauss/robot_state/rate_tf_listener")
        rate_publish_state = rospy.get_param("/gauss/robot_state/rate_publish_state")

        rospy.Timer(rospy.Duration(1.0/rate_tf_listener), self.get_robot_pose)
        rospy.Timer(rospy.Duration(1.0/rate_publish_state), self.publish_state)

        rospy.loginfo("Started Gauss robot state publisher")
