#!/usr/bin/env python

import rospy
import subprocess

from gauss_msgs.srv import SetInt

def send_hotspot_command():
    rospy.loginfo("HOTSPOT")
    send_led_state(5)
    rospy.wait_for_service('/gauss/wifi/set_hotspot')
    try:
        set_hotspot = rospy.ServiceProxy('/gauss/wifi/set_hotspot', SetInt)
        set_hotspot()
    except rospy.ServiceException, e:
        rospy.logwarn("Could not call set_hotspot service")


def send_trigger_sequence_autorun():
    rospy.loginfo("Trigger sequence autorun from button")
    try:
        rospy.wait_for_service('/gauss/sequences/trigger_sequence_autorun', 2)
        trigger = rospy.ServiceProxy('/gauss/sequences/trigger_sequence_autorun', SetInt)
        trigger(1) # value doesn't matter, it will switch state on the server
    except (rospy.ServiceException, rospy.ROSException), e:
        return


def send_shutdown_command():
    rospy.loginfo("SHUTDOWN")
    send_led_state(1)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/gauss/activate_learning_mode', 1)
    except rospy.ROSException, e:
        pass
    try:
        activate_learning_mode = rospy.ServiceProxy('/gauss/activate_learning_mode', SetInt)
        activate_learning_mode(1)
    except rospy.ServiceException, e:
        pass
    rospy.loginfo("Command 'sudo shutdown now'")
    try: 
        output = subprocess.check_output(['sudo', 'shutdown', 'now'])
    except subprocess.CalledProcessError:
        rospy.loginfo("Can't exec shutdown cmd")

def send_led_state(state):
    rospy.wait_for_service('/gauss/rpi/set_led_state')
    try:
        set_led = rospy.ServiceProxy('/gauss/rpi/set_led_state', SetInt)
        set_led(state)
    except rospy.ServiceException, e:
        rospy.logwarn("Could not call set_led_state service")

