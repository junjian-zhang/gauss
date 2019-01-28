#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

FAN_1_GPIO = 27
FAN_2_GPIO = 23

class FansManager:

    def setup_fans(self):
        GPIO.setwarnings(False) 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FAN_1_GPIO, GPIO.OUT)
        GPIO.setup(FAN_2_GPIO, GPIO.OUT)
        rospy.sleep(0.05)
        rospy.loginfo("------ RPI FANS SETUP OK ------")
    
    def set_fans(self, activate):
        if activate:
            GPIO.output(FAN_1_GPIO, GPIO.HIGH)
            GPIO.output(FAN_2_GPIO, GPIO.HIGH)
        else:
            GPIO.output(FAN_1_GPIO, GPIO.LOW)
            GPIO.output(FAN_2_GPIO, GPIO.LOW)

    def __init__(self):
        self.setup_fans()
        self.learning_mode_on = True
        # Activate fans for 5 seconds to give an audio signal to the user
        self.set_fans(True)
        rospy.sleep(5)
        self.set_fans(not self.learning_mode_on)

        self.learning_mode_subscriber = rospy.Subscriber(
                '/gauss/learning_mode', Bool, self.callback_learning_mode)


    def callback_learning_mode(self, msg):
        if msg.data != self.learning_mode_on:
            self.learning_mode_on = msg.data
            self.set_fans(not self.learning_mode_on)


