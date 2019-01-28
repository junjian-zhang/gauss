#!/usr/bin/env python

import os

#
# Feature to set a name for the robot (different than "Gauss")
# - Will appear on Wi-Fi and ROS connection (todo)
#


# default value, will be replaced with value in setup launch file
FILENAME = '~/gauss_saved_values/robot_name.txt' 

def get_filename_robot_name():
    return FILENAME

def set_filename_robot_name(filename):
    global FILENAME
    FILENAME = filename

def read_robot_name():
    print "read_robot_name: "+FILENAME
    if os.path.isfile(FILENAME):
        print "FILENAME: "+FILENAME
        with open(FILENAME, 'r') as f:
            for line in f:
                print line
                if not (line.startswith('#') or len(line) == 0): 
                    name = line.rstrip() 
                    print "robot name: "+name
                    return name
    return ''

def write_robot_name(name):
    with open(FILENAME, 'w') as f:
        comment =  "# THIS IS A GENERATED FILE\n"
        comment += "# Here is the name the user gave to this robot\n"
        comment += "# This name does not affect anything,\n"
        comment += "# it is only useful for user to easily recognize the robot on desktop/mobile apps\n"
        f.write(comment)
        f.write(name)
