#!/usr/bin/env python

import rospy

class Sequence:

    def __init__(self, id=0, name="", description="", blockly_xml="", python_code=""):
        self.id = id
        self.name = name
        self.description = description
        self.created = rospy.Time.now().secs
        self.updated = rospy.Time.now().secs
        self.blockly_xml = blockly_xml
        self.python_code = python_code
