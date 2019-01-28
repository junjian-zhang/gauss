#!/usr/bin/env python

import rospy, actionlib

from gauss_msgs.msg import ToolAction
from gauss_msgs.msg import ToolGoal

from gauss_commander.command_status import CommandStatus
from gauss_commander.robot_commander_exception import RobotCommanderException
from actionlib_msgs.msg import GoalStatus

class ToolCommander:

    def __init__(self, logger):
        self.action_client = actionlib.SimpleActionClient(
            'gauss/tool_action', ToolAction)
        rospy.loginfo("Waiting for action server : gauss/tool_action...")
        self.action_client.wait_for_server()
        rospy.loginfo("Found action server : gauss/tool_action")

        rospy.loginfo("Tool Commander has been started")
        self.gauss_ros_logger = logger

    def send_tool_command(self, cmd):
        goal = self.create_goal(cmd)
        self.action_client.send_goal(goal)
        rospy.loginfo("Tool command sent")
        self.gauss_ros_logger.publish_log_status("INFO", "ToolCommander Tool command sent")

        # wait for goal transition to DONE
        self.action_client.wait_for_result() 

        # if goal has been rejected/aborted, stop tracking it and return error
        if self.has_problem():
            self.gauss_ros_logger.publish_log_status("ERROR", "ToolCommander Tool command sent has_problem")
            message = self.action_client.get_result().message
            self.action_client.stop_tracking_goal()
            self.gauss_ros_logger.publish_log_status("ERROR", message)
            raise RobotCommanderException(CommandStatus.TOOL_FAILED, message)

    def stop_tool_command(self):
        pass
        #todo cancel goal

    def create_goal(self, cmd):
        goal = ToolGoal()
        goal.cmd = cmd
        return goal

    # Returns LOST if this SimpleActionClient isn't tracking a goal. 
    # see documentation : http://docs.ros.org/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a6bdebdd9f43a470ecd361d2c8b743188
    def get_command_status(self):
        return self.action_client.get_state()

    def has_problem(self):
        status = self.get_command_status()
        #rospy.loginfo("STATUS : " + str(status))
        return (status == GoalStatus.ABORTED or status == GoalStatus.REJECTED)

