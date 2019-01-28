#!/usr/bin/env python

from gauss_msgs.msg import TrajectoryPlan

class Trajectory: 

    def __init__(self, id = 0 , name = "", description = "", 
            trajectory_plan = TrajectoryPlan() ):
        self.id = id
        self.name = name
        self.description = description
        self.trajectory_plan = trajectory_plan 




