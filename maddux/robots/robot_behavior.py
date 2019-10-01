import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from datetime import datetime
import numpy as np
from maddux.environment import Environment
from maddux.objects import Obstacle, Ball
from maddux.robots import simple_human_arm, inchworm
from math import pi
import utils

import logging


class RobotBehavior:

    def __init__(self, initialAngles, basePosition):
        self.robot_animation_cache = []
        self.env = Environment([10.0, 10.0, 10.0])
        self.robot = inchworm(initialAngles, basePosition)
        self.obstacles = []
        self.logger = logging.getLogger('RobotBehavior')

    def follow_path(self, path):
        self.logger.info('FOLLOWING PATH')
        path_length = len(path) - 1
        for index, point in enumerate(path[1:path_length]):
            print "Base Position: {}".format(utils.create_point_from_homogeneous_transform(self.robot.base))
            print "First Step: {}".format(point)



            if index == 0:
                second_step = (point[0]-2, point[1], point[2])
                # second_step = path[index-1]

                print "\t\tSecond Step: {}".format(second_step)
                self.initial_move(point)
                print "Made first move"
                print "\t\tBase Position: {}".format(utils.create_point_from_homogeneous_transform(self.robot.base))

                self.move_to_point(second_step)
                print "Made second move"
                print "\t\tEE Position: {}".format(self.robot.end_effector_position())
            else:

                self.move_to_point(point)
                second_step = path[index-1]
                print "\t\tSecond Step: {}".format(second_step)
                print "\t\tBase Position: {}".format(utils.create_point_from_homogeneous_transform(self.robot.base))

                self.move_to_point(second_step)

    def initial_move(self, destination, new_robot_pos=None):
        if new_robot_pos is not None:
            joint = self.robot.get_current_joint_config()
            self.robot = inchworm(joint, new_robot_pos)
            self.robot.qs = []
        q = self.robot.ikineConstrained(destination)
        self.robot_animation_cache.append(self.robot)
        self.robot.update_angles(q)
        return self.robot

    def move_to_point(self, destination, offset=pi):
        new_pos = self.robot.end_effector_position()
        joint = self.robot.get_current_joint_config()
        joint[0] = joint[0] + offset
        self.robot = inchworm(joint, new_pos)
        self.robot.qs = []
        q = self.robot.ikineConstrained(destination)

        self.robot_animation_cache.append(self.robot)
        self.robot.update_angles(q)
        return self.robot

    def place_block(self, location, direction="Z"):
        if direction == "Z":
            offsetLocation = [location[0], location[1], location[2]+1]

        elif direction == "Y":
            offsetLocation = [location[0], location[1]+1, location[2]]

        else:
        # direction == "X"
            offsetLocation = [location[0]+1, location[1], location[2]]

        print "-" * 20
        print "PLACING BLOCK"
        print "-" * 20
        print "\tRaising Above"
        print "\t\tLocation: {}".format(offsetLocation)
        print "\tSwitching EE to place block"

        new_pos = self.robot.end_effector_position()
        self.move_to_point(offsetLocation)
        # self.initial_move(offsetLocation)
        print "\tLowering to put block down"
        print "\t\tLocation: {}".format(location)
        self.initial_move(location, new_pos)
        self.obstacles.append((len(self.robot_animation_cache), Obstacle([location[0]+0.5, location[1], location[2]-1], [location[0]-0.5, location[1]+1, location[2]])))

        print "\tLifting up again"
        print "\t\tLocation: {}".format(offsetLocation)
        self.initial_move(offsetLocation, new_pos)

        return self.robot

    def show_behavior(self, blueprint, color):
        # obstacles = [Obstacle([8., 0., 0], [7., 1.0, 1]),
        #              Obstacle([1, 2, 1], [4, 3, 1.5])]


        self.env.animate(robot=self.robot_animation_cache, display_plot=True, obstacles=self.obstacles, blueprint=blueprint, colors=color)
