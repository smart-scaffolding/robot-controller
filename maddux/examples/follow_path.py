import numpy as np
from maddux.environment import Environment
from maddux.objects import Obstacle, Ball
from maddux.robots import simple_human_arm, inchworm, RobotBehavior, PathPlanner
import logging
import os
from math import pi
import serial
import time

block_size = 3.0


def arm_animation():
    logging.basicConfig(level=logging.DEBUG,
                       format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                       datefmt='%m-%d %H:%M',
                       filename=os.getcwd() + "/app.log",
                       filemode='w')
    # define a Handler which writes INFO messages or higher to the sys.stderr
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    logging.info('Jackdaws love my big sphinx of quartz.')


    q0 = np.array([3.14143349, 1.77881778, 2.11949991, 0.20802146])
    base_pos = np.array([0.5, 0., 1.0])

    robot_behavior = RobotBehavior(q0, base_pos)

    blueprint = np.array([
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        ])

    building_implemented = np.array([
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

    left_to_build = (blueprint == building_implemented)

    result = np.where(left_to_build == False)

    listOfCoordinates = list(zip(result[0], result[1], result[2]))

    print("{} BLOCKS REMAINING".format(len(listOfCoordinates)))

    start = (2, 0, 0)
    for index, cord in enumerate(listOfCoordinates):
        print("\tBlock {}: ({}, {}, {})".format(
            index, cord[0], cord[1], cord[2]))
        if index > 0:
            start = goal
        goal = (listOfCoordinates[index])



        path_planner = PathPlanner(start, goal, blueprint)

        route = path_planner.get_path()
        color = path_planner.display_path()

    # q0 = np.array([0.7, 1.9, 1.1, 0, 1.5])


    # ball = Ball(np.array([4.5, 0., 1.1]), 0.15, target=True)
    # ball_2 = Ball(np.array([6.5, 0., 1.1]), 0.15, target=True)
    # ball_3 = Ball(np.array([8.5, 0., 1.0]), 0.15, target=True)
    # ball_4 = Ball(np.array([8., 0., 1.1]), 0.15, target=True)
    # ball_5 = Ball(np.array([8., 0., 2.1]), 0.15, target=True)

    # q = robot_behavior.robot.ikineConstrained(ball.position)
    # robot_behavior.robot_animation_cache.append(robot_behavior.robot)
    # robot_behavior.robot.update_angles(q)
        sample_path = []
        for i in route[1:]:
            sample_path.append([i[0] + 0.5, i[1], i[2]+ 1.0])
        robot_behavior.follow_path(sample_path)
        robot_behavior.place_block([goal[0]+0.5, goal[1], goal[2]+1])

    # path_planner = PathPlanner(goal, start, blueprint)
    # route = path_planner.get_path()
    # color = path_planner.display_path()
    # sample_path = []
    #
    # for i in route:
    #     sample_path.append([i[0] + 0.5, i[1]-0.2, i[2]+ 1.1])
    # robot_behavior.follow_path(sample_path)

    robot_behavior.show_behavior(building_implemented, color)

if __name__ == '__main__':
    arm_animation()
