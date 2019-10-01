import numpy as np
from maddux.environment import Environment
from maddux.objects import Obstacle, Ball
from maddux.robots import simple_human_arm, inchworm
from math import pi
import serial
import time

block_size = 3.0

def arm_animation():
    """Animate the arm moving to touch a ball"""

    # Declare a human arm

    # q0 = np.array([0.5, 0.2, 0, 0.5, 1.5])
    # arm = simple_human_arm(2.0, 2.0, q0, np.array([2.0, 2.0, 0.0]))

    q0 = np.array([0.7, 1.9, 1.1, 0, 1.5])
    base_pos = np.array([0.5, 0., 1.1])


    arm = inchworm(q0, base_pos)

    # q0_2 = np.array([0.7, 1.9, 1.1, 0, 1.5])
    # base_pos_2 = np.array([10., 10., 0.])
    # seg_lens_2 = np.array([2.0, 4.0, 4.0, 2.0])
    #
    # arm2 = inchworm(seg_lens_2, q0_2, base_pos_2)
    
    # Create a ball as our target
    ball = Ball(np.array([6.5, 0., 1.1]), 0.15, target=True)
    ball_2 = Ball(np.array([4.5, 0., 1.1]), 0.15, target=True)
    ball_3 = Ball(np.array([8., 0., 2.1]), 0.15, target=True)
    ball_4 = Ball(np.array([8., 0., 1.1]), 0.15, target=True)
    ball_5 = Ball(np.array([8., 0., 2.1]), 0.15, target=True)
    
    # Create our environment
    env = Environment([10.0, 10.0, 10.0], dynamic_objects=[ball, ball_2, ball_3, ball_4],
                      robot=[arm])

    # plots = []
    robots = []
    # Run inverse kinematics to find a joint config that lets arm touch ball
    # arm.ikine(ball.position)
    # arm2.ikine(ball_2.position)

    # arm.ikineConstrained(ball.position)
    q = arm.ikineConstrained(ball.position)
    
    # Animate
    # animate_prior_q = arm.qs
    # plots.append(env.animate(5.0, robot=arm))
    # print "Plots: {}".format(plots)


    robots.append(arm)
    arm.update_angles(q)

    new_pos = arm.end_effector_position()
    joint = q
    joint[0] = joint[0] + pi
    arm = inchworm(joint, new_pos)
    arm.qs = []
    q = arm.ikineConstrained(ball_2.position)

    robots.append(arm)
    arm.update_angles(q)

    new_pos = arm.end_effector_position()
    joint = q
    joint[0] = joint[0] + pi
    arm = inchworm(joint, new_pos)
    arm.qs = []
    q = arm.ikineConstrained(ball_3.position)
    robots.append(arm)
    arm.update_angles(q)


    # new_pos = arm.end_effector_position()
    joint = q
    arm = inchworm(joint, new_pos)
    arm.qs = []
    q = arm.ikineConstrained(ball_4.position)
    robots.append(arm)
    arm.update_angles(q)


    # new_pos = arm.end_effector_position()
    joint = q
    arm = inchworm(joint, new_pos)
    arm.qs = []
    q = arm.ikineConstrained(ball_5.position)
    robots.append(arm)
    arm.update_angles(q)



    obstacles = [Obstacle([8., 0., 0], [7., 1.0, 1]),
                 Obstacle([1, 2, 1], [4, 3, 1.5])]

    # env = Environment([20.0, 20.0, 5.0], dynamic_objects=[ball, ball_2, ball_3, ball_4],
    #                   robot=[arm], static_objects=obstacles,)



    env.animate(robot=robots, display_plot=True, obstacles=[(4, obstacles[0])])
    # plots.append(env.animate(5.0, robot=arm))
    # print "Plots: {}".format(plots)
    # env.show(plots)

if __name__ == '__main__':
    arm_animation()
