import numpy as np
from maddux.environment import Environment
from maddux.objects import Ball
from maddux.robots import simple_human_arm, inchworm
from math import pi
import serial
import time

def arm_animation():
    """Animate the arm moving to touch a ball"""

    # Declare a human arm

    # q0 = np.array([0.5, 0.2, 0, 0.5, 1.5])
    # arm = simple_human_arm(2.0, 2.0, q0, np.array([2.0, 2.0, 0.0]))

    q0 = np.array([0.7, 1.9, 1.1, 0])
    qTest = np.array([0,0,0,0])
    base_pos = np.array([0., 0., 0.])
    seg_lens = np.array([2.0, 4.0, 4.0, 2.0])

    arm = inchworm(seg_lens, qTest, base_pos)

    # q0_2 = np.array([0.7, 1.9, 1.1, 0, 1.5])
    # base_pos_2 = np.array([10., 10., 0.])
    # seg_lens_2 = np.array([2.0, 4.0, 4.0, 2.0])
    #
    # arm2 = inchworm(seg_lens_2, q0_2, base_pos_2)
    
    # Create a ball as our target
    ball = Ball(np.array([4, 0., 0.]), 0.15, target=True)
    ball_2 = Ball(np.array([6, 0., 0.]), 0.15, target=True)
    ball_3 = Ball(np.array([7, 1., 0.]), 0.15, target=True)
    ball_4 = Ball(np.array([5, 5., 0.]), 0.15, target=True)
    
    # Create our environment
    env = Environment([20.0, 20.0, 5.0], dynamic_objects=[ball, ball_2, ball_3, ball_4],
                      robot=[arm])

    ser = serial.Serial(port='COM9', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS, timeout=3.0)
    time.sleep(1)

    # Run inverse kinematics to find a joint config that lets arm touch ball
    # arm.ikine(ball.position)
    # arm2.ikine(ball_2.position)

    # arm.ikineConstrained(ball.position)
    q = arm.ikineConstrained(ball.position, ser)
    
    # Animate
    env.animate(5.0, robot=arm)
    #
    # new_pos = arm.end_effector_position()
    # joint = q
    # print joint
    # joint[0] = joint[0] + pi
    # arm = inchworm(seg_lens, joint, new_pos)
    # arm.ikineConstrained(ball_2.position, ser)

    # env.animate(5.0, robot=arm)

if __name__ == '__main__':
    arm_animation()
