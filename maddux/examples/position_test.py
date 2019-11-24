import numpy as np
from maddux.environment import Environment
from maddux.objects import Obstacle, Ball
from maddux.robots import simple_human_arm, inchworm, RobotBehavior, PathPlanner
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import time
block_size = 3.0


def arm_animation():
    ser = serial.Serial(port='/dev/cu.usbmodem14201', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS, timeout=3.0)

    time.sleep(2)

    q0 = np.array([0, np.pi/2, 0, 0])
    base_pos = np.array([0., 0., 1.])

    robot_behavior = RobotBehavior(q0, base_pos)

    fig = plt.figure(figsize=(8, 8))
    ax = Axes3D(fig)

    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])

    angles = []
    q1 = [0., 0, 0, 0]
    q2 = [0, np.pi/2, 0, 0] #180 degrees
    q3 = [0, np.pi/3, 0, 0] #90 degrees
    # qFow = np.array([0, 65, 36, 60]) #90 degrees
    qFow = np.array([0, 65, -124, -30]) # supposed to be:
    qFow = qFow / 180.0 * np.pi
    qBack = np.array([0, 49, -91, -48]) #90 degrees
    qBack = qBack / 180.0 * np.pi

    # angles.append(qBack)
    # angles.append(qFow)

    # angles.append(q3)
    # angles.append(q2)
    # angles.append(q1)

    angles.append(robot_behavior.initial_move([4, 0, 2]))

    while len(angles) > 0:
        curr_angle = angles.pop()
        print "Current Angle: {}".format(curr_angle)
        targetAngles = send_angles(curr_angle)
        ser.write(targetAngles)
        # time.sleep(5.0)
        # data = ser.readline()[:-2]
        # if data:
        #     print data
        #     break
        #
        # time.sleep(10.0)

        robot_behavior.robot.update_angles(np.asarray(curr_angle))
        robot_behavior.robot.plot(ax, np.asarray(curr_angle), robot_behavior.robot.links)
        plt.show()
        time.sleep(2.0)

def send_angles(q):
    qTemp = np.array([q[0] + np.pi, q[1] + (np.pi / 2), q[2] + np.pi, q[3] + np.pi])
    qTemp = qTemp * 180.0 / np.pi  # convert to degrees
    print "Final Angles: {}".format(qTemp[1:])
    targetAngles = str(int(qTemp[1])).zfill(3) + str(int(qTemp[2])).zfill(3) + str(int(qTemp[3])).zfill(3)
    return targetAngles


if __name__ == '__main__':
    arm_animation()
