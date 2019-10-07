import numpy as np
from maddux.environment import Environment
from maddux.objects import Obstacle, Ball
from maddux.robots import simple_human_arm, inchworm, RobotBehavior, PathPlanner, inchworm_vertical
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import time
block_size = 3.0


def arm_animation():
    # ser = serial.Serial(port='/dev/cu.usbmodem14201', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
    #                     bytesize=serial.EIGHTBITS, timeout=3.0)
    #
    # time.sleep(2)


    #joint[1] = -np.pi/2 for horizontal position
    #joint[1] = np.pi for vertical position
    q0 = np.array([0, np.pi, 0.7, 0.8, 0.9, 0])
    # q0 = np.array([3.14143349, 1.77881778, 2.11949991, 0.20802146])

    base_pos = np.array([3., 5., 1.])

    robot = inchworm_vertical(q0, base_pos)

    fig = plt.figure(figsize=(8, 8))
    ax = Axes3D(fig)

    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])

    # q = [0., np.pi/2, 0, 0.]

    # qTemp = np.array([q[0] + np.pi, q[1] + (np.pi/2), q[2] + np.pi, q[3] + np.pi])
    # qTemp = qTemp * 180.0 / np.pi # convert to degrees
    # targetAngles = str(int(qTemp[1])).zfill(3) + str(int(qTemp[2])).zfill(3) + str(int(qTemp[3])).zfill(3)

    # while True:
    #     ser.write(targetAngles)
    #     time.sleep(0.1)
    #     data = ser.readline()[:-2]
    #     if data:
    #         print data
    #         break
    robot.plot(ax, robot.qs, robot.links)
    plt.show()

if __name__ == '__main__':
    arm_animation()
