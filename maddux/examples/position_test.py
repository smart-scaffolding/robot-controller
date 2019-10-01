import numpy as np
from maddux.environment import Environment
from maddux.objects import Obstacle, Ball
from maddux.robots import simple_human_arm, inchworm, RobotBehavior, PathPlanner
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

block_size = 3.0


def arm_animation():
    q0 = np.array([0, np.pi/2, -np.pi/2, -np.pi/2])
    base_pos = np.array([0.5, 0., 1.3])

    robot_behavior = RobotBehavior(q0, base_pos)

    fig = plt.figure(figsize=(8, 8))
    ax = Axes3D(fig)

    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])

    robot_behavior.robot.plot(ax, robot_behavior.robot.qs, robot_behavior.robot.links)
    plt.show()

if __name__ == '__main__':
    arm_animation()
