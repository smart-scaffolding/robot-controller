import numpy as np
from maddux.environment import Environment
from maddux.objects import Ball, Target
from maddux.robots import simple_human_arm, inchworm, utils

def arm_animation():
    """Animate the arm moving to touch a ball"""

    # Declare a human arm

    # q0 = np.array([0.5, 0.2, 0, 0.5, 1.5])
    # arm = simple_human_arm(2.0, 2.0, q0, np.array([2.0, 2.0, 0.0]))

    q0 = np.array([0.7, 1.9, 1.1, 0, 1.5, 0])
    base_pos = np.array([0., 0., 0.])
    seg_lens = np.array([2.0, 4.0, 4.0, 2.0, 6.0])

    arm = inchworm(seg_lens, q0, base_pos)

    q0_2 = np.array([0.7, 1.9, 1.1, 0, 1.5])
    base_pos_2 = np.array([10., 10., 0.])
    seg_lens_2 = np.array([2.0, 4.0, 4.0, 2.0])

    # arm2 = inchworm(seg_lens_2, q0_2, base_pos_2)
    
    # Create a ball as our target
    # ball = Ball(np.array([6, 0., 0.]), 0.15, target=True)
    # ball_2 = Ball(np.array([10, 13., 0.]), 0.15, target=True)

    target = Target(position=[-2, 0., 4.], radius=0.5)
    target2 = Target(position=[6, 1., 2.], radius=0.5)


    # Create our environment
    env = Environment([20.0, 20.0, 5.0], static_objects=[target],
                      robot=[arm])
    # np.flip(arm.links, 0)
    # print(arm.links)
    # arm.links = np.flip(arm.links, 0)
    # print(arm.links)
    #
    # print(arm.end_effector_position())
    # Run inverse kinematics to find a joint config that lets arm touch ball
    joints = arm.get_current_joint_config()
    joints = arm.ikine(target.position, printVals=False, q=joints)

    print(joints)
    # print(arm.links)
    # joints = arm.get_current_joint_config()
    # arm.update_link_angle(4, -1.5)

    
    # Animate
    env.animate(5.0, robot=arm)
    new_pos = arm.end_effector_position()
    joints = arm.get_current_joint_config()
    print "New Pos: {}".format(new_pos)

    # arm.links = np.flip(arm.links, 0)
    arm = inchworm(seg_lens, np.flip(joints, 0) + 3.14, new_pos)
    arm.ikine(target2.position, printVals=True, q=np.flip(joints, 0) + 3.14)

    env.animate(5.0, robot=arm)
if __name__ == '__main__':
    arm_animation()
