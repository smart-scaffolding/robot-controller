import numpy as np
from maddux.robots.predefined_robots import simple_human_arm, inchworm
from maddux.objects import Ball, Target, Obstacle
from maddux.environment import Environment


def tutorial():
    """Code from our tutorial on the documentation"""
    
    # Create an arm with a specific config and base position
    # q0 = np.array([0.5, 0.2, 0, 0.5, 0, 0, 0])
    # q0 = np.array([0, 0.4, -0.4, -1.5, 0])
    q0 = np.array([-1.5, 1.9, 1.1, 1.1, 1.5])
    base_pos = np.array([2.0, 2.0, 0.0])
    seg_lens= np.array([2.0, 4.0, 4.0, 2.0])
    
    # And link segments of length 2.0
    # arm = simple_human_arm(2.0, 2.0, q0, base_pos)
    arm = inchworm(seg_lens, q0, base_pos)
    # We then create a ball, target, and obstacle
    ball = Ball(position=[1.0, 0.0, 2.0], radius=0.15)
    target = Target(position=[5.0, 8.0, 2.0], radius=0.5)
    # obstacle = Obstacle([4, 4, 0], [5, 5, 2])

    # And use these to create an environment with dimensions 10x10x10
    env = Environment(dimensions=[10, 10, 10],
                      dynamic_objects=[ball],
                      static_objects=[target],
                      robot=[arm])
    
    # arm.ikine(target.position)
    # env.animate(3.0)
    # arm.save_path("tutorial_path")
    env.plot()

if __name__ == '__main__':
    tutorial()
