"""
A ball object to throw.
"""
import numpy as np
from throwable import ThrowableObject
from maddux.plot import plot_sphere
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class Ball(ThrowableObject):

    def __init__(self, p2, position, radius, target=False):
        """Ball object that can move, have a velocity, and hit objects

        :param position: The position (x,y,z) of the center of the ball
        :type position: numpy.ndarray

        :param: radius: The radius of the ball
        :type radius: int

        :rtype: None
        """
        self.pt2 = p2
        self.radius = radius
        ThrowableObject.__init__(self, position, target)

    def get_paths(self):
        """Returns the paths for each of the surfaces of the
        rectangle for plotting.

        :returns (bottom, top, front, back, left, right)
        :rtype: list of 6 4x3 numpy.ndarrays
        """
        [x1, y1, z1] = self.position
        [x2, y2, z2] = self.pt2
        [x2, y2, z2] = [x2 + x1, y2 + y1, z2 + z1]

        pt1 = [x1, y1, z1]
        pt2 = [x1, y1, z2]
        pt3 = [x1, y2, z1]
        pt4 = [x1, y2, z2]
        pt5 = [x2, y1, z1]
        pt6 = [x2, y1, z2]
        pt7 = [x2, y2, z1]
        pt8 = [x2, y2, z2]

        bottom = [pt1, pt3, pt7, pt5]
        top = [pt2, pt4, pt8, pt6]
        front = [pt1, pt2, pt6, pt5]
        back = [pt3, pt4, pt8, pt7]
        left = [pt1, pt2, pt4, pt3]
        right = [pt5, pt6, pt8, pt7]
        paths = [bottom, top, front, back, left, right]
        return paths

    def plot(self, ax):
        """Plots the ball at its current location.

        :param ax: Figure to plot on.
        :type ax: matplotlib.axes

        :returns: Matplotlib figure
        :rtype: matplotlib.axes
        """
        paths = self.get_paths()
        rectangle = Poly3DCollection(paths, facecolors="red")
        return ax.add_collection3d(rectangle)
        # return plot_sphere(self.position, self.radius, ax)
