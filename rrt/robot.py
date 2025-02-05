from rrt.tf_rmp import *
import fcl
import numpy as np

class  Robot:
    def __init__(self, q):
        """ Create a Robot object that will hold configuration and collision objects for a robot.

        Args:
            q (numpy.array): array representing the robot's configuration.
        """

        # Link lengths and radius
        self.link_lengths = [3.0, 3.0, 2.0]
        self.link_radius = 0.1

        # Current configuration
        self.q = q

        # Create collision objects for the robot's links
        self.link_colls = []
        self._setup_coll()

    # TODO: Implement this method
    def _setup_coll(self):
        """ Create FCL collision objects for the robot's links.
        """
        # Basis for how I implemented this was:
        # Forumla from textbook. Chapter 4
        # x = L1 cos θ1 + L2 cos(θ1 + θ2) + L3 cos(θ1 + θ2 + θ3),
        # y = L1 sin θ1 + L2 sin(θ1 + θ2) + L3 sin(θ1 + θ2 + θ3),
        # φ = θ1 + θ2 + θ3.

        # link 1 geometry
        link1_geom = fcl.Cylinder(self.link_radius, self.link_lengths[0])
        tf1_mat = np.matrix(transform(0, 0, self.link_lengths[0] / 2, 0, 0, self.q[0]))
        tf1 = fcl.Transform(tf1_mat[0:3, 0:3], tf1_mat[0:3, 3])
        self.link_colls.append(fcl.CollisionObject(link1_geom, tf1))

        # link 2 geometry
        link2_geom = fcl.Cylinder(self.link_radius, self.link_lengths[1])
        tf2_transform = transform(np.cos(self.q[1]) * self.link_lengths[0] / 2, 0, 
                                  np.sin(self.q[1]) * self.link_lengths[0] / 2 + self.link_lengths[0] / 2, 0, self.q[1], 0)
        tf2_mat = np.matmul(tf1_mat, tf2_transform)
        tf2 = fcl.Transform(tf2_mat[0:3, 0:3], tf2_mat[0:3, 3])
        self.link_colls.append(fcl.CollisionObject(link2_geom, tf2))

        # link 3 geometry
        link3_geom = fcl.Cylinder(self.link_radius, self.link_lengths[2])
        tf3_transform = transform(np.cos(self.q[1]) * self.link_lengths[1] + np.cos(self.q[1] + self.q[2]) * self.link_lengths[2] / 2, 0,
                                  np.sin(self.q[1]) * self.link_lengths[1] + self.link_lengths[1] / 2 + np.sin(self.q[1] + self.q[2]) * self.link_lengths[2] / 2, 0, np.pi/2 - (self.q[1] + self.q[2]), 0)
        tf3_mat = np.matmul(tf1_mat, tf3_transform)
        tf3 = fcl.Transform(tf3_mat[0:3, 0:3], tf3_mat[0:3, 3])
        self.link_colls.append(fcl.CollisionObject(link3_geom, tf3))
        