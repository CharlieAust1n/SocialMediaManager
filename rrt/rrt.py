import fcl
import time
import random
import numpy as np
import networkx as nx
from math import sqrt
from visualization_msgs.msg import Marker
from rrt.collider_test import BoxOb, buildRvizCube
from rrt.robot import Robot

# You may change this value to simplify running the algorithm while developing.
N_OBSTACLES = 10

# TODO: Implement this function
def interpolate_configurations(config1, config2, num_steps):
    """
    Interpolates between two configurations using straight-line interpolation
    q_i = q_s + i * (q_g - q_s)

    :param config1: Starting joint angles (list or numpy array).
    :param config2: Ending joint angles (list or numpy array).
    :param num_steps: Number of intermediate configurations to generate.
    :return: List of interpolated configurations (as numpy arrays).
    """
    return [config1 + i * (config2 - config1) / (num_steps - 1) for i in range(num_steps)]

class RRTPlanner:
    def __init__(self, start, goal, node, max_iterations=1000, step_size=0.5):
        # Store the node instance so that we can publish with this object
        self.node = node

        # Make a publisher for markers
        self.publisher = self.node.create_publisher(Marker, 'visualization_marker', 10)

        # Start/goal states and parameters
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.max_iterations = max_iterations
        self.step_size = step_size

        # For ensuring randomized fcl boxes do not collide immediately with start/goal states
        self.r_config_start = Robot(start)
        self.r_config_goal = Robot(start)

        # Store the FCL objects to check against robot collision
        self.obstacles = []

        # Randomly generates N_OBSTACLES number of randomly-shaped boxes near the workspace of the robot
        self.gen_obs(N_OBSTACLES)

        # RRT tree structure stored as a networkx.Graph
        self.tree = nx.Graph()
        self.tree.add_node(tuple(self.start))

    def clear_all_markers(self):
        """ Clear all markers in RViz.
        """
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(marker)
        self.node.get_logger().info('Cleared all markers')

    def gen_obs(self, n):
        """ Generate n random obstacles in the workspace.

        Args:
            n (int): Number of obstacles to generate.
        """
        # Robot collision for start/goal states
        coll_init = self.r_config_start.link_colls
        coll_goal = self.r_config_goal.link_colls

        while True:
            self.clear_all_markers()

            # Generate valid FCL obstacles
            for _ in range(n):
                boxOb = self.createObRand()
                marker = buildRvizCube(boxOb, len(self.obstacles))

                # Wait until the publisher is ready
                while self.publisher.get_subscription_count() < 1:
                    self.node.get_logger().info('Waiting for RViz to subscribe...')
                    time.sleep(0.2)

                # Publish the marker and log the event
                self.publisher.publish(marker)
                self.obstacles.append(boxOb)

                # Add a small delay to ensure message delivery
                time.sleep(0.1)

            # Check for collisions with start/goal states
            if not self.init_collision_check(coll_init) and not self.init_collision_check(coll_goal):
                break
            else:
                self.obstacles.clear()

    def createObRand(self):
        """
        Creates a BoxOb object with random dimensions and position within specified workspace bounds.
        Returns:
            BoxOb: A random BoxOb object.
        """
        # Workspace bounds
        xmin = -3.0
        xmax = 3.0
        ymin = -3.0
        ymax = 3.0
        zmin = 0.0
        zmax = 6.0

        # Obstacle size range
        obMin = 0.33
        obMax = 1.5

        # Box dimensions
        random.seed(time.time())
        l = random.uniform(obMin, obMax)
        w = random.uniform(obMin, obMax)
        h = random.uniform(obMin, obMax)

        # Random position sampling
        withinWorkspace = False
        while not withinWorkspace:
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            z = random.uniform(zmin, zmax)
            withinWorkspace = sqrt(x**2 + y**2 + z**2) < 5.0

        pos = np.array([x, y, z])

        # Create BoxOb
        return BoxOb(w, l, h, pos)

    def init_collision_check(self, robot_objects):
        """ Check for collision between the robot and all FCL objects.

        Args:
            robot_objects (list): List of FCL objects representing the robot.

        Returns:
            bool: True if a collision is detected, False otherwise.
        """
        colliding_obstacles = []

        for robot_obj in robot_objects:
            for i, obstacle in enumerate(self.obstacles):
                request = fcl.CollisionRequest()
                result = fcl.CollisionResult()
                collision = fcl.collide(robot_obj, obstacle.collObj, request, result)

                if collision:
                    self.node.get_logger().info(f'Collision detected with object ID: {obstacle}. Restarting...')
                    colliding_obstacles.append((obstacle, i))

        # Show colliding obstacles in red, then try again until success
        for obstacle, i in colliding_obstacles:
            marker = buildRvizCube(obstacle, i)
            self.publisher.publish(marker)

        if colliding_obstacles:
            return True
        else:
            return False

    # TODO: Implement this method
    def coll_check(self, q1, q2, num_steps=100):
        """ Check for collision between two configurations.

        Args:
            q1 (list): Starting joint angles.
            q2 (list): Ending joint angles.
            num_steps (int): Number of interpolated configurations to generate.

        Returns:
            bool: True if a collision is detected, False otherwise
        """
        # Get all interpolated configurations between config1 and config2
        interpolated_configs = interpolate_configurations(np.array(q1), np.array(q2), num_steps)

        """
        Checking collision with FCL can be done in the following way:
            request = fcl.CollisionRequest()
            result = fcl.CollisionResult()
            collision = fcl.collide(obj1, obj2, request, result)

            if collision: (the return of the collide function is the number of collisions found)
                return True
        """
        for config in interpolated_configs:
            robot = Robot(config)
            for link_collision in robot.link_colls:
                for obstacle in self.obstacles:
                    request = fcl.CollisionRequest()
                    result = fcl.CollisionResult()
                    collision = fcl.collide(link_collision, obstacle.collObj, request, result)
                    if collision:
                        return True
        return False

    # TODO: Implement this function
    def run_planner(self):
        """ Runs the RRT planner to find a path from the start to the goal.

        Returns:
            bool: True if a path was found, False otherwise.
            networkx.Graph: The final RRT tree.
        """
        for iteration in range(self.max_iterations):
            '''random_sample = self.random_configuration()
            
            if self.distance(random_sample, self.goal) <= self.step_size:
                if not self.coll_check(random_sample, self.goal):
                    self.tree.add_node(tuple(random_sample))
                    self.tree.add_edge(tuple(self.start), tuple(random_sample))
                    self.tree.add_node(tuple(self.goal))
                    self.tree.add_edge(tuple(random_sample), tuple(self.goal))
                    return True, self.tree

            nearest_sample = self.nearest(random_sample)
            new_point = self.step_towards(nearest_sample, random_sample)
            if not self.coll_check(nearest_sample, new_point):
                self.tree.add_node(tuple(new_point))
                self.tree.add_edge(tuple(nearest_sample), tuple(new_point))'''
            random_sample = self.random_configuration()
            nearest_sample = self.nearest(random_sample)
            new_point = self.step_towards(nearest_sample, random_sample)
            if not self.coll_check(nearest_sample, new_point):
                self.tree.add_node(tuple(new_point))
                self.tree.add_edge(tuple(nearest_sample), tuple(new_point))
                if not self.distance(new_point, self.goal) <= self.step_size:
                    if not self.coll_check(new_point, self.goal):
                        self.tree.add_node(tuple(self.goal))
                        self.tree.add_edge(tuple(new_point), tuple(self.goal))
                        return True, self.tree

        return False, self.tree
        

    # TODO: Implement this function
    def random_configuration(self):
        """ Generate a random configuration within the joint limits.
        """
        return np.random.uniform(-np.pi, np.pi, 3)

    # TODO: Implement this method
    def nearest(self, q):
        """ Find the nearest node in the tree to the random point.
        This is done by finding the node with the minimum distance to configuration.

        You may use the distance method defined in this class to calculate the distance between two 
        configurations, or you may create your own distance method.

        Args:
            q (np.array): Sampled configuration to add to the tree

        Returns:
            np.array: Nearest node in the tree to the random point
        """

        min_distance = float('inf')
        nearest_node = None
        for node in self.tree.nodes:
            current_distance = self.distance(np.array(node), q)
            if min_distance > current_distance:
                min_distance = current_distance
                nearest_node = node
        return np.array(nearest_node)
        


    def step_towards(self, nearest_point, q):
        """ Step from the nearest point in the tree to q towards the configuration q.

        This is needed because the distance between q and its nearest point in the tree
        might be larger than the step size. If so, we need to move towards q only by the step size.

        Args:
            nearest_point (np.array): Nearest point in the tree to q
            random_point (np.array): Randomly sampled configuration

        Returns:
            np.array: New point in the direction of q from the nearest point
        """
        direction = q - nearest_point
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = (direction / distance) * self.step_size
        return nearest_point + direction

    def distance(self, point1, point2):
        """ Calculate the Euclidean distance between two points.

        Args:
            point1 (np.array): point 1
            point2 (np.array): point 2

        Returns:
            float: Euclidean distance between the two points
        """
        return np.linalg.norm(point1 - point2)

    def print_tree(self):
        """ Print the tree nodes for debugging purposes.
        """
        t = [list(node) for node in self.tree.nodes()]
        print(t)

