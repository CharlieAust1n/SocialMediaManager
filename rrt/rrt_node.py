from .rrt import RRTPlanner
import numpy as np
import rclpy
from rmp_util.msg import RMPPath
from rclpy.node import Node
from sensor_msgs.msg import JointState
import networkx as nx
import math

# RRT parameters
RRT_ITERATIONS = 1000
RRT_STEP_SIZE = 2.0

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt')
        self.publisher_ = self.create_publisher(RMPPath, 'path', 10)

        # Define initial and goal states in radians
        # Redo the comments in the lines below to use
        # predefined start and goal states instead of random ones
        #self.start_state = np.array([0.0, 0.0, 0.0])
        #self.goal_state = np.array([-math.pi/4, math.pi/4, -math.pi/4])
        self.start_q = np.random.uniform(-math.pi, math.pi, 3)
        self.goal_q = np.random.uniform(-math.pi, math.pi, 3)

        # Initialize the RRT planner
        self.planner = RRTPlanner(self.start_q, self.goal_q, self, RRT_ITERATIONS, RRT_STEP_SIZE)

        # Don't necessarily need to return the tree since we have access to self.planner.tree
        path_found, tree = self.planner.run_planner()

        if path_found:
            path = nx.shortest_path(tree, source=tuple(self.start_q), target=tuple(self.goal_q))
            self.get_logger().info(f"Path found:")

            js_sequence = []

            # Convert points in path to JointState
            for p in path:
                print(p)
                js = JointState()
                js.name = ["j1", "j2", "j3"]
                js.position = [float(p[0]), float(p[1]), float(p[2])]
                js_sequence.append(js)

            # Send as RMPPath
            msg = RMPPath()
            msg.qs = js_sequence
            self.publisher_.publish(msg)

        else:
            self.get_logger().info("No path found.")


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
