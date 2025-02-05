import rclpy
import rclpy.clock
from rclpy.node import Node
from .grid import Grid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import fcl
import random
import math
import numpy as np
import copy
import numpy.linalg as linalg
from .robot import Robot
from sensor_msgs.msg import JointState

class BoxOb:
    # posArray should be numpy.array. w,l,h are floats
    def __init__(self, w, l, h, posArray, id=0):
        self.w = w
        self.l = l
        self.h = h
        self.posArray = posArray

        self.collGeom = fcl.Box(w, l, h)  # This should be a CollisionGeometry object from fcl
        self.collTf = fcl.Transform(posArray)
        self.collObj = fcl.CollisionObject(self.collGeom, self.collTf)

        self.rviz_marker = buildRvizCube(self, id)  # This should be a Marker object from visualization_msgs.msg

def buildRvizCube(boxOb, i=0):
    """
    Builds and returns a visualization marker representing a Box Obstacle for use in RViz.
    Parameters:
      boxOb (BoxOb): A BoxOb object representing the properties of the object
      i (int): An index used to differentiate between markers
    Returns:
      Marker: A visualization marker representing the BoxOb object
    The BoxOb object should have the following attributes:
    - w (float): Width of the cube.
    - l (float): Length of the cube.
    - h (float): Height of the cube.
    - posArray (list): List containing the x, y, and z coordinates of the cube's position.
    The function creates a Marker object, configures it to represent a cube with specified dimensions,
    color, and position, and returns it.
    """

    # Create the Marker basics
    pointMarker = Marker()
    pointMarker.header.frame_id = 'base_link'
    pointMarker.ns = ''

    pointMarker.id = 10+i
    pointMarker.type = 1 # Cube
    pointMarker.action = 0 # Add

    # Set the scale
    pointMarker.scale.x = boxOb.w
    pointMarker.scale.y = boxOb.l
    pointMarker.scale.z = boxOb.h

    # Set the color
    pointMarker.color.b = 1.0
    pointMarker.color.a = 1.0
    pointMarker.colors.append(pointMarker.color)

    # Set the position
    pointMarker.pose.position.x = boxOb.posArray[0]
    pointMarker.pose.position.y = boxOb.posArray[1]
    pointMarker.pose.position.z = boxOb.posArray[2]

    return pointMarker

def createOb(x, y, z, w, l, h):
    """
    Creates a BoxOb object with specified dimensions and position.
    Parameters:
        x (float): x-coordinate of the box's position.
        y (float): y-coordinate of the box's position.
        z (float): z-coordinate of the box's position.
        w (float): Width of the box.
        l (float): Length of the box.
        h (float): Height of the box.
    Returns:
        BoxOb: A BoxOb object with the specified properties.
    """
    pos = np.array([x, y, z])
    return BoxOb(w, l, h, pos)

def createObRand(id=0):
    """
    Creates a BoxOb object with random dimensions and position within specified workspace bounds.
    Returns:
        BoxOb: A random BoxOb object.
    """
    # Workspace bounds
    xmin = -1.0
    xmax = 1.0
    ymin = -1.0
    ymax = 1.0
    zmin = 0.0
    zmax = 1.0

    # Obstacle size range
    obMin = 0.33
    obMax = 1.5

    # Box params
    l = random.uniform(obMin, obMax)
    w = random.uniform(obMin, obMax)
    h = random.uniform(obMin, obMax)

    # Rejection sampling approach to get an obstacle center
    # within the robot's workspace
    withinWorkspace = False
    while not withinWorkspace:
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        z = random.uniform(zmin, zmax)

        withinWorkspace = math.sqrt(x**2 + y**2 + z**2) < 5.0

    # Make position vector
    pos = np.array([x, y, z])

    # Make BoxOb object
    obj = BoxOb(w,l,h,pos, id=id)

    #print('Exiting createOb')
    return obj

def createNObs(n, id_start=0):
    return [createObRand(id=id_start + i) for i in range(n)]

class ColliderTest(Node):

    def __init__(self):
        super().__init__('collider_test')

        # Create robot object
        # TODO: Students - change the starting configuration as desired to test the collision!
        self.robot = Robot(np.array([-math.pi/6, math.pi/4.0, -math.pi/4]))

        # Create publisher for joint states
        self.pub_joint_state = self.create_publisher(JointState, 'joint_states', 10)

        # Put the configuration into a JointState message and publish it
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()
        j.name = ['joint1', 'joint2', 'joint3']
        j.position = self.robot.q.tolist()
        self.pub_joint_state.publish(j)

        # Create publisher for grid markers
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', qos_profile=10)

        # Create timer to move the grid
        timer_freq = 0.1
        self.timer = self.create_timer(timer_freq, self.move_grid, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

        # Initialize grid and publish it
        # TODO: Students - change the initial grid point and grid size as desired to test the collision.
        self.init_grid_point = Point(x=-5.0, y=-5.0, z=7.0)
        self.grid = Grid(10.0, 10.0, self.init_grid_point.x, self.init_grid_point.y, self.init_grid_point.z)
        self.publisher.publish(self.grid.grid)

        # Initialize grid increment values
        self.grid_inc = Point(x=0.0, y=0.0, z=-0.1)

        # Create "prefab" for collision marking cell
        self.cell_marker = Marker()
        self.cell_marker.type = Marker.CUBE
        self.cell_marker.action = Marker.ADD
        self.cell_marker.header.frame_id = "base_link"
        self.cell_marker.scale.x = 0.1
        self.cell_marker.scale.y = 0.1
        self.cell_marker.scale.z = 0.1
        self.cell_marker.color.r = 1.0
        self.cell_marker.color.g = 0.0
        self.cell_marker.color.b = 0.0
        self.cell_marker.color.a = 1.0
        self.cell_marker_id = 10000

        # z value to stop moving the grid
        self.z_min = -1.0


    def move_grid(self):
        if self.grid.init_point.z > self.z_min:
            cb_start = self.get_clock().now()
            #print("time since last call: %s" % ((cb_start - self.cb_end).nanoseconds / 1e9))
            self.grid.init_point.z += self.grid_inc.z
            draw_start = self.get_clock().now()
            self.grid.draw()
            draw_end = self.get_clock().now()
            pub_start = self.get_clock().now()
            self.publisher.publish(self.grid.grid)
            pub_end = self.get_clock().now()
            coll_start = self.get_clock().now()
            self.run_coll_check()
            coll_end = self.get_clock().now()
            self.cb_end = self.get_clock().now()
            """
            print("draw time: %s" % ((draw_end - draw_start).nanoseconds / 1e9))
            print("pub time: %s" % ((pub_end - pub_start).nanoseconds / 1e9))
            print("coll time: %s" % ((coll_end - coll_start).nanoseconds / 1e9))
            print("cb time: %s" % ((self.cb_end - cb_start).nanoseconds / 1e9))
            """

    def run_coll_check(self):
        # Create a MarkerArray object to hold the generated red grid cell markers
        coll_markers = MarkerArray()

        # For each grid cell, check for collision with each obstacle
        for ob_coll in self.robot.link_colls:
            num_cells = 0

            # For each cell in the grid
            for cell in self.grid.coll_objs:

                request = fcl.CollisionRequest()
                result = fcl.CollisionResult()
                fcl.collide(cell, ob_coll, request, result)

                # If collision, spawn a red marker at the cell
                if result.is_collision:
                    #print("Collision detected between grid cell {} and obstacle {}".format(cell.getTranslation(), robot_link_coll.getTranslation()))

                    # Create a red grid cell marker
                    cell_marker = copy.deepcopy(self.cell_marker)
                    cell_marker.id = self.cell_marker_id
                    cell_marker.pose.position.x = cell.getTranslation()[0]
                    cell_marker.pose.position.y = cell.getTranslation()[1]
                    cell_marker.pose.position.z = cell.getTranslation()[2]
                    self.cell_marker_id += 1
                    coll_markers.markers.append(cell_marker)

                    num_cells += 1

        # Publish the list of red grid cell markers (if any)
        self.publisher.publish(coll_markers)

# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = ColliderTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()