from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import fcl
from builtin_interfaces.msg import Duration

# for timing out grid after grid test completes
marker_life = Duration()
marker_life.sec = 1
marker_life.nanosec = 0

class Grid:
    """
    Represents a grid of cells for visualization and collision detection.

    Attributes:
        width (float): The width of the grid.
        length (float): The length of the grid.
        grid_cell (Marker): A Marker object representing each grid cell.
        grid (MarkerArray): A collection of markers representing the entire grid.
        id (int): A unique identifier for grid markers.
        init_point (Point): The initial position of the grid.
        coll_objs (list): A list of FCL collision objects for each grid cell.
    """
    def __init__(self, width, length, x = 0.0, y = 0.0, z = 0.0):
        """
        Initializes a Grid instance with specified dimensions and initial position.

        Parameters:
            width (float): The width of the grid.
            length (float): The length of the grid.
            x (float, optional): The x-coordinate of the grid's initial position, default is 0.0.
            y (float, optional): The y-coordinate of the grid's initial position, default is 0.0.
            z (float, optional): The z-coordinate of the grid's initial position, default is 0.0.
        """
        self.width = width
        self.length = length

        self.grid_cell = Marker()
        self.grid_cell.type = Marker.CUBE
        self.grid_cell.action = Marker.ADD
        self.grid_cell.header.frame_id = "base_link"
        self.grid_cell.scale.x = 0.1
        self.grid_cell.scale.y = 0.1
        self.grid_cell.scale.z = 0.1
        self.grid_cell.color.r = 0.0
        self.grid_cell.color.g = 1.0
        self.grid_cell.color.b = 0.0
        self.grid_cell.color.a = 1.0
        self.grid_cell.lifetime = marker_life

        self.grid = MarkerArray()

        self.id = 0

        # Set up the initial point for the grid
        self.init_point = Point(x=x, y=y, z=z)

        # Initial draw
        # draw will also create FCL collision objects for each grid cell
        self.coll_objs = []
        self.draw()

    def draw(self):
        """
        Draws the grid by generating markers and FCL collision objects for each cell.
        Resets the marker ID and collision object list before creating the grid.
        """
        # Reset markers and id value
        self.id = 0
        self.grid.markers = []
        self.coll_objs = []

        # Get number of grid cells in length and width
        num_len = int(self.length / self.grid_cell.scale.y)
        num_wid = int(self.width / self.grid_cell.scale.x)

        # Same geometry for each cell
        coll_geom = fcl.Box(self.grid_cell.scale.x, self.grid_cell.scale.y, self.grid_cell.scale.z)

        z = self.init_point.z
        # Loop through the grid to generate markers
        for i in range(num_len):
            for j in range(num_wid):
                x = self.init_point.x + (j * 0.1)
                y = self.init_point.y + (i * 0.1)
                self.id += 1
                coll_tf = fcl.Transform([x, y, z])
                coll_obj = fcl.CollisionObject(coll_geom, coll_tf)
                self.coll_objs.append(coll_obj)
        self.create_grid_lines()


    def create_grid_lines(self):
        """
        Creates grid lines using markers and appends them to the grid marker array.
        The lines are drawn both horizontally and vertically to delineate the grid.
        """
        # Create a new Marker for grid lines
        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.header.frame_id = "base_link"
        line_marker.scale.x = 0.02  # Line width
        line_marker.lifetime = marker_life

        # Set line color (black)
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        # Starting position
        x_s = self.init_point.x
        y_s = self.init_point.y
        z_s = self.init_point.z

        # Get number of grid cells in length and width
        num_len = int(self.length / self.grid_cell.scale.y)
        num_wid = int(self.width / self.grid_cell.scale.x)

        scale_offset = [-self.grid_cell.scale.x/2, -self.grid_cell.scale.y/2, self.grid_cell.scale.z/2]

        # Length lines
        for i in range(num_len):
            p1_top = Point(x=x_s + scale_offset[0], y=y_s + (i * 0.1) + scale_offset[1], z=z_s + scale_offset[2])
            p2_top = Point(x=x_s + (num_wid * self.grid_cell.scale.x) + scale_offset[0], y=y_s + (i  * 0.1) + scale_offset[1], z=z_s +  + scale_offset[2])
            #print("line points: {}, {}".format(p1_top, p2_top))
            line_marker.points.append(p1_top)
            line_marker.points.append(p2_top)

            # Vertical line bottom
            p1_bot = Point(x=x_s + scale_offset[0], y=y_s + (i * 0.1) + scale_offset[1], z=z_s - scale_offset[2])
            p2_bot = Point(x=x_s + (num_wid * self.grid_cell.scale.x) + scale_offset[0], y=y_s + (i  * 0.1) + scale_offset[1], z=z_s -  + scale_offset[2])
            #print("line points: {}, {}".format(p1_bot, p2_bot))
            line_marker.points.append(p1_bot)
            line_marker.points.append(p2_bot)

            # Side lines
            p1_side = Point(x=x_s + scale_offset[0], y=y_s + (i * 0.1) + scale_offset[1], z=z_s + scale_offset[2])
            p2_side = Point(x=x_s + scale_offset[0], y=y_s + (i * 0.1) + scale_offset[1], z=z_s - scale_offset[2])
            #print("side line points: {}, {}".format(p1_side, p2_side))
            line_marker.points.append(p1_side)
            line_marker.points.append(p2_side)

            p1_side = Point(x=x_s + (num_wid * self.grid_cell.scale.x) + scale_offset[0], y=y_s + (i * 0.1) + scale_offset[1], z=z_s + scale_offset[2])
            p2_side = Point(x=x_s + (num_wid * self.grid_cell.scale.x) + scale_offset[0], y=y_s + (i * 0.1) + scale_offset[1], z=z_s - scale_offset[2])
            #print("side line points: {}, {}".format(p1_side, p2_side))
            line_marker.points.append(p1_side)
            line_marker.points.append(p2_side)

        # Width lines
        for i in range(num_wid):
            # Horizontal line top
            p1_top = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1], z=z_s + scale_offset[2])
            p2_top = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1] + (num_len * self.grid_cell.scale.y), z=z_s +  + scale_offset[2])
            #print("line points: {}, {}".format(p1_top, p2_top))
            line_marker.points.append(p1_top)
            line_marker.points.append(p2_top)

            # Horizontal line bottom
            p1_bot = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1], z=z_s - scale_offset[2])
            p2_bot = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1] + (num_len * self.grid_cell.scale.y), z=z_s -  + scale_offset[2])
            #print("line points: {}, {}".format(p1_bot, p2_bot))
            line_marker.points.append(p1_bot)
            line_marker.points.append(p2_bot)

            # Side lines
            p1_side = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1], z=z_s + scale_offset[2])
            p2_side = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1], z=z_s - scale_offset[2])
            #print("side line points: {}, {}".format(p1_side, p2_side))
            line_marker.points.append(p1_side)
            line_marker.points.append(p2_side)

            p1_side = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1] + (num_len * self.grid_cell.scale.y), z=z_s + scale_offset[2])
            p2_side = Point(x=x_s + (i * 0.1) + scale_offset[0], y=y_s + scale_offset[1] + (num_len * self.grid_cell.scale.y), z=z_s - scale_offset[2])
            line_marker.points.append(p1_side)
            line_marker.points.append(p2_side)

        # Assign a unique ID to the grid lines marker and append to MarkerArray
        line_marker.id = self.id
        self.grid.markers.append(line_marker)
        self.id += 1