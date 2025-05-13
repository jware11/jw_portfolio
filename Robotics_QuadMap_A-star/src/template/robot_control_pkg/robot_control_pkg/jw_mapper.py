import math
import tf2_ros
import rclpy
from time import sleep
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
import numpy as np
from .mapping import QuadMap, QuadMapNode


class Mapper(Node):

	def __init__(self, offset=0):
		super().__init__('Mapper')
		self.scan_subscriber = self.create_subscription(
			LaserScan,
			'/en613/scan',
			self.scan_callback,
			10)

		self.odom_subscriber = self.create_subscription(
			Odometry,
			'/en613/odom',
			self.odom_callback,
			10)
		
		# self.state_pub = self.create_publisher(Pose, '/en613/state_est', 10)
		self.grid_pub = self.create_publisher(OccupancyGrid, '/en613/grid', 10)
		# self.grid_pub = self.create_publisher(OccupancyGrid, '/OccupancyGrid', 10)

		self._tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self._tf_buffer,self)

		self._to_frame = 'odom'
		self._from_frame = 'chassis'

		#init parameters
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.pos = ([self.x, self.y])		
		self.state = ([self.x, self.y, self.theta])
		# self.odom_update = False

		#init timer
		self.dt = 1/30
		self.timer = self.create_timer(self.dt, self.timer_callback)

		#init sizing
		quad_depth = 6
		quad_size = 10 #meters
		quad_size_cells = int(np.power(2, quad_depth))
		quad_res = quad_size / quad_size_cells

		occ_size = quad_size
		occ_h = quad_size_cells
		occ_w = quad_size_cells
		occ_res = quad_res

		#init map
		origin = np.array([offset, offset])
		self.map = OccupancyGrid()
		self.map.header.frame_id = 'odom'
		self.map.info.width = occ_w
		self.map.info.height = occ_h
		self.map.info.resolution = occ_res
		self.map.info.origin.position.x = -occ_w // 2 * occ_res + origin[0]
		self.map.info.origin.position.y = -occ_h // 2 * occ_res + origin[1]
		self.map.data = [-1 for i in range(occ_h * occ_w)]
		self.quadmap = QuadMap(max_depth = quad_depth, size=quad_size, origin=origin)
		self.publish_count = 0


	def scan_callback(self, msg):
		#starting point
		# print('Starting Scan!')
		angle = self.theta + msg.angle_min 
		inc = msg.angle_increment
		endpoints = []
		for i in msg.ranges: #create endpoint matrix
			#i = magnitude of range
			xp = self.x + i*np.cos(angle)
			yp = self.y + i*np.sin(angle)
			endpoints.append([xp,yp])
			angle += inc

		for row in endpoints: 
			self.quadmap.ray_update(self.pos, row)

		self.pub_grid()

	def pub_grid(self): 
		# Publish Occupancy Grid
		grid = self.quadmap.to_occupancygrid2()
		self.quadmap.plot()
		self.map.data = grid
		# grid = np.flip(grid, axis=0)
		# grid = np.flatten().astype(int)
		# grid = np.reshape(grid, (grid.shape[0]*grid.shape[1],)).astype(int)
		# grid = [int(i) for i in grid]
		# grid = np.int8(grid)
		# grid = set(grid.flatten())
		# grid = set(np.ravel(grid))
		# grid = set(np.int8(grid).flatten()) #change data type, flatten, convert to list
		# msg = OccupancyGrid()
		# msg.header.frame_id = 'odom'
		# msg.header.seq = 1
		# msg.header.time
		# msg.data = grid

		# msg.data = grid.flatten()
		# msg.data = np.ravel(grid)
		# print('Publishing grid!')
		# self.grid_pub.publish(msg)

		# pass

	def quaternion_to_euler(self, Q):
		# elements of the Quaternion
		(q0, q1, q2, q3) = (Q[0], Q[1], Q[2], Q[3])

		alpha = np.arctan2(2*(q0*q1 + q2*q3), 1-2*(q1**2 + q2**2))
		beta = np.arcsin(2*(q0*q2 - q1*q3))
		gamma = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2**2 + q3**2))

		arr = np.array([alpha, beta, gamma])

		return arr

	def euler_from_quaternion(self, Q):
	    #adapted from solution
	    (x, y, z, w) = (Q[0], Q[1], Q[2], Q[3])

	    sinr_cosp = 2 * (w * x + y * z)
	    cosr_cosp = 1 - 2 * (x * x + y * y)
	    roll = np.arctan2(sinr_cosp, cosr_cosp)

	    sinp = 2 * (w * y - z * x)
	    pitch = np.arcsin(sinp)

	    siny_cosp = 2 * (w * z + x * y)
	    cosy_cosp = 1 - 2 * (y * y + z * z)
	    yaw = np.arctan2(siny_cosp, cosy_cosp)

	    return roll, pitch, yaw

	def odom_callback(self, msg):
		pose = msg.pose.pose
		self.x = pose.position.x
		self.y = pose.position.y
		self.pos = ([self.x, self.y])

		qx = pose.orientation.x
		qy = pose.orientation.y
		qz = pose.orientation.z
		qw = pose.orientation.w

		Q = ([qx, qy, qz, qw])
		a, b, g = self.quaternion_to_euler(Q)
		# a, b, g = self.euler_from_quaternion(Q)
		self.theta = a #should this be g? 

	def timer_callback(self): 
		self.publish_count += 1
		self.grid_pub.publish(self.map)


def main(args=None):
	rclpy.init(args=args)

	mapper = Mapper()

	rclpy.spin(mapper)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	mapper.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
