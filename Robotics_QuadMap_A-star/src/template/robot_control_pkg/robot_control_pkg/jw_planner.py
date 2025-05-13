import math
import tf2_ros
import rclpy
from time import sleep
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry, OccupancyGrid #added
import numpy as np
from .a_star_planner import A_Star, GridNode
from tf2_ros import TransformBroadcaster, TransformStamped

class Planner(Node):

	def __init__(self):
		super().__init__('Planner')

		self.odom_subscriber = self.create_subscription(
			Odometry,
			'/en613/odom',
			self.odom_callback,
			10)

		self.goal_subscriber = self.create_subscription(
			PoseArray,
			'/en613/goals',
			self.goal_callback,
			10)

		self.map_subscriber = self.create_subscription(
			OccupancyGrid, 
			'/en613/grid',
			self.map_callback, 
			10)

		#pub next interim goal?
		self.publisher_ = self.create_publisher(Pose, '/en613/target', 10)

		#init
		self.path = []
		self.goals = []
		self.gotgoal = False
		self.cur_goal_i = 0 #current target
		self.num_goals = 3
		self.map = []

		#init position parameters
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.pos = ([self.x, self.y])
		self.state = ([self.x, self.y, self.theta])
		self.tol = 0.5
		# self.odom_update = False

		#create a_star
		self.a_star = A_Star
		self.map_size = 20
		
		#Timer
		pub_freq = 30
		self.dt = 1.0 / pub_freq
		self.timer = self.create_timer(self.dt, self.timer_callback)

		self._tf_buffer = tf2_ros.Buffer()
		# self.listener = tf2_ros.TransformListener(self._tf_buffer,self)
		# self.vel_pub = self.create_publisher(Twist, 'cmd_vel',qos_profile)
		self._to_frame = 'odom'
		self._from_frame = 'chassis'		

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

	def grid_dist(self, cell1, cell2):
		return np.sqrt((cell1[0]-cell2[0])**2+(cell1[1]-cell2[1])**2)
	
	def timer_callback(self):
		#get next
		if self.gotgoal == False: 
			sleep(1)
		else: 
			dist = self.grid_dist(self.pos, self.goals[self.cur_goal_i])
			print('Cur pos: ', self.pos)
			# print('Current goal: ', self.goals[self.cur_goals_i])
			print('Dist to goal: ', dist)
			if np.abs(dist) < self.tol and (self.cur_goal_i < self.num_goals):
				#goal achieved
				print('Goal Reached! Goal Number (of 3): ', (self.cur_goal_i+1))
				sleep(3)
				if self.cur_goal_i == self.num_goals-1: 
					print('All goals reached!')
				else: 
					self.cur_goal_i += 1 #increment goals
			elif self.cur_goal_i >= self.num_goals: 
				print('All goals reached!')
			else: #keep going
				# print('Path planning...')
				print('Current goal: ', self.goals[self.cur_goal_i])
				cur_goal = self.goals[self.cur_goal_i]
				path = self.a_star.a_star_grid(map=self.map, start=self.pos, goal=cur_goal) # self.goals[[self.cur_goal_i]])
				# print('Path: ', path)
				target = path[0] #early path waypoint
				# target = cur_goal
				# print('Current target: ', target)
				self.publish_next(target)

	def publish_next(self, target):
		#publish the target!
		msg = Pose()
		msg.position.x = target[0]
		msg.position.y = target[1]

		# print('Publishing target!')
		self.publisher_.publish(msg)

	def goal_callback(self, msg):
		if self.gotgoal == False:
			for i in msg.poses: 
				xp = i.position.x
				yp = i.position.y
				goal = ([xp, yp])
				self.goals.append(goal)
		# print('Got goals!')
		# print(self.goals)
		self.gotgoal = True #only need to update / store goals once

	def map_callback(self, msg): 
		# Grab occupancy grid
		# convert it back to normal map / grid
		# size = self.map_size
		size = int(np.sqrt(len(msg.data)))
		data = msg.data
		# print('Size of data: ', len(msg.data))

		self.map = np.reshape(data, (size, size), order='F')
		# print('Map: ', self.map)


def main(args=None):
	rclpy.init(args=args)

	planner = Planner()

	rclpy.spin(planner)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	planner.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
