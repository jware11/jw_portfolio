import math
import rclpy
from time import sleep
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry #added
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformStamped

class Controller(Node):

	def __init__(self):
		super().__init__('Controller')
		
		self.odom_subscriber = self.create_subscription(
			Odometry,
			'/en613/odom',
			self.odom_callback,
			10)

		self.target_subscriber = self.create_subscription(
			Pose,
			'/en613/target',
			self.target_callback,
			10)
		
		#Timer
		pub_freq = 30
		self.dt = 1.0 / pub_freq
		self.timer = self.create_timer(self.dt, self.publish_cmd)

		#Listen to TF
		self._tf_buffer = tf2_ros.Buffer()

		self._to_frame = 'odom'
		self._from_frame = 'chassis'
		self.publisher_ = self.create_publisher(Twist, '/en613/cmd_vel', 10)
		
		#init parameters
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.pos = ([self.x, self.y])
		self.state = ([self.x, self.y, self.theta])

		#target init
		self.xt = 0.0
		self.yt = 0.0
		self.target = ([self.xt, self.yt])

		#robot init
		self.length = -0.000000001 ##something wrong here, length of trailer hitch in front of vehicle
		self.width = 0.2 #was 0.2
		self.wheel_radius = 0.1

		#PID init
		self.k_p = 1.0 #was 5
		self.k_d = 0.2 #was 0.5
		self.k_i = 0.0 #was 10
		
		#set max rates
		self.max_vx = 0.01
		self.max_vth = 0.02

		self.Xp_last = None

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
		
	def target_callback(self, msg):
		self.xt = msg.position.x
		self.yt = msg.position.y		
		self.target = ([self.xt, self.yt])
		# print('Target received...moving to ', self.target)

	def compute_vel(self):
		x_r = self.x
		y_r = self.y
		th_r = self.theta

		# Compute a trailer hitch point in front of the agent
		X_p = np.array([x_r + self.length * np.cos(th_r), y_r + self.length * np.sin(th_r)])

		# Compute the proportional error between the desired position and the final position
		# The [:,None] prefix reshapes the vector from (2,) to (2,1)
		p_err = (X_p - self.target)[:,None]

		if self.Xp_last is None:
			# If this is the first timestep we do not compute the derivative or integral terms
			d_err = 0
			i_err = 0

		else:
			# The velocity of the point is equivalent to the derivative of its error
			d_err = (X_p - self.Xp_last)[:, None] / self.dt

			# For the integral  we don't integrate X_p we integrate the error term instead
			p_err_last = (self.Xp_last - self.target)[:, None]

			# We use the Trapezoidal rule to integrate the error
			i_err = self.dt * (p_err + p_err_last) / 2 

		inv_rot = np.array([[np.cos(th_r), np.sin(th_r)],[- np.sin(th_r),np.cos(th_r)]])

		V = inv_rot @ (-self.k_p * p_err - self.k_d * d_err - self.k_i * i_err)
		linear_x = V[0,0]
		angular_z = V[1,0] / self.length

		self.Xp_last = X_p

		return np.array([linear_x, angular_z])

	def publish_cmd(self):


		desired_vel = self.compute_vel()

		vx = desired_vel[0]
		vth = desired_vel[1]

		#Limit speeds
		if abs(vx) > self.max_vx: 
			# fac = abs(self.max_vx / vx)
			fac = self.max_vx / vx
			vx = vx * fac

		if abs(vth) > self.max_vth: 
			# fac = abs(self.max_vth / vth)
			fac = self.max_vth / vth
			vth = vth * fac

		msg = Twist()
		msg.linear.x = vx
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = vth

		# print('Publishing new velocity...')
		# self.publisher_.publish(msg) #comment out to use teleop twist keyboard


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
