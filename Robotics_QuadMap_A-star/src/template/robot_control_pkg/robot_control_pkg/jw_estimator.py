import math
import tf2_ros
import rclpy
from time import sleep
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class PlaceholderEstimator(Node):

	def __init__(self):
		super().__init__('PlaceholderEstimator')
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
		
		self.publisher_ = self.create_publisher(Pose, '/en613/state_est', 10)
		#publish occupancy grid message

		self._tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self._tf_buffer,self)

		self._to_frame = 'odom'
		self._from_frame = 'chassis'


	def scan_callback(self, msg):
		#print('msg: ', msg)
		'''
		(I think) take scans + current position (odom) and pass into quad map
			- this will update the quad map with occupied / unoccupied
			- scan format: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
		
		angle = msg.angle_min
		inc = msg.angle_increment
		endpoints = []
		for i in msg.ranges: #create endpoint matrix
			calc x
			calc y
			endpoint = x,y
			endpoints.append(endpoint)
			i+=inc

		for j in endpoints: 
			ray_update(j) #adjust to update occupancy grid? call main?
		'''
		pass

	def odom_callback(self, msg):
		'''
		This callback currently publishes the transfrom message from odom as a state estimate. 
		This is not the way this node should be implemented.
		Update from office hours - no state est needed, can use odom messages
		'''
		try:
			when = rclpy.time.Time()
			trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
													 when, timeout=Duration(seconds=5.0))
		except tf2_ros.LookupException:
			self.get_logger().info('Transform isn\'t available, waiting...')
			sleep(1)
			return

		T_msg = trans.transform.translation
		R_msg = trans.transform.rotation


		T = np.array([T_msg.x,T_msg.y,T_msg.z])
		R = np.array([R_msg.x,R_msg.y,R_msg.z,R_msg.w])

		self.publish_pose(T,R)

	def publish_pose(self,X1,Q1):

		#Chage all of this to self.____
		msg = Pose()
		msg.position.x = X1[0]
		msg.position.y = X1[1]
		msg.position.z = X1[2]
		msg.orientation.w = Q1[0]
		msg.orientation.x = Q1[1]
		msg.orientation.y = Q1[2]
		msg.orientation.z = Q1[3]

		self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    estimator = PlaceholderEstimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
