import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import time
import os, signal


class RelayTopics(Node):

	def __init__(self):

		super().__init__('relay_topics_node')

		### ROS parameters ###
		self.declare_parameter('sensor_offset', 0.2)
		self.declare_parameter('do_offset', True)
		self.declare_parameter('pub_odom_tf', True)


		self.add_on_set_parameters_callback(self.parameter_callback)

		self.sensor_offset = self.get_parameter('sensor_offset').get_parameter_value().double_value
		self.do_offset = self.get_parameter('do_offset').get_parameter_value().bool_value
		self.pub_odom_tf = self.get_parameter('pub_odom_tf').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("sensor_offset: {}".format(self.sensor_offset))
		self.get_logger().info("do_offset: {}".format(self.do_offset))
		self.get_logger().info("pub_odom_tf: {}".format(self.pub_odom_tf))

		#self.sensor_offset = 0.2

		self.kill_node_list =  [
			'ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml',
			'fastlio_mapping --ros-args --params-file',
		]


		### Other parameters ###
		self.got_odom = False
		self.br = TransformBroadcaster(self)

		self.ax = 0.0
		self.ay = 0.0
		self.az = 0.0
		self.gx = 0.0
		self.gy = 0.0
		self.gz = 0.0
		self.got_imu = False

		## sensor_qos
		sensor_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=1)

		## Odometry topic from fast_lio
		self.odom_sub = self.create_subscription(Odometry, "/Odometry", self.odom_callback, sensor_qos)


		self.lidar_odom_pub = self.create_publisher(Odometry, "/lidar_odom", qos_profile=sensor_qos)

		# self.ori_odom_pub = self.create_publisher(Odometry, "/Odometry_ori", 10)

		## laserscan topic from pointcloud_to_laserscan
		scan_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=10)
		self.laser_pub = self.create_subscription(LaserScan, "/lidar_scan", self.lidar_scan_callback, qos_profile=scan_qos)

		## relay topic from /lidar_scan
		self.laser_repeat_pub = self.create_publisher(LaserScan, "/scan", qos_profile=scan_qos)

		# pcl_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, \
		# 									history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
		# 									depth=1)
		# self.cloud_sub = self.create_subscription(PointCloud2, "/cloud_registered_body", self.cloud_callback, qos_profile=pcl_qos)
		# self.cloud_pub = self.create_publisher(PointCloud2, "/lidar_cloud", qos_profile=pcl_qos)

		self.imu_sub = self.create_subscription(Imu, "/livox/imu", self.imu_callback, qos_profile=sensor_qos)
		self.imu_pub = self.create_publisher(Imu, "/lidar_imu", qos_profile=sensor_qos)

		self.since_start_stamp = time.time()

		self.get_logger().info("start relay_topics_node")
		# self.timer = self.create_timer(0.05, self.timer_callback)

	#####################
	### ROS Callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			# print(param.name, param.type_)
			if (param.name == 'sensor_offset') and (param.type_ == Parameter.Type.DOUBLE):
				self.sensor_offset = param.value
			elif (param.name == 'do_offset') and (param.type_ == Parameter.Type.BOOL):
				self.do_offset = param.value
			elif (param.name == 'pub_odom_tf') and (param.type_ == Parameter.Type.BOOL):
				self.pub_odom_tf = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)


	def lidar_scan_callback(self, msg):
		laser_msg = LaserScan()
		laser_msg = msg
		laser_msg.header.stamp = self.get_clock().now().to_msg() #msg.header.stamp
		laser_msg.header.frame_id = "laser_frame"
		self.laser_repeat_pub.publish(laser_msg)

		# print(len(msg.ranges))


	def odom_callback(self, msg):

		self.got_odom = True

		### To publish original odometry from 3DLidar but put in under odom frame
		# ori_odom_msg = Odometry()
		# ori_odom_msg.header.stamp = self.get_clock().now().to_msg()
		# ori_odom_msg.header.frame_id = "odom"
		# ori_odom_msg.child_frame_id = "lidar_ori"
		# ori_odom_msg.pose = msg.pose
		# ori_odom_msg.twist = msg.twist
		# self.ori_odom_pub.publish(ori_odom_msg)

		### For Lidar odom
		odom_msg = Odometry()
		odom_msg.header.stamp = self.get_clock().now().to_msg()
		odom_msg.header.frame_id = "odom"
		odom_msg.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
		odom_msg.pose = msg.pose
		odom_msg.twist = msg.twist

		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w

		x_l = msg.pose.pose.position.x
		y_l = msg.pose.pose.position.y
		z_l = msg.pose.pose.position.z

		if self.do_offset:

			q_list = [qx, qy, qz, qw]
			(roll, pitch, yaw) = euler_from_quaternion(q_list)

			x_b = x_l - self.sensor_offset*np.cos(yaw)
			y_b = y_l - self.sensor_offset*np.sin(yaw)

		else:

			x_b = x_l
			y_b = y_l

		odom_msg.pose.pose.position.x = x_b
		odom_msg.pose.pose.position.y = y_b

		if (self.got_imu):
			imu_msg = Imu()
			imu_msg.header.frame_id = "imu_link"
			imu_msg.header.stamp = self.get_clock().now().to_msg()
			imu_msg.orientation.x = qx
			imu_msg.orientation.y = qy
			imu_msg.orientation.z = qz
			imu_msg.orientation.w = qw
			imu_msg.angular_velocity.x = self.gx
			imu_msg.angular_velocity.y = self.gy
			imu_msg.angular_velocity.z = self.gz
			imu_msg.linear_acceleration.x = self.ax
			imu_msg.linear_acceleration.y = self.ay
			imu_msg.linear_acceleration.z = self.az
			self.imu_pub.publish(imu_msg)



		#self.get_logger().info("xl: {:.3f} yl: {:.3f} xb: {:.3f} yb: {:.3f} yaw: {:.2f}".format(x_l, y_l, x_b, y_b, yaw))

		if self.pub_odom_tf:
			# construct tf
			t = TransformStamped()
			t.header.frame_id = "odom" 
			t.header.stamp = self.get_clock().now().to_msg()
			t.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
			t.transform.translation.x = x_b #msg.pose.pose.position.x 
			t.transform.translation.y = y_b #msg.pose.pose.position.y
			t.transform.translation.z = msg.pose.pose.position.z

			t.transform.rotation.x = msg.pose.pose.orientation.x
			t.transform.rotation.y = msg.pose.pose.orientation.y
			t.transform.rotation.z = msg.pose.pose.orientation.z
			t.transform.rotation.w = msg.pose.pose.orientation.w
			self.br.sendTransform(t)

		self.lidar_odom_pub.publish(odom_msg)

		### detect position jump error which will happen sometime when start fast-lio
		if ((time.time() - self.since_start_stamp) < 300):
			if ((abs(x_l) > 100.0) or (abs(y_l) > 100.0) or (abs(z_l) > 100.0)):
				self.get_logger().info('Position jump error, restart fast-lio')
				for node_to_kill in self.kill_node_list:
					for line in os.popen('ps ax | grep "{}" | grep -v grep'.format(node_to_kill)):
						fields = line.split()
						pid = fields[0]
						os.kill(int(pid), signal.SIGKILL)
						self.get_logger().info("Kill process {:d}".format(int(pid)))

	## /livox/imu has no orientation data,
	## we cannot orientation from this
	## but gyro and accel are good
	def imu_callback(self, msg):

		self.got_imu = True
		self.gx = msg.angular_velocity.x
		self.gy = msg.angular_velocity.y
		self.gz = msg.angular_velocity.z
		self.ax = msg.linear_acceleration.x
		self.ay = msg.linear_acceleration.y
		self.az = msg.linear_acceleration.z

	# def cloud_callback(self, msg):
	# 	cloud_msg = PointCloud2()
	# 	cloud_msg = msg
	# 	cloud_msg.header.stamp = self.get_clock().now().to_msg()
	# 	cloud_msg.header.frame_id = "cloud_frame"


	# 	self.cloud_pub.publish(cloud_msg)


	# def timer_callback(self):

	# 	if not self.got_odom:
	# 		lidar_cmd_msg = Bool()
	# 		lidar_cmd_msg.data = True

	# 		self.lidar_cmd_pub.publish(lidar_cmd_msg)

	
			

def main(args= None):
	rclpy.init(args=None)
	node = RelayTopics()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":

	main()