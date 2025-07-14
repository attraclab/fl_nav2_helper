import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import subprocess
import os, signal
import time
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

kill_node_list =  [
	'ros2 launch fl_nav2_helper gps_robot_localization.launch.py',
	'static_transform_publisher 0 0 0 0 0 0 1 base_link gps',
	'static_transform_publisher 0 0 0 0 0 0 1 base_link imu_link',
	'navsat_transform_node',
	'mavlink_sender',
	'ekf_filter_node'

]

class NavSatHandler(Node):

	def __init__(self):
		super().__init__("navsat_handler_node")

		### ROS parameters ###
		self.declare_parameter('wait_rtk', False)

		self.add_on_set_parameters_callback(self.parameter_callback)

		self.wait_rtk = self.get_parameter('wait_rtk').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("convert_frame: {}".format(self.wait_rtk))

		self.start_node = False
		self.already_started = False
		self.first_time_fixed = False

		self.fix_period_sec = 2 * 60

		self.fix_sub = self.create_subscription(NavSatFix, "/fix", self.fix_callback, 10)

		self.get_logger().info('navsat_handler started')

		self.timer = self.create_timer(1, self.timer_callback)

	#####################
	### ROS Callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			if (param.name == 'wait_rtk') and (param.type_ == Parameter.Type.BOOL):
				self.wait_rtk = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)

	def fix_callback(self, msg):
		fix_status = msg.status.status

		if fix_status == 2:
			if self.wait_rtk:
				if not self.first_time_fixed:
					self.first_time_stamp = time.time()
					self.first_time_fixed = True

				if ((time.time() - self.first_time_stamp) > self.fix_period_sec):
					self.start_node = True
			else:
				self.start_node = True

	def timer_callback(self):

		if (self.start_node and (not self.already_started)):

			cmd = 'ros2 launch fl_nav2_helper gps_robot_localization.launch.py &'
			subprocess.call(cmd, shell=True)
			self.get_logger().info("start gps_odom_fusion")
			self.already_started = True
			# else:
			# 	for node_to_kill in kill_node_list:
			# 		for line in os.popen('ps ax | grep "{}" | grep -v grep'.format(node_to_kill)):
			# 			fields = line.split()
			# 			pid = fields[0]
			# 			## kill the process 
			# 			os.kill(int(pid), signal.SIGKILL)
			# 			print("Kill process {:d}".format(int(pid)))


def main(args=None):

	rclpy.init(args=None)
	node = NavSatHandler()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	
	main()
