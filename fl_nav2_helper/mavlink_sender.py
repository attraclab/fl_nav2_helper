import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float64MultiArray
from sensor_msgs.msg import NavSatFix, TimeReference, LaserScan
from pymavlink import mavutil
import time
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math as m
import numpy as np
import os
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

os.environ["MAVLINK20"] = "2"

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=1, source_component=190)
master.wait_heartbeat()

def send_gps_input(lat, lon, alt, fix):
	"""Sends a GPS_INPUT message."""
	msg = master.mav.gps_input_encode(
		int(time.time() * 1e3),		# time_usec
		0,							# gps_id
		0b000000000000,  		 	# ignore_flags (use all data)
		int((time.time() * 1000) % 604800000), 	# GPS time in milliseconds since the start of the week
		0,                       	# time_week (fake)
		fix,                       	# fix_type (3D fix)
		int(lat * 1e7),				# lat 
		int(lon * 1e7),				# lon
		int(alt*1000),				# alt (mm)
		100,  						# HDOP (cm*100)
		100,  						# VDOP (cm*100)
		0,							# Velocity (cm/s) in N
		0,							# Velocity (cm/s) in E
		0,							# Velocity (cm/s) in D
		100,						# speed_accuracy (cm/s * 100)
		100,						# horiz_accuracy (cm*100)
		100,						# vert_accuracy (cm*100)
		10,							# satellites_visible
	)

	# print(msg)

	master.mav.send(msg)

def send_vision_odom(x,y,z,roll,pitch,yaw, reset_counter):
	covariance = np.array([
		1E-29, 0, 0, 0, 0 ,0,
		1E-29, 0, 0, 0, 0,
		1E-29, 0, 0, 0,
		1E-31, 0, 0,
		1E-31, 0,
		1E-31])
	vision_msg = master.mav.vision_position_estimate_encode(
		int(time.time() * 1e6),
		x, y, z,
		roll, pitch, yaw,
		covariance,
		reset_counter
	)
	master.mav.send(vision_msg)

def request_msg_interval(message_id: int, frequency: float):
	master.mav.command_long_send(
		master.target_system, master.target_component,
		mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
		message_id, 1e6/frequency,
		0,0,0,0,
		0,
	)

def send_home_origin(lat, lon, alt):
	'''
		lat, lon int 1e7
		alt is in mm int
	'''
	## Set home
	master.mav.command_long_send(
		master.target_system, master.target_component,
		mavutil.mavlink.MAV_CMD_DO_SET_HOME, 
		0,
		0, # 1 use current location, 0 use specified location
		0, 0, 0,
		lat/1e7, lon/1e7, alt/1000
	)

	## SET_GPS_GLOBAL_ORIGIN 
	# master.mav.gps_set_global_origin_send(
	master.mav.gps_global_origin_send(
		master.target_system, master.target_component,
		int(lat),
		int(lon),
		int(alt)
	)
	

def send_ekf_src(src):
	master.mav.command_long_send(
		master.target_system, master.target_component,
		mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
		0,
		src,
		0, 0, 0, 0, 0, 0
	)

def send_rc_channel_pwm(id, pwm):
	rc_channel_values = [65535 for _ in range(8)]
	rc_channel_values[id - 1] = pwm
	master.mav.rc_channels_override_send(
		master.target_system, master.target_component,
		*rc_channel_values
	)

def send_preflight_reboot():
	master.mav.command_long_send(
		master.target_system, master.target_component,
		mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
		0,
		1,
		0, 0, 0, 0, 0, 0
	)

angle_shifted = 180
min_distance = 5
max_distance = 1500
distance_array_length = 72
angle_offset = -angle_shifted
increment_f = (angle_shifted*2)/72

def send_obstacle_distance(distance_array):
	
	master.mav.obstacle_distance_send(
		int(round(time.time() * 1e6)),# time_usec
		0,							# sensor_type MAV_DISTANCE_SENSOR
		distance_array,				# distance array
		0,							# increment
		min_distance,				# min dist in cm
		max_distance,				# max dist in cm
		increment_f,				# deg increment
		angle_offset,				# offset angle
		12							# MAV_FRAME_BODY_FRD 
		)



class MAVLinkSender(Node):

	def __init__(self):
		super().__init__("mavlink_sender_node")

		### ROS parameters ###
		self.declare_parameter('convert_frame', True)
		self.declare_parameter('fix_wait_sec', 120.0)
		self.declare_parameter('wait_gps', True)
		self.declare_parameter('send_obstacle', False)

		self.add_on_set_parameters_callback(self.parameter_callback)

		self.convert_frame = self.get_parameter('convert_frame').get_parameter_value().bool_value
		self.fix_wait_sec = self.get_parameter('fix_wait_sec').get_parameter_value().double_value
		self.wait_gps = self.get_parameter('wait_gps').get_parameter_value().bool_value
		self.send_obstacle = self.get_parameter('send_obstacle').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("convert_frame: {}".format(self.convert_frame))
		self.get_logger().info("fix_wait_sec: {}".format(self.fix_wait_sec))
		self.get_logger().info("wait_gps: {}".format(self.wait_gps))
		self.get_logger().info("send_obstacle: {}".format(self.send_obstacle))



		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0
		self.gps_fix = 1

		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.got_gps = False
		self.got_odom = False

		self.reset_counter = 1

		self.hb_stamp = time.time()
		self.gps_stamp = time.time()
		self.odom_stamp = time.time()
		self.last_recv_stamp = time.time()

		self.first_time_fix = False
		self.allowSendOdom = False
		self.prev_allowSendOdom = self.allowSendOdom
		self.start_send_odom = False

		self.check_buffer = 10
		self.fix_type = 0
		self.gps_lat = 0.0
		self.gps_lon = 0.0
		
		self.lat_list = np.array([], dtype=np.int32) 
		self.lon_list = np.array([], dtype=np.int32)

		self.set_origin = False

		### Laserscan ###
		self.front_min_scan_ang = -angle_shifted
		self.front_max_scan_ang = angle_shifted
		self.got_scan = False
		self.got_lidar_data = False
		self.distance_array = []
		self.got_scan = False
		self.last_send_obstacle_stamp = time.time()

		### Publish rate
		self.last_pub_fix = time.time()
		self.last_pub_gps = time.time()

		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
											durability=rclpy.qos.DurabilityPolicy.VOLATILE, 
											depth=1)
		# self.gps_sub = self.create_subscription(NavSatFix, "/gps/filtered", self.gps_callback, qos_profile=qos_policy)
		# self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, qos_profile=qos_policy)
		# self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)

		self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, qos_profile=qos_policy)
		self.scan_sub = self.create_subscription(LaserScan, "/lidar_scan", self.scan_callback, qos_profile=qos_policy)

		self.ekf_src_sub = self.create_subscription(Int8, "/ap/ekf_src", self.ekf_src_callback, 10)
		self.gps_fix_pub =  self.create_publisher(Int8, "/ap/fix", 10)
		self.robot_pos_pub = self.create_publisher(Float64MultiArray, "/ap/gps", 10)

		self.get_logger().info("start mavlink_sender")

		self.timer = self.create_timer(0.01, self.timer_callback)

	########################
	### Helper functions ###
	########################

	def _print(self, msg):
		self.get_logger().info(msg)

	def increment_reset_counter(self):
		if self.reset_counter >= 255:
			self.reset_counter = 1
		self.reset_counter += 1

	def update_lidar_index(self):

		## front stop
		self.front_stop_first_idx = self.lidarAng_to_lidarIdx(self.front_min_scan_ang) 
		self.front_stop_last_idx = self.lidarAng_to_lidarIdx(self.front_max_scan_ang)

	def lidarAng_to_lidarIdx(self, ang):
		return int(self.map_with_limit(ang, -180.0, 180.0, 0.0, (self.scan_length-1)))
	
	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out


	#####################
	### ROS Callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			if (param.name == 'convert_frame') and (param.type_ == Parameter.Type.BOOL):
				self.convert_frame = param.value
			elif (param.name == 'fix_wait_sec') and (param.type_ == Parameter.Type.DOUBLE):
				self.fix_wait_sec = param.value
			elif (param.name == 'wait_gps') and (param.type_ == Parameter.Type.BOOL):
				self.wait_gps = param.value
			elif (param.name == 'send_obstacle') and (param.type_ == Parameter.Type.BOOL):
				self.send_obstacle = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)

	# def gps_callback(self, msg):

	# 	self.lat = msg.latitude
	# 	self.lon = msg.longitude
	# 	self.alt = msg.altitude
	# 	fix_stat = msg.status.status

	# 	# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatStatus.html
	# 	# https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
	# 	if fix_stat == -1:
	# 		## No fix
	# 		self.gps_fix = 1

	# 	elif fix_stat == 0:
	# 		# On Ros, unaugmented fix?? I guess it's 3D fix on APM??
	# 		self.gps_fix = 3

	# 	elif fix_stat == 1:
	# 		# On Ros, with satellite-based augmentation??? I guess it's DGPS on APM??
	# 		self.gps_fix = 4

	# 	elif fix_stat == 2:
	# 		# On Ros, with ground-based augmentation??? I guess it's RTK-Fixed on APM??
	# 		self.gps_fix = 6
	# 		if not self.first_time_fix:
	# 			self.got_rtkFix_stamp = time.time()
	# 			self.first_time_fix = True

	# 	self.got_gps = True

	def odom_callback(self,  msg):
		self.prev_x = self.x
		self.prev_y = self.y
		self.prev_z = self.z

		if self.convert_frame:
			self.x = msg.pose.pose.position.x
			self.y = -msg.pose.pose.position.y
			self.z = msg.pose.pose.position.z

			qx = msg.pose.pose.orientation.y
			qy = msg.pose.pose.orientation.x
			qz = -msg.pose.pose.orientation.z
			qw = msg.pose.pose.orientation.w
		else:
			self.x = msg.pose.pose.position.x
			self.y = msg.pose.pose.position.y
			self.z = msg.pose.pose.position.z

			qx = msg.pose.pose.orientation.x
			qy = msg.pose.pose.orientation.y
			qz = msg.pose.pose.orientation.z
			qw = msg.pose.pose.orientation.w

		q_list = [qx, qy, qz, qw]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(q_list)

		self.got_odom = True

		delta_translation = [self.x - self.prev_x, self.y - self.prev_y, self.z - self.prev_z]
		position_displacement = np.linalg.norm(delta_translation)
		jump_threshold = 0.1
		if (position_displacement > jump_threshold):
			self.increment_reset_counter()

	def ekf_src_callback(self, msg):
		send_ekf_src(msg.data)
		self._print("ekf src {}".format(msg.data))

	def scan_callback(self, msg):
		if not self.got_scan:
			self.scan_length = len(msg.ranges)
			self.got_scan = True
			self.update_lidar_index()
			self._print("scan length is {}".format(self.scan_length))
		else:
			# self.distance_array = msg.ranges[self.front_stop_first_idx:self.front_stop_last_idx]
			self.distance_array = msg.ranges
			self.got_lidar_data = True

	############
	### Loop ###
	############
	def timer_callback(self):

		global master

		# if self.gps_fix == 6:

		# 	rtkFix_period = time.time() - self.got_rtkFix_stamp
		# 	if (rtkFix_period > (2 * 60.0)):
		# 		rtkFix_timeOver = True
		# 	else:
		# 		rtkFix_timeOver = False
		# else:
		# 	rtkFix_timeOver = False

		# latLon_not_Nan = (not m.isnan(self.lat)) and (not m.isnan(self.lon)) and (not m.isnan(self.alt))

		# if self.got_gps and latLon_not_Nan and rtkFix_timeOver:
		# 	if ((time.time() - self.gps_stamp) > 1/20.0):
		# 		send_gps_input(self.lat, self.lon, self.alt, self.gps_fix)
		# 		self.gps_stamp = time.time()

		if not self.set_origin:
			if self.first_time_fix and (self.fix_type == 6):
				self.allowSendOdom = ((time.time() - self.got_rtkFix_stamp) > self.fix_wait_sec)
			# elif (self.first_time_fix and (self.fix_type == 5)):
			# 	self.allowSendOdom = True
			else:
				self.allowSendOdom = False

			if ((self.prev_allowSendOdom != self.allowSendOdom) and (self.allowSendOdom == True)):
				send_preflight_reboot()
				self._print("do preflight reboot")
				send_home_origin(self.ap_lat, self.ap_lon, self.ap_alt)
				self._print("set home as {} {} {}".format(self.ap_lat, self.ap_lon, self.ap_alt))
				self.start_send_odom = True
				### Reconnect again
				master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=1, source_component=190)
				master.wait_heartbeat()
				self.set_origin = True


		if self.got_odom and (not self.wait_gps or self.allowSendOdom):
			if ((time.time() - self.odom_stamp) > 1/20.0):
				self.odom_stamp = time.time()
				send_vision_odom(self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.reset_counter)

		### Recv AP message ###
		msg = master.recv_msg()
		if (msg is not None) and (msg.get_type() == "GPS_RAW_INT"):

			data_dict = msg.to_dict()
			self.ap_lat = data_dict['lat']
			self.ap_lon = data_dict['lon']
			self.ap_alt = data_dict['alt']
			self.fix_type = data_dict['fix_type']
			self.gps_lat = self.ap_lat / 1e7
			self.gps_lon = self.ap_lon / 1e7

			if not self.start_send_odom and self.wait_gps:
				msg = "lat: {} lon: {} fix_type {}".format(self.ap_lat, self.ap_lon, self.fix_type)
				self._print(msg)
				# if (fix_type == 6) or (fix_type == 5):
				# if (self.fix_type == 5):
					
				# 	if len(self.lat_list) < self.check_buffer:
				# 		self.lat_list = np.append(self.lat_list, self.ap_lat)
				# 		self.lon_list = np.append(self.lon_list, self.ap_lon)

				# 	elif len(self.lat_list) == self.check_buffer:
				# 		self.lat_list = np.delete(self.lat_list, 0)
				# 		self.lon_list = np.delete(self.lon_list, 0)

				# 		self.lat_list = np.append(self.lat_list, self.ap_lat)
				# 		self.lon_list = np.append(self.lon_list, self.ap_lon)

				# 		all_lat_same = np.all(self.lat_list == self.lat_list[0])
				# 		all_lon_same = np.all(self.lon_list == self.lon_list[0])

				# 		if all_lat_same and all_lon_same and (not self.first_time_fix):
				# 			self.first_time_fix = True
				# 			self.got_rtkFix_stamp = time.time()
				# 			self._print("float but stable enough")

				if (self.fix_type == 6):
					if not self.first_time_fix:
						self.first_time_fix = True
						self.got_rtkFix_stamp = time.time()
						self._print("got first time rtkFixed")

		elif (msg is not None) and (msg.get_type() == "RC_CHANNELS"):
			# self._print("{}".format(msg.to_dict()['chan8_raw']))
			pass

		self.prev_allowSendOdom = self.allowSendOdom

		## Send DISTANCE_SENSOR ##
		if self.got_lidar_data and self.send_obstacle:

			dist_array = np.copy(self.distance_array)
			dist_array = dist_array[::-1]*100

			array_len = len(dist_array)
			take_every = int(array_len/72)
			dist_array_small = np.round(dist_array[0::take_every]).astype(int)
			dist_array_small[dist_array_small == 0] = 65535
			dist_array_small[dist_array_small > 65535] = 65535
			distances = dist_array_small.tolist()
			if len(distances) > 72:
				dist_len = len(distances)
				remove_idx = dist_len - 72
				distances = distances[:-remove_idx]

			if (time.time() - self.last_send_obstacle_stamp) > 1/20.0:

				self.last_send_obstacle_stamp = time.time()
				send_obstacle_distance(distances)

		### publish gps_fix status
		if ((time.time() - self.last_pub_fix) >= 1/20.0):
			gps_fix_msg = Int8()
			gps_fix_msg.data = self.fix_type
			self.gps_fix_pub.publish(gps_fix_msg)
			self.last_pub_fix = time.time()

		### publish GPS lat/lon
		if ((time.time() - self.last_pub_gps) >= 1/20.0):
			gps_pos_msg = Float64MultiArray()
			gps_pos_msg.data = [self.gps_lat, self.gps_lon]
			self.robot_pos_pub.publish(gps_pos_msg)
			self.last_pub_gps = time.time()

		

		# if (time.time() - self.hb_stamp) >= (1.0/5.0):
		# 	_type = 6
		# 	autopilot = 0					# Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
		# 	base_mode = 0					# System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
		# 	custom_mode = 0					# A bitfield for use for autopilot-specific flags (type:uint32_t)
		# 	system_status = 4				# System status flag. (type:uint8_t, values:MAV_STATE), https://mavlink.io/en/messages/common.html#MAV_STATE
		# 	mavlink_version = 3				# default is 3
		# 	master.mav.heartbeat_send(_type, autopilot, base_mode, custom_mode, system_status, mavlink_version)
		# 	self.hb_stamp = time.time()


def main(args=None):

	rclpy.init(args=None)
	node = MAVLinkSender()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	
	main()

