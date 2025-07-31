import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float64MultiArray
import numpy as np
from shapely.geometry import Point, Polygon
import time
import shutil
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class FenceHandler(Node):

	def __init__(self):

		super().__init__("fence_handler_node")

		### ROS parameters ###
		self.declare_parameter('show_log', True)

		self.add_on_set_parameters_callback(self.parameter_callback)

		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("show_log: {}".format(self.show_log))

		self.fix_type = 0
		self.ap_lat = 0.0
		self.ap_lon = 0.0

		self.fence_dir = "/home/raspberry/fence/"
		self.fence_path = os.path.join(self.fence_dir, "fence.waypoints")
		self.tmp_dir = os.path.join(self.fence_dir, "tmp")

		if not os.path.isdir(self.tmp_dir):
			os.mkdir(self.tmp_dir)


		self.fence_obj = {
			'polygon':  [],
			'circle': []
		}
		self.inside_fence = False
		self.got_new_polygon = False
		self.got_fence_obj = False
		self.got_gps = False
		self.ekf_src = 1
		self.prev_ekf_src = 1
		self.last_not_gpsFix_stamp = time.time()
		self.last_checkNewFile_stamp = time.time()
		self.last_log_stamp = time.time()
		self.period_notFix = 0.0
		## Pub/Sub ##
		self.gps_fix_sub = self.create_subscription(Int8, "/ap/fix", self.gps_fix_callback, 10)
		self.ekf_src_pub = self.create_publisher(Int8, "/ap/ekf_src", 10)
		self.gps_pos_sub = self.create_subscription(Float64MultiArray, "/ap/gps", self.gps_pos_callback, 10)
		

		self.get_logger().info("start fence_handler")
		self.timer = self.create_timer(0.02, self.timer_callback)

	def _print(self, msg):
		self.get_logger().info(msg)
	
	##################
	## ROS Callback ##
	##################
	def parameter_callback(self, params):
		for param in params:
			if (param.name == 'show_log') and (param.type_ == Parameter.Type.BOOL):
				self.show_log = param.value

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)
	
	def gps_fix_callback(self, msg):
		self.fix_type = msg.data

	def gps_pos_callback(self, msg):
		self.ap_lat = msg.data[0]
		self.ap_lon = msg.data[1]

		if (self.ap_lat != 0.0) and (self.ap_lon != 0.0):
			self.got_gps = True

	######################
	## Helper functions ##
	######################
	def parse_fence_file(self, fence_path, read_in_tmp=False):

		if os.path.exists(fence_path):
			'''
			fence code
			5001 : polygon inclusion
			5002 : polygon exclusion
			5003 : circle inclusion
			5004 : circle exclusion
			'''
			self._print("Extract lat/lon from fence_path")
			file = open(fence_path, 'r')
			lines = file.read().splitlines()
			got_new_polygon = False
			for line in lines:
				if len(line)<40:
					pass
				else:
					line_list = line.split('\t')
					## skip home position
					if int(line_list[0]) == 0:
						pass
					else:
						## polygon
						if (int(line_list[3]) == 5001) or (int(line_list[3]) == 5002):
							if not got_new_polygon:
								polygon_pts = int(float(line_list[4]))
								got_new_polygon = True
								polygon = []

							lat = float(line_list[8])
							lon = float(line_list[9])
							polygon.append((lat,lon))

							if len(polygon) == polygon_pts:
								self.fence_obj["polygon"].append(polygon)
								got_new_polygon = False

						## circle
						elif ((int(line_list[3]) == 5003) or (int(line_list[3]) == 5004)):
							lat = float(line_list[8])
							lon = float(line_list[9])
							radius = float(line_list[4])
							circle = (lat, lon, radius)
							self.fence_obj["circle"].append(circle)
							
			file.close()

			if read_in_tmp == False:
				## clear old file in tmp/
				ls_tmp_file = os.listdir(self.tmp_dir)
				for tmp_file in ls_tmp_file:
					tmp_file_path = os.path.join(self.tmp_dir, tmp_file)
					os.remove(tmp_file_path)

				## move new fence file to tmp/
				file_name = fence_path.split("/")[-1]
				new_tmp_file_path = os.path.join(self.tmp_dir, file_name)
				shutil.copyfile(fence_path, new_tmp_file_path)
				os.remove(fence_path)

			return True
		else:
			# self._print("No fence.waypoints is found")
			return False
	
	def get_distance(self, lat1, lon1, lat2, lon2):

		R = 6371.0*1000.0
		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		a = np.sin(dLat/2.0)*np.sin(dLat/2.0) + np.cos(lat_start)*np.cos(lat_end)*np.sin(dLon/2.0)*np.sin(dLon/2.0)
		c = 2.0*np.arctan2(np.sqrt(a),np.sqrt(1-a))

		d = c*R

		return d

	def robot_in_fence(self, robot_lat, robot_lon):
		
		robot_point = Point(robot_lon, robot_lat)

		for geofence_coords in self.fence_obj["polygon"]:
			geofence_polygon = Polygon([(lon, lat) for lat, lon in geofence_coords])

			if geofence_polygon.contains(robot_point):
				return True
			
		for circle in self.fence_obj["circle"]:
			c_lat = circle[0]
			c_lon = circle[1]
			c_radius = circle[2]
			dist = self.get_distance(c_lat, c_lon, robot_lat, robot_lon)

			if dist <= c_radius:
				return True
			
		return False
	
	############
	### Loop ###
	############
	def timer_callback(self):

		### load previous fence file from tmp/
		if not self.got_fence_obj:
			ls_tmp_dir = os.listdir(self.tmp_dir)
			for tmp_file in ls_tmp_dir:
				if (tmp_file.startswith("fence")):
					tmp_file_path = os.path.join(self.tmp_dir, tmp_file)
					self.got_fence_obj = self.parse_fence_file(tmp_file_path, read_in_tmp=True)
					self._print("Load {}".format(tmp_file_path))

		### regularly checking new file every 2 seconds
		if self.got_fence_obj and ((time.time() - self.last_checkNewFile_stamp) > 2.0):
			self.last_checkNewFile_stamp = time.time()
			ls_dir = os.listdir(self.fence_dir)
			for file in ls_dir:
				if (file.startswith("fence")):
					self._print("Got new file, try parsing")
					fence_path = os.path.join(self.fence_dir, file)
					self.parse_fence_file(fence_path)

		### condition to switch ekf_src
		if (self.got_gps and self.got_fence_obj):
			self.inside_fence = self.robot_in_fence(self.ap_lat, self.ap_lon)

			if self.inside_fence:
				self.ekf_src = 2
			else:
				## If GPS not RTK-Fixed then wait until it got fixed 
				## and wait more for 5 seconds to switch back to GPS
				self.period_notFix = (time.time() - self.last_not_gpsFix_stamp)
				if (self.fix_type == 6) and (self.period_notFix > 5.0):
					self.ekf_src = 1
				elif self.fix_type != 6:
					self.last_not_gpsFix_stamp = time.time()

		### Publish ekf_src
		if (self.prev_ekf_src != self.ekf_src):
			self.prev_ekf_src = self.ekf_src
			ekf_src_msg = Int8()
			ekf_src_msg.data = self.ekf_src
			self.ekf_src_pub.publish(ekf_src_msg)


		### Logging ###
		if (((time.time() - self.last_log_stamp) >= 1.0) and self.show_log):
			self._print("inside_fence: {} ekf_src: {} fix: {} period_notFix: {:.2f}".format(\
				self.inside_fence, self.ekf_src, self.fix_type, self.period_notFix))
			self.last_log_stamp = time.time()
		


	


def main(args=None):

	rclpy.init(args=None)
	node = FenceHandler()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	
	main()