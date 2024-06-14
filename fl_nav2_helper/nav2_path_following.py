import time
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8MultiArray, UInt8, Float32, Int16MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Transform
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from lifecycle_msgs.srv import GetState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler
import numpy as np
import os
import json

class Nav2PathFollwoing(Node):

	def __init__(self):
		super().__init__('nav2_path_following')
		print("start nav2_path_following")

		## Sbus ##
		self.ch6 = 144
		self.ch9 = 144

		self.prev_ch6 = self.ch6
		self.prev_ch9 = self.ch9

		### Pose recording ###
		self.x_list = []
		self.y_list = []
		self.got_start_pose = False
		self.sampling_dist = 5.0 #1.0
		self.record_path = False
		self.enable_nav = False
		self.dist = 0.0
		self.last_record_log_stamp = time.time()

		### Nav2 WaypointFollower ###
		self.follow_waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
		self.got_followWaypoint_goal = False
		self.followWaypoint_done = False
		self.goal_poses = []
		self.map_goal_poses = []
		self.closest_idx = 0
		self.current_waypoint = 0
		self.map_current_waypoint = 0
		self.goal_poses_len = 0
		self.feedback_last_stamp = time.time()
		self.last_cancel_stamp = time.time()
		self.first_time = True

		self.sbus_sub = self.create_subscription(Int16MultiArray, '/zmoab/sbus_rc_ch', self.sbus_callback, 10)
		self.odom_sub = self.create_subscription(Odometry, '/lidar_odom', self.lidar_odom_callback, 10)
		self.follow_path_pub = self.create_publisher(Path, '/following_path', 10)
		self.follow_path_msg = Path()
		self.follow_path_msg.header.frame_id = 'map'
		self.goal_poses_pub = self.create_publisher(PoseArray, '/goal_pose_array', 10)
		self.goal_poses_msg = PoseArray()
		self.goal_poses_msg.header.frame_id = 'map'

		############
		### Loop ###
		############
		
		self.timer = self.create_timer(0.05, self.timer_callback)


	#####################################
	### FollowWaypoint action client ###
	#####################################

	def followWaypoint_send_goal(self, poses):

		self.get_logger().info("Waiting for 'FollowWaypoint' action server")
		while not self.follow_waypoint_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().info("FollowWaypoint action server not available, waiting...")

		goal_msg =  FollowWaypoints.Goal() #FollowWaypoint.Goal()
		goal_msg.poses = poses
		#goal_msg.behavior_tree = ''

		self.got_followWaypoint_goal = True
		self.followWaypoint_done = False
		self.nav2pose_done = False

		self.get_logger().info(f'FollowWaypoint Navigating with {len(goal_msg.poses)} goals....')
		send_goal_future = self.follow_waypoint_client.send_goal_async(goal_msg, self.followWaypoint_feedbackCallback)
		send_goal_future.add_done_callback(self.followWaypoint_goal_response_callback)

	def followWaypoint_goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('FollowWaypoint Goal rejected :(')
			return

		self.get_logger().info('FollowWaypoint Goal accepted :)')
		print("followWaypoint_done", self.followWaypoint_done)
		self.followWaypoint_goal_handle = goal_handle

		get_result_future = goal_handle.get_result_async()
		get_result_future.add_done_callback(self.followWaypoint_get_result_callback)

	def followWaypoint_get_result_callback(self, future):
		result = future.result().result
		# self.get_logger().info('Result: {0}'.format(result.result))
		self.get_logger().info('FollowWaypoint result {:}'.format(result))
		if self.current_waypoint == (self.goal_poses_len-1):
			self.followWaypoint_done = True
			print("followWaypoint_done", self.followWaypoint_done)
			

	def followWaypoint_feedbackCallback(self, msg):
		# self.get_logger().info('FollowWaypoint feedback {:}'.format(msg.feedback))
		self.current_waypoint = msg.feedback.current_waypoint

		if ((self.closest_idx + self.current_waypoint) >= self.goal_poses_len):
			self.map_current_waypoint = self.closest_idx + self.current_waypoint - self.goal_poses_len 
		else:
			self.map_current_waypoint = self.closest_idx + self.current_waypoint

		if (time.time() - self.feedback_last_stamp) > 1.0:
			print("goal_len {} current_wp {} map_current_wp {}".format(self.goal_poses_len, self.current_waypoint, self.map_current_waypoint))
			self.feedback_last_stamp = time.time()
		# feedback = msg.feedback
		return

	def followWaypoint_cancel_done(self, future):
		cancel_response = future.result()
		self.last_cancel_stamp = time.time()

		if len(cancel_response.goals_canceling) > 0:
			self.get_logger().info('FollowWaypoint Goal successfully canceled')
		else:
			self.get_logger().info('FollowWaypoint Goal failed to cancel')

	####################
	### ROS callback ###
	####################
	def sbus_callback(self, msg):
		self.prev_ch6 = self.ch6
		self.prev_ch9 = self.ch9

		self.ch6 = msg.data[5]
		self.ch9 = msg.data[8]

		if (self.ch9 > 1500) and (self.prev_ch9 != self.ch9):
			self.enable_nav = True

		if (self.ch6 > 1500) and (self.prev_ch6 != self.ch6):
			print("record path start")
			self.record_path = True
		elif (self.ch6 < 1500) and (self.prev_ch6 != self.ch6):
			print("record path stop ch6 {:d} prev_ch6 {:d}".format(self.ch6, self.prev_ch6))
			## put the current point as last point
			self.x_list.append(self.pose_x)
			self.y_list.append(self.pose_y)

			self.record_path = False
			if (len(self.x_list) > 1):
				self.generate_poses_msg()

	def lidar_odom_callback(self, msg):

		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y

	######################
	### Math Functions ###
	######################
	def get_distance_from_points(self, x1, y1, x2, y2):

		vector1 = np.array([x1, y1])
		vector2 = np.array([x2, y2])

		return np.linalg.norm(vector1 - vector2)

	def generate_poses_msg(self):

		hdg_array = []
		x_array = []
		y_array = []
		self.map_goal_poses = []
		

		for i in range(len(self.x_list)):

			if i == (len(self.x_list) - 1):

				x = float(self.x_list[i])
				y = float(self.y_list[i])
				hdg = float(hdg_array[i-1])
				q = quaternion_from_euler(0,0, hdg)

			else:

				x_before = self.x_list[i]
				y_before = self.y_list[i]
				x_next = self.x_list[i+1]
				y_next = self.y_list[i+1]

				x_diff = x_next - x_before
				y_diff = y_next - y_before

				x = x_before
				y = y_before

				hdg = np.arctan2(y_diff, x_diff)

				q = quaternion_from_euler(0,0, hdg)


			hdg_array.append(hdg)

			goal_pose = PoseStamped()
			goal_pose.header.frame_id = 'map'
			goal_pose.header.stamp = self.get_clock().now().to_msg()
			goal_pose.pose.position.x = x
			goal_pose.pose.position.y = y
			goal_pose.pose.orientation.z = q[2]
			goal_pose.pose.orientation.w = q[3]

			print("goal_pose", goal_pose)

			self.map_goal_poses.append(goal_pose)
			self.goal_poses_len = len(self.map_goal_poses)

			self.follow_path_msg.header.stamp = self.get_clock().now().to_msg()
			self.follow_path_msg.poses.append(goal_pose)

			print("waypoint: {:d} x: {:.2f} y: {:.2f}".format(i, x, y))

			

		print("map_goal_poses")
		print(self.map_goal_poses)

		self.x_list = []
		self.y_list = []
		self.got_start_pose = False
		self.dist = 0.0

	############
	### Loop ###
	############
	def timer_callback(self):

		if self.record_path:

			if not self.got_start_pose:

				self.got_start_pose = True
				## store first point
				self.x_list.append(self.pose_x)
				self.y_list.append(self.pose_y)
				self.start_x = self.pose_x
				self.start_y = self.pose_y

				## store goal poses
				self.goal_poses_msg.poses = []
				pose = Pose()
				pose.position.x = self.pose_x
				pose.position.y = self.pose_y
				pose.position.z = 0.0
				pose.orientation.x = 0.0
				pose.orientation.y = 0.0
				pose.orientation.z = 0.0
				pose.orientation.w = 1.0

				self.goal_poses_msg.header.stamp = self.get_clock().now().to_msg()
				self.goal_poses_msg.poses.append(pose)

				self.follow_path_msg.poses = []


			if len(self.x_list) > 0:

				self.dist = self.get_distance_from_points(self.start_x, self.start_y, self.pose_x, self.pose_y)
				if self.dist >= self.sampling_dist:
					print("added new point x: {:.2f} y: {:.2f}".format(self.pose_x, self.pose_y))
					self.x_list.append(self.pose_x)
					self.y_list.append(self.pose_y)
					
					## add arrow as waypoint on rviz ##
					x_diff = self.pose_x - self.start_x
					y_diff = self.pose_y - self.start_y
					x = self.start_x
					y = self.start_y
					hdg = np.arctan2(y_diff, x_diff)
					q = quaternion_from_euler(0,0, hdg)

					pose = Pose()

					pose.position.x = self.pose_x
					pose.position.y = self.pose_y
					pose.position.z = 0.0
					pose.orientation.x = q[0]
					pose.orientation.y = q[1]
					pose.orientation.z = q[2]
					pose.orientation.w = q[3]

					self.goal_poses_msg.header.stamp = self.get_clock().now().to_msg()
					self.goal_poses_msg.poses.append(pose)

					## update new start point
					self.start_x = self.pose_x
					self.start_y = self.pose_y

			self.total_points = len(self.x_list)

			if (time.time() - self.last_record_log_stamp) > 1.0:
				self.last_record_log_stamp = time.time()
				print("points: {:d} pose_x: {:.2f} pose_y: {:.2f} dist: {:.2f}".format(\
					self.total_points, self.pose_x, self.pose_y, self.dist))
					


		if self.enable_nav:
			print("start following path")
			self.enable_nav = False
			print(self.map_goal_poses)
			self.followWaypoint_send_goal(self.map_goal_poses)

		## keep publishing path
		self.follow_path_pub.publish(self.follow_path_msg)
		## keep publishing goalposes
		self.goal_poses_pub.publish(self.goal_poses_msg)


def main(args=None):
	rclpy.init(args=args)
	node = Nav2PathFollwoing()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()

if __name__ == '__main__':
	main()