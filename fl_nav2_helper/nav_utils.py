import numpy as np

####################
### Maths helper ###
####################
def clamp(val, smallest, biggest):
	return max(smallest, min(val, biggest))

def map_with_limit(val, in_min, in_max, out_min, out_max):
	"""
	val: input value
	in_min: input minimum value
	in_max: input maximum value
	out_in: the value to map to in_min
	out_max: the value to map to in_max

	return output mapped value
	"""

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

	return out	

def get_distance_between_points(point1, point2):
	"""
	point1: np.array or list of [x1, y1]
	point2: np.array or list of [x2, y2]

	return distance in meter
	"""

	if not isinstance(point1, np.ndarray):
		point1 = np.array(point1)

	if not isinstance(point2, np.ndarray):
		point2 = np.array(point2)

	return np.linalg.norm(point1-point2)

def get_path_heading(goal_pose, start_pose):
	"""
	goal_pose: np.array or list of goal pose [x, y]
	start_pose: np.array or list of start pose [x, y]

	return heading angle in radians
	"""

	hdg = np.arctan2((goal_pose[1]-start_pose[1]), (goal_pose[0]-start_pose[0]))
	return hdg

def smallestDiffAng_fromPath(goal_pose, start_pose, bot_hdg):
	"""
	https://www.nagwa.com/en/videos/467173691818/
	consider path vector is from start to goal pose (A vector in video)
	bot vector is unit vector (B vector in video)

	goal_pose and start_pose are two points and considered as path
	could be np.array of [x,y]

	bot_hdg is robot heading in radians

	return smallest diff angle between path and bot_hdg, and direction to turn + is CCW, - is CW
	"""
	path_vector = [goal_pose[0] - start_pose[0], goal_pose[1] - start_pose[1]]
	bot_vector = [np.cos(bot_hdg), np.sin(bot_hdg)]
	dot_ = np.dot(path_vector, bot_vector)
	path_norm = np.linalg.norm(path_vector)
	bot_norm = 1.0

	smallest_ang = np.arccos(dot_/(path_norm*bot_norm))

	# ## check turning direction
	check_V = np.array([path_vector, bot_vector])
	det = np.linalg.det(check_V)

	if det > 0.0:
		dir_sign = -1.0
	elif det < 0.0:
		dir_sign = 1.0
	else:
		dir_sign = 0.0

	return np.degrees(smallest_ang), dir_sign

def smallest_angle_between_heading(goal_hdg, bot_hdg):
	"""
	https://www.nagwa.com/en/videos/467173691818/

	goal_hdg: angle of goal in degrees
	bot_hdg: angle of robot heading in degrees

	return smallest difference angle, and direction to turn + is CCW, - is CW
	"""
	goal_vector = [np.cos(np.radians(goal_hdg)), np.sin(np.radians(goal_hdg))]
	bot_vector = [np.cos(np.radians(bot_hdg)), np.sin(np.radians(bot_hdg))]
	dot_ = np.dot(goal_vector, bot_vector)
	goal_norm = 1.0
	bot_norm = 1.0
	smallest_ang = np.degrees(np.arccos(dot_/(goal_norm*bot_norm)))
	smallest_ang = self.ConvertTo360Range(smallest_ang)

	check_V = np.array([goal_vector, bot_vector])
	det = np.linalg.det(check_V)

	if det > 0.0:
		dir_sign = -1.0
	elif det < 0.0:
		dir_sign = 1.0
	else:
		dir_sign = 0.0
		
	return smallest_ang, dir_sign

######################
### Turning Motion ###
######################
def turning_to_goalPoint(goal_pose, bot_pose, mb_yaw, pid_turn, wz_end_clamp, min_angle):
	"""
	Keeps calling this function in loop, and feed input
	Using Turning PID to turn to bot_hdg to goal_pose

	goal_pose: np.array or list of goal point [x,y]
	bot_pose: np.array or list of robot point [x,y]
	mb_yaw: in radians
	pid_turn: PID object declared in main scripts
	wz_end_clamp: the limit value of wz
	min_angle: minimum threshold angle to consider as finish

	return Success (True,False), diff_ang (with sign +/-), wz for cmd_vel
	"""

	pid_turn.auto_mode = True
	diff_ang, sign = smallestDiffAng_fromPath(goal_pose, bot_pose, mb_yaw)
	pid_out_turn = pid_turn(diff_ang*sign)
	wz = clamp(-pid_out_turn, -wz_end_clamp, wz_end_clamp)

	if (abs(diff_ang) <= min_angle):
		wz = 0.0
		pid_turn.auto_mode = False
		return True, diff_ang*sign, wz

	else:
		return False, diff_ang*sign, wz

def turning_to_targetHeading(goal_hdg, bot_hdg, pid_turn, wz_end_clamp, min_angle):
	"""
	Keeps calling this function in loop, and feed input
	Using Turning PID to turn to bot_hdg to goal_hdg

	goal_hdg: in degrees
	bot_hdg: in degrees
	pid_turn: PID object declared in main scripts
	wz_end_clamp: the limit value of wz
	min_angle: minimum threshold angle to consider as finish

	return Success (True,False), diff_ang (with sign +/-), wz for cmd_vel
	"""

	pid_turn.auto_mode = True
	diff_ang, sign = smallest_angle_between_heading(goal_hdg, bot_hdg) 
	pid_out_turn = pid_turn(diff_ang*sign)
	wz = clamp(-pid_out_turn, -wz_end_clamp, wz_end_clamp)

	if (abs(diff_ang) <= min_angle):
		wz = 0.0
		pid_turn.auto_mode = False
		return True, diff_ang*sign, wz

	else:
		return False, diff_ang*sign, wz

def turning_to_pathHeading(goal_pose, start_pose, mb_yaw, pid_turn, wz_end_clamp, min_angle):
	"""
	Keeps calling this function in loop, and feed input
	Using Turning PID to turn

	goal_pose: np.array or list of [x,y]
	start_pose: np.array or list of [x,y]
	mb_yaw: robot heading in radians
	pid_turn: PID object declared in main scripts
	wz_end_clamp: the limit value of wz
	min_angle: minimum threshold angle to consider as finish

	return Success (True,False), diff_ang (with sign +/-), wz for cmd_vel
	"""

	pid_turn.auto_mode = True
	diff_ang, dir_sign = smallestDiffAng_fromPath(goal_pose, start_pose, mb_yaw)
	pid_out_turn = pid_turn(diff_ang*dir_sign)
	wz = clamp(-pid_out_turn, -wz_end_clamp, wz_end_clamp)

	if abs(diff_ang) <= min_angle:
		wz = 0.0
		pid_turn.auto_mode = False
		return True, diff_ang*dir_sign, wz
	else:
		return False, diff_ang*dir_sign, wz

def turning_by_diffAng(diff_ang, pid_turn, wz_end_clamp, min_angle):
	"""
	Keeps calling this function in loop, and feed input
	Using Turning PID to turn

	diff_ang: difference angle
	pid_turn: PID object declared in main scripts
	wz_end_clamp: the limit value of wz
	min_angle: minimum threshold angle to consider as finish

	return Success (True, False), wz

	"""

	pid_turn.auto_mode = True
	pid_out_turn = pid_turn(diff_ang)
	wz = clamp(-pid_out_turn, -wz_end_clamp, wz_end_clamp)

	if abs(diff_ang) <= min_angle:
		wz = 0.0
		pid_turn.auto_mode = False
		return True, wz

	else:
		return False, wz
