from pymavlink import mavutil, mavwp
import time
import os

def fence_fetch_point(i):
	'''fetch one fence point'''
	master.mav.fence_fetch_point_send(master.target_system, master.target_component, i)
	tstart = time.time()
	p = None
	while time.time() - tstart < 3:
		p = master.recv_match(type='FENCE_POINT', blocking=False)
		if p is not None:
			break
		time.sleep(0.1)
		continue
	if p is None:
		print("Failed to fetch point %u" % i)
		return None
	return p

os.environ["MAVLINK20"] = "2"

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=1, source_component=190)
# master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=200, source_component=190)
master.wait_heartbeat()


# master.waypoint_request_list_send()
# time.sleep(1)

fence = mavwp.MAVFenceLoader(master.target_system, master.target_component)

p = fence_fetch_point(0)
print(p)

# recv_fence = False
# fence_points = []
# while True:

#     msg = master.recv_match(type='MISSION_ITEM_INT', blocking=False, timeout=5)
#     if (msg is not None):
#         print(msg)
#     if (msg is not None) and (msg.get_type() == "MISSION_ITEM_INT"):
#         if msg.seq == 0:
#             continue  # Skip the home location message
#         if (msg.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) or (msg.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION):
#             fence_points.append({
#                 'lat': msg.x,
#                 'lon': msg.y,
#                 'alt': msg.z,
#                 'frame': msg.frame,
#                 'command': msg.command
#             })
#             print(f"Fence point {msg.seq}: lat={msg.x}, lon={msg.y}, alt={msg.z}, frame={msg.frame}, command={msg.command}")
            


# print("after loop")