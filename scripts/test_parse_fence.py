import os
from shapely.geometry import Point, Polygon
import numpy as np
import time

fence_path = "/home/raspberry/fence.waypoints"

fence_obj = {
    'polygon':  [],
    'circle': []
}

got_new_polygon = False

def parse_fence_file(fence_path):
    global got_new_polygon, fence_obj

    if os.path.exists(fence_path):
        '''
        fence code
        5001 : polygon inclusion
        5002 : polygon exclusion
        5003 : circle inclusion
        5004 : circle exclusion
        '''
        print("Extract lat/lon from fence_path")
        file = open(fence_path, 'r')
        lines = file.read().splitlines()

        for line in lines:
            # print(line)
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
                            fence_obj["polygon"].append(polygon)
                            got_new_polygon = False

                    ## circle
                    elif ((int(line_list[3]) == 5003) or (int(line_list[3]) == 5004)):
                        lat = float(line_list[8])
                        lon = float(line_list[9])
                        radius = float(line_list[4])
                        circle = (lat, lon, radius)
                        fence_obj["circle"].append(circle)

        file.close()

def get_distance(lat1, lon1, lat2, lon2):

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

def robot_in_fence(robot_lat, robot_lon):
    global fence_obj
    
    robot_point = Point(robot_lon, robot_lat)

    for geofence_coords in fence_obj["polygon"]:
        geofence_polygon = Polygon([(lon, lat) for lat, lon in geofence_coords])


        if geofence_polygon.contains(robot_point):
            return True
        
    for circle in fence_obj["circle"]:
        c_lat = circle[0]
        c_lon = circle[1]
        c_radius = circle[2]
        dist = get_distance(c_lat, c_lon, robot_lat, robot_lon)

        if dist <= c_radius:
            return True
        
    return False

parse_fence_file(fence_path) 

robot_pos = (35.8413527, 139.5242744) # not in fence
# robot_pos = (35.8413326, 139.5243153) # in fence

start_time = time.time()
in_fence = robot_in_fence(robot_pos[0], robot_pos[1])
period = time.time() - start_time
print(in_fence, period)


# parse_fence_file(fence_path)
# print(fence_obj)