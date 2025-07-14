from pymavlink import mavutil
import time
import os

os.environ["MAVLINK20"] = "1"

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=1, source_component=190)
# master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=200, source_component=190)
master.wait_heartbeat()

period = 0.0
ap_lat = 0
ap_lon = 0
ch8 = 0

while True:
    start_time = time.time()

    msg = master.recv_msg()

    if (msg is not None) and (msg.get_type() == "GPS_RAW_INT"):
        data_dict = msg.to_dict()
        ap_lat = data_dict['lat']
        ap_lon = data_dict['lon']
        ap_alt = data_dict['alt']
        fix_type = data_dict['fix_type']

    elif (msg is not None) and (msg.get_type() == "RC_CHANNELS"):
        ch8 = msg.to_dict()['chan8_raw']

    print("{} {} {} {}".format(period, ap_lat, ap_lon, ch8))

    period = time.time() - start_time