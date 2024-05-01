import time
from pymavlink import mavutil
import random

def send_gcs_message(message):
    # Send a message for QGC to read out loud
    #  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING,
                            message.encode())

def get_param(param):
    # Request parameter
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        param,
        -1
    )
    # Print old parameter value
    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    print('name: %s\tvalue: %d' %
        (message['param_id'].decode("utf-8"), message['param_value']))
    return message['param_value']

def request_position():
    # Send MAV_CMD_SET_MESSAGE_INTERVAL command to request ATTITUDE at one-second interval
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        33,  # Message ID (GLOBAL_POSITION_INT)
        100,  # Interval in microseconds (.1 second)
        0, 0, 0, 0, 0  # Parameters 4-9 (Unused)
    )
    master.mav.send(msg)

def get_postion():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat
    lon = msg.lon
    alt = msg.alt/1000
    alt_agl = msg.relative_alt/1000
    return lat, lon, alt, alt_agl

def set_postition(lat, lon, alt):
    msg = master.mav.set_position_target_global_int_send(
        int((time.time() - boot_time)*1000),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        3576 ,  # Decimal Use Postion
        lat,
        lon,
        alt,
        0,  # VX
        0,  # VY
        0,  # VZ (set to desired velocity)
        0,  # AFX
        0,  # AFY
        0,  # AFZ
        0,  # Yaw Heading
        0   # Yaw rate setpoint (not used in this case)
    )

def main():
    global master
    global boot_time
    # %% Initial Setup
    # Create the connection
    master = mavutil.mavlink_connection('udp:10.0.15.63:14553', source_system=1) # Eaton Wifi UDP
    #master = mavutil.mavlink_connection('tcp:10.0.15.63:5763', source_system=1) # SITL TCP
    master.mav.ping_send(
        int(time.time() * 1e6), # Unix time in microseconds
        0, # Ping number
        0, # Request ping of all systems
        0 # Request ping of all components
    )

    master.wait_heartbeat()
    boot_time = time.time()
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    request_position()
    last_wp_time = 0
    # %% Main
    try:
        while True:
            cur_time = time.time()
            cur_lat, cur_lon, cur_alt_msl, cur_alt_agl = get_postion()
            #print(f"Current Lat: {cur_lat} Lon: {cur_lon} Alt MSL: {cur_alt_msl} Alt AGL: {cur_alt_agl}")
            if cur_time - last_wp_time > 10:
                # Generate a random number between 0 and 1
                sign_lat = random.randint(0, 1)
                sign_lon = random.randint(0, 1)
                sign_alt = random.randint(0, 1)
                lat_offset = random.randint(10000, 50000)
                lon_offset = random.randint(10000, 50000)
                alt_offset = coor_offset = random.randint(5, 10)
                lat_offset = lat_offset if sign_lat == 0 else -lat_offset
                lon_offset = lon_offset if sign_lon == 0 else -lon_offset
                alt_offset = alt_offset if sign_alt == 0 else -alt_offset

                des_lat = cur_lat+lat_offset
                des_lon = cur_lon+lon_offset
                des_alt = cur_alt_msl+alt_offset
                set_postition(des_lat, des_lon, des_alt)
                print(f"Going to Lat: {des_lat} Lon: {des_lon} Alt: {des_alt}")
                last_wp_time = time.time()
            time.sleep(.01)

    except KeyboardInterrupt:
        print("Exiting Script")

    # Close the connection
    master.close()
if __name__ == "__main__":
    main()