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

def request_data():
    # Send MAV_CMD_SET_MESSAGE_INTERVAL command to request all data at one ms interval
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        mavutil.mavlink.MAV_DATA_STREAM_ALL,  # All data streams
        1,  # Interval in microseconds (.1 second)
        0, 0, 0, 0, 0  # Parameters 4-9 (Unused)
    )
    master.mav.send(msg)

def send_takeoff(height):
    # Send the MAV_CMD_NAV_TAKEOFF command to the vehicle
    msg = master.mav.command_long_encode(
        master.target_system,    # Target system ID
        master.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command code for takeoff
        0,     # Confirmation
        0,     # Param1: Minimum pitch (not used for takeoff)
        0,     # Param2: Empty for takeoff (not used)
        0,     # Param3: Empty for takeoff (not used)
        0,     # Param4: Empty for takeoff (not used)
        0,     # Param5: X position (not used for takeoff)
        0,     # Param6: Y position (not used for takeoff)
        height # Param7: Altitude (in meters) to take off to
    )
    master.mav.send(msg)
    print("Takeoff command sent", height, "meters.")

def is_armed():
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed:
            return True
        else:
            return False
    else:
        return False

# Function to get the current mode of the vehicle
def get_mode():
    # Request the current mode from the vehicle
    master.wait_heartbeat()
    mode = master.mode_mapping()[master.flightmode]
    return mode

# Function to arm the vehicle
def arm_vehicle():
    # Send the MAV_CMD_COMPONENT_ARM_DISARM command to arm the vehicle
    msg = master.mav.command_long_encode(
        master.target_system,    # Target system ID
        master.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command code for arming
        0,  # Confirmation
        1,  # Param1: Arm (1 to arm, 0 to disarm)
        0,  # Param2: Empty
        0,  # Param3: Empty
        0,  # Param4: Empty
        0,  # Param5: Empty
        0,  # Param6: Empty
        0   # Param7: Empty
    )
    master.mav.send(msg)
    print("Arm command sent.")

# Function to arm the vehicle
def disarm_vehicle():
    # Send the MAV_CMD_COMPONENT_ARM_DISARM command to arm the vehicle
    msg = master.mav.command_long_encode(
        master.target_system,    # Target system ID
        master.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command code for arming
        0,  # Confirmation
        0,  # Param1: Arm (1 to arm, 0 to disarm)
        0,  # Param2: Empty
        0,  # Param3: Empty
        0,  # Param4: Empty
        0,  # Param5: Empty
        0,  # Param6: Empty
        0   # Param7: Empty
    )
    master.mav.send(msg)
    print("Disarm command sent.")

def get_position():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat
    lon = msg.lon
    alt = msg.alt/1000
    alt_agl = msg.relative_alt/1000
    return lat, lon, alt, alt_agl

def set_position(lat, lon, alt):
    msg = master.mav.set_position_target_global_int_send(
        int((time.time() - boot_time)*1000),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
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

takeoff_height = 15
guided_mode = 4
takeoff_active = False

def main():
    global master
    global boot_time
    global takeoff_active

    # %% Initial Setup
    # Create the connection
    master = mavutil.mavlink_connection('udp:127.0.0.1:14553') # UDP Connection
    master.mav.ping_send(
        int(time.time() * 1e6), # Unix time in microseconds
        0, # Ping number
        0, # Request ping of all systems
        0 # Request ping of all components
    )
    print("Waiting for Vehicle Heartbeat")
    master.wait_heartbeat()
    boot_time = time.time()
    print("Vehicle Connection Established")
    request_data()
    print("Requested Data Streams")
    last_wp_time = 0

    # %% Main
    try:
        while True:
            cur_lat, cur_lon, cur_alt_msl, cur_alt_agl = get_position()
            if (not is_armed()) and (get_mode() == guided_mode):
                arm_vehicle()
                send_takeoff(takeoff_height)
                takeoff_active = True
            if takeoff_active == True:
                if cur_alt_agl < .95*takeoff_height:
                    print(f"Current Height: {cur_alt_agl}")
                    continue
                else:
                    print("Successful Takeoff")
                    takeoff_active=False
            cur_time = time.time()
            #print(f"Current Lat: {cur_lat} Lon: {cur_lon} Alt MSL: {cur_alt_msl} Alt AGL: {cur_alt_agl}")
            if cur_time - last_wp_time > 10 and get_mode() == guided_mode and is_armed() and takeoff_active == False:
                # Generate a random number between 0 and 1
                sign_lat = random.randint(0, 1)
                sign_lon = random.randint(0, 1)
                sign_alt = random.randint(0, 1)
                lat_offset = random.randint(10000, 50000)
                lon_offset = random.randint(10000, 50000)
                alt_offset = random.randint(5, 10)
                lat_offset = lat_offset if sign_lat == 0 else -lat_offset
                lon_offset = lon_offset if sign_lon == 0 else -lon_offset
                alt_offset = alt_offset if sign_alt == 0 else -alt_offset

                des_lat = cur_lat+lat_offset
                des_lon = cur_lon+lon_offset
                des_alt = cur_alt_agl + alt_offset
                set_position(cur_lat, cur_lon, des_alt)
                print('Current Lat: %f Lon: %f Alt: %f' % (cur_lat, cur_lon, cur_alt_agl))
                print('Going to Lat: %f Lon: %f Alt: %f' % (des_lat, des_lon, des_alt))
                last_wp_time = time.time()
            time.sleep(.01)

    except KeyboardInterrupt:
        print("Exiting Script")

    # Close the connection
    master.close()
if __name__ == "__main__":
    main()