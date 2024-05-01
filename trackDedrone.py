from flask import Flask, request
import time
from pymavlink import mavutil
import threading
import math

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

def calc_distance(lat1, lon1, lat2, lon2):
    # Earth radius in kilometers
    earth_radius = 6371000.0  # in meters

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    # Calculate differences
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    # Calculate Haversine distance
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = earth_radius * c
    #print(distance)
    return distance

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

def set_relay(relayNum, state):
    # Send MAV_CMD_SET_MESSAGE_INTERVAL command to request all data at one ms interval
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
        0, #Confirmation
        relayNum,  # Relay Number
        state,  # 1=on, 0=off
        0, 0, 0, 0, 0  # Parameters 3-7 (Unused)
    )
    master.mav.send(msg)
    if state == 1:
        message = "Relay 1 set HIGH"
        send_gcs_message(message)

def control_drone():
    global takeoff_active
    global cur_lat, cur_lon, cur_alt_msl, cur_alt_agl, target_lat, target_lon, target_alt
    while True:
        cur_lat, cur_lon, cur_alt_msl, cur_alt_agl = get_position()
        if (not is_armed()) and (get_mode() == guided_mode): # Vehicle is in guided mode and disarmed -> Takeoff
            arm_vehicle()
            send_takeoff(takeoff_height)
            takeoff_active = True
        if takeoff_active == True:
            if cur_alt_agl < .95*takeoff_height:
                print(f"Current Height: {cur_alt_agl}")
            else:
                print("Successful Takeoff")
                takeoff_active=False
        if get_mode() == guided_mode and is_armed() and takeoff_active == False:
            if target_lat and target_lon and cur_lat and cur_lon:
                distance_to_target = calc_distance(target_lat*10**-7, target_lon*10**-7, cur_lat*10**-7, cur_lon*10**-7)
                if distance_to_target < triggerRelayThresh:
                    set_relay(relayNum=0, state=1)
                    print("Relay 1 set HIGH")
        else:
            set_relay(relayNum=0, state=0)

        time.sleep(.1)  # Adjust sleep duration as needed

# Initialize global variables for drone control
master = mavutil.mavlink_connection('udp:127.0.0.1:14553', source_system=1) # UDP Connection Forwarded from GCS
master.mav.ping_send(
    int(time.time() * 1e6), # Unix time in microseconds
    0, # Ping number
    0, # Request ping of all systems
    0 # Request ping of all components
)
print("Waiting for vehicle heartbeat")
# Wait a heartbeat before sending commands
master.wait_heartbeat()
boot_time = time.time()
print("Vehicle Connection Established")
request_data()
print("Requested Data Streams")
takeoff_height = 5
guided_mode = 4
takeoff_active = False
ignoreVehicleThresh = 300
triggerRelayThresh = 3

# Create a Flask app instance
app = Flask(__name__)

control_thread = threading.Thread(target=control_drone)
control_thread.daemon = True
control_thread.start()

# Define a route to handle incoming POST requests
@app.route('/chase_drone', methods=['POST'])
def chase_drone():
    global takeoff_active
    global cur_lat, cur_lon, cur_alt_msl, cur_alt_agl, target_lat, target_lon, target_alt
    # Extract and parse the JSON data from the POST request
    try:
        data = request.get_json()
        #print("Received Data:", data)  # Debugging output
        # Check if 'detections' key is present and not empty
        if 'data' in data and data['data']:
            data = data['data']
        else:
            #print('No data present')
            return 'No data present'
        if 'detections' in data and data['detections']:
            # Extract the latest detection data
            latest_detection = data['detections'][-1]
            if latest_detection['identification']['detectionType'] == 'drone':    
                # Extract relevant information from the latest detection
                target_lon = int(float(latest_detection['positions'][-1]['longitude'])*10**7)
                target_lat = int(float(latest_detection['positions'][-1]['latitude'])*10**7)
                target_alt = float(latest_detection['positions'][-1]['altitude'])+4.572
                target_heading = latest_detection['heading']
                target_speed = latest_detection['speed']
                if target_lat and target_lon and cur_lat and cur_lon:
                    distance_to_target = calc_distance(target_lat*10**-7, target_lon*10**-7, cur_lat*10**-7, cur_lon*10**-7)
                if get_mode() == guided_mode and is_armed() and takeoff_active == False and distance_to_target < ignoreVehicleThresh:
                    set_position(target_lat, target_lon, target_alt)
                    print('Going to Lat: %f Lon: %f Alt: %f' % (target_lat, target_lon, target_alt))
                else:
                    print('Target at Lat: %f Lon: %f Alt: %f' % (target_lat, target_lon, target_alt))
                return 'Detection data processed successfully.'
            else:
                #print('Invalid Detection Type')
                return 'Invalid Detection Type'
        else:
            #print("No valid detections found in the data.")
            return 'No valid detections found.'
    except Exception as e:
        print(f"Error processing data: {e}")
        return 'Error processing data.'

if __name__ == '__main__':
    # Run the Flask app with the specified IP address and port
    app.run(host='127.0.0.1', port=8080)
