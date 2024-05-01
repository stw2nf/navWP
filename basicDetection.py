import requests
import json
import time
import math

# Flask app URL
URL = 'http://172.31.98.63:8080'  # Update with your actual Flask app URL

def send_post_request(url, data):
    try:
        # Send POST request with JSON data
        response = requests.post(url, json=data)

        # Print response
        print(response.status_code)
        print(response.text)
    except Exception as e:
        print(f"Error sending POST request: {e}")

def send_detection(lat, lon, alt, heading, speed, timestamp):
    alert = {
        "locationRating": "A",
        "detectionType": "drone",
        "level": 100,
        "positions": [
            {
            "longitude": lon,
            "latitude": lat,
            "altitude": alt,
            "timestamp": timestamp
            },
        ],
        "heading": heading,
        "speed": speed,
        "sensors": [
            "chase_drone",
        ],
        "identification": {
            "detectionType": "drone",
            "model": "S500",
            "manufacturer": "Holybro",
            "protocol": "Mavlink",
            "serial": "serialNumberOfDrone"
        },
        "detectionId": 4,
        "positionState": "ALIVE",
        "uncertainty": "",
        "bearings": {}
    }
    send_post_request(URL, alert)

# Initial latitude and longitude values
initial_lat = 38.502363
initial_lon = -90.490937
#initial_lat = 38.5
#initial_lon = -90.4
radius = 0.0001  # Approximately 100 meters in degrees (adjust as needed)

# Simulate drone moving in a circle
iteration = 0
while True:  # Run indefinitely
    # Calculate next point on the circle
    angle = iteration * (math.pi / 180)  # Convert iteration to radians
    latitude = initial_lat + radius * math.sin(angle)
    longitude = initial_lon + radius * math.cos(angle)

    # Send updated data as a POST request
    print(f"Iteration {iteration + 1}: Sending updated data...")
    send_detection(latitude, longitude, 5.0, 180.0, 7.5, time.time())

    # Increment iteration
    iteration += 1

    # Optional: Add a delay between iterations
    time.sleep(.1)  # 1 second delay

