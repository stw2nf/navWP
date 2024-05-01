import requests
import json
import time
import math

# Flask app URL
URL = 'http://127.0.0.1:8080/chase_drone'  # Update with your actual Flask app URL

def send_post_request(url, data):
    try:
        # Send POST request with JSON data
        response = requests.post(url, json=data)

        # Print response
        print(response.status_code)
        print(response.text)
    except Exception as e:
        print(f"Error sending POST request: {e}")

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

    # Create detection data for the current iteration
    data = {
        "data": {
            "detections": [
                {
                    "identification": {
                        "detectionType": "drone"
                    },
                    "positions": [
                        {
                            "longitude": longitude,
                            "latitude": latitude,
                            "altitude": 5.0
                        }
                    ],
                    "heading": 246.0,
                    "speed": 50
                }
            ]
        }
    }

    # Send updated data as a POST request
    print(f"Iteration {iteration + 1}: Sending updated data...")
    send_post_request(URL, data)

    # Increment iteration
    iteration += 1

    # Optional: Add a delay between iterations
    time.sleep(.01)  # 1 second delay

