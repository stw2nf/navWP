from flask import Flask, request
import json

# Determine the port to use based on command-line arguments
PORT = 8080
IP_ADDRESS = "127.0.0.1"

# Create a Flask app instance
app = Flask(__name__)

# Define a route to handle incoming POST requests
@app.route('/chase_drone', methods=['POST'])
def chase_drone():
    # Extract and parse the JSON data from the POST request
    try:
        data = request.get_json()
        # Check if 'detections' key is present and not empty
        if 'data' in data and data['data']:
            data = data['data']
        else:
            print('No data present')
            return
        print(data['detections'][-1])
        if 'detections' in data and data['detections']:
            # Extract the latest detection data
            latest_detection = data['detections'][-1]
            if latest_detection['identification']['detectionType'] == 'drone':
                # Extract relevant information from the latest detection
                longitude = latest_detection['positions'][-1]['longitude']
                latitude = latest_detection['positions'][-1]['latitude']
                altitude = latest_detection['positions'][-1]['altitude']
                heading = latest_detection['heading']
                speed = latest_detection['speed']
                # Print the extracted data
                print(f"Latest Detection Data:")
                print(f"Longitude: {longitude}")
                print(f"Latitude: {latitude}")
                print(f"Altitude: {altitude}")
                print(f"Heading: {heading}")
                print(f"Speed: {speed}")
                return 'Detection data processed successfully.'
            else:
                print('Invalid Detection Type')
                return 'Invalid Detection Type'
        else:
            print("No valid detections found in the data.")
            return 'No valid detections found.'
    except Exception as e:
        print(f"Error processing data: {e}")
        return 'Error processing data.'

if __name__ == '__main__':
    # Run the Flask app with the specified IP address and port
    app.run(host=IP_ADDRESS, port=PORT)
