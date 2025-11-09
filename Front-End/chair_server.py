
import serial
import time
from flask import Flask, jsonify

# --- Configuration ---
# IMPORTANT: Replace 'COM3' with the correct serial port for your Arduino.
# On Windows, it might be 'COM3', 'COM4', etc.
# On Linux or macOS, it might be '/dev/ttyACM0', '/dev/ttyUSB0', etc.
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
# --- End Configuration ---

app = Flask(__name__)

# Define the mapping from web commands to serial commands
COMMAND_MAP = {
    "forward": "F",
    "backward": "B",
    "left": "L",
    "right": "R",
    "stop": "S",
}

def send_command(command):
    """Sends a single character command to the Arduino."""
    try:
        # Initialize serial connection
        # The timeout is important to prevent the script from hanging
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for the connection to establish

        if command in COMMAND_MAP:
            serial_command = COMMAND_MAP[command]
            ser.write(serial_command.encode())
            ser.close()
            print(f"Sent command: {command} -> {serial_command}")
            return {"status": "success", "command": command}
        else:
            print(f"Error: Unknown command '{command}'")
            return {"status": "error", "message": "Unknown command"}
    except serial.SerialException as e:
        print(f"Error connecting to serial port {SERIAL_PORT}: {e}")
        return {"status": "error", "message": str(e)}
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return {"status": "error", "message": str(e)}

@app.route('/move/<string:direction>', methods=['POST'])
def move_chair(direction):
    """API endpoint to move the chair."""
    result = send_command(direction.lower())
    return jsonify(result)

if __name__ == '__main__':
    # Runs the Flask server
    # host='0.0.0.0' makes the server accessible from other computers on your network
    app.run(host='0.0.0.0', port=5000, debug=True)
