import cv2
import numpy as np
import serial
import io
from PIL import Image

# Open a serial connection
ser = serial.Serial('COM11', 115200)  # Change 'COM1' to your serial port

# Function to read MJPEG frames from serial
def read_mjpeg_frame(serial_connection):
    image_data = b''
    while True:
        # Read byte by byte until we find the start of the JPEG marker
        char = serial_connection.read(1)
        if char == b'\xff':
            # Check for the next byte to identify the start of the frame
            next_char = serial_connection.read(1)
            if next_char == b'\xd8':
                image_data += b'\xff\xd8'
                while True:
                    image_data += serial_connection.read(1)
                    if image_data[-2:] == b'\xff\xd9':
                        try:
                            # Attempt to decode the JPEG data
                            cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), 1)
                            return image_data
                        except cv2.error as e:
                            print(f"Error decoding frame: {e}")
                            image_data = b''  # Reset image_data if decoding fails
                            break
        # return None
# Display the MJPEG stream
def display_mjpeg_stream():
    while True:
        # Read MJPEG frame
        frame_data = read_mjpeg_frame(ser)

        # Convert the binary data to an OpenCV image
        frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), 1)

        # Display the frame
        cv2.imshow('MJPEG Stream', frame)

        # Increase the delay to slow down the frame rate (10 milliseconds in this example)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    display_mjpeg_stream()
    # Release resources
    ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    display_mjpeg_stream()
