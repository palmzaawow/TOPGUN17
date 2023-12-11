import cv2
import numpy as np
import serial

# Open a serial connection
ser = serial.Serial('COM5', 115200)  # Change 'COM1' to your serial port

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
                            image_array = np.frombuffer(image_data, dtype=np.uint8)
                            frame = cv2.imdecode(image_array, 1)
                            if frame is not None:
                                return image_data, frame
                            else:
                                print("Received data is not a valid image.")
                                return image_data, None
                        except cv2.error as e:
                            print(f"Error decoding frame: {e}")
                            image_data = b''  # Reset image_data if decoding fails
                            break

# Display the MJPEG stream
def display_mjpeg_stream():
    while True:
        # Read MJPEG frame
        frame_data, frame = read_mjpeg_frame(ser)

        # Check if the frame is not None (i.e., a valid image)
        if frame is not None:
            # Display the frame
            cv2.imshow('Streamimg by Palmzaawow', frame)

        # If the frame is None, you can handle or print the raw data
        else:
            print("Received data:", frame_data)

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
