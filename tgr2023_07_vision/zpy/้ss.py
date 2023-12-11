import serial
import io
from PIL import Image
import numpy as np

# Open a serial connection
ser = serial.Serial('COM4', 115200)  # Change 'COM11' to your ESP32 serial port

def receive_data_and_display():
    while True:
        # Read data from serial
        data = ser.readline().strip()

        # Check if the data is an image
        if data.startswith(b'\xff\xd8'):
            try:
                # Decode the JPEG data
                image = Image.open(io.BytesIO(data))
                # Display the image
                image.show()
            except Exception as e:
                print(f"Error decoding image: {e}")
        else:
            # If the data is not an image, print it as a log
            print(f"Received log: {data.decode('utf-8')}")

if __name__ == "__main__":
    try:
        receive_data_and_display()
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        ser.close()