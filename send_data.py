from usb import core
import time
import serial

# Find the ESP32 device by vendor and product ID (adjust the ID if necessary)
# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Check if the serial port is open
if not ser.is_open:
    raise ValueError("Serial port not open")
def bounce_data():
    while True:
        try:
            # Read data from the ESP32
            data = ser.read(512)
            print(f"Received: {data}")

            # Send the same data back to the ESP32
            ser.write(data)
            print(f"Sent back: {data}")

            time.sleep(1)  # Delay for a bit before reading/sending again
        except serial.SerialException as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    bounce_data()
