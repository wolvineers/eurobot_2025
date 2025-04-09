from utils.scripts.serial_communication import open_serial_port, read_message
import time

def main():
    # Configure the serial port
    port = '/dev/ttyUSB0'
    baudrate = 115200

    # Open the serial port using the specified port and baud rate
    serial_port = open_serial_port(port, baudrate)

    # Check if the serial port was successfully opened
    if not serial_port:
        print("Could not open serial port.")
        return

    # Read messages from the opened serial port
    read_message(serial_port)

if __name__ == "__main__":
    main()
