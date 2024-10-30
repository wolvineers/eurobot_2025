from utils.src.serial_communication import open_serial_port, send_message, read_message
import time

def main():
    # Configure the serial port
    port = '/dev/ttyUSB0'
    baudrate = 9600

    # Open the serial port using the specified port and baud rate
    serial_port = open_serial_port(port, baudrate)

    # Check if the serial port was successfully opened
    if not serial_port:
        print("Could not open serial port.")
        return

    for count_msg in range(10):
        # Send messages to the opened serial port
        start_time = time.time()

        message = "M," + str(count_msg)
        send_message(serial_port, message)
        read_message(serial_port)

        elapsed_time = time.time() - start_time
        print("Ping: " + str(elapsed_time*1000*1000) + "us")

if __name__ == "__main__":
    main()
