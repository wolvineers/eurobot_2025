from utils.scripts.serial_communication import open_serial_port, send_message
import time

port = '/dev/ttyUSB0'
baudrate = 115200
serial_port = open_serial_port(port, baudrate)


while True:
    send_message(serial_port, "S3,1,S04,1")
    time.sleep(3)
    send_message(serial_port, "S3,0,S04,0")
    time.sleep(3)