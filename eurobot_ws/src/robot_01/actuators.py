from utils.scripts.serial_communication import open_serial_port, send_message

port = '/dev/ttyUSB0'
baudrate = 115200
serial_port = open_serial_port(port, baudrate)

send_message(serial_port, "S06,0")