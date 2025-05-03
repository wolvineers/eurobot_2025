from utils.scripts.serial_communication import open_serial_port, send_message
def move(distance, power):
   port = '/dev/ttyUSB0'
   baudrate = 115200


   serial_port = open_serial_port(port, baudrate)


   if not serial_port:
       print("Could not open serial port.")
       return


   print(f"Distance to move {distance} cm with {power} power")


   send_message(serial_port, f"D:{distance},P:{power}")


move(100, 50)