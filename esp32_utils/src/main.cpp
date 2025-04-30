#include <Wire.h>               // Library for I2C communication.
#include <Arduino.h>            // Core Arduino functions and definitions.
#include <ESP32Servo.h>         // Library to control servos using ESP32.
#include "serial_utils/serial_utils.h"  // Custom library for handling serial messages.
#include "Placa.h"              // Configuration file that defines pin mappings like GPIO_SERVO1, etc.

Servo servos[8];                // Array of Servo objects to control up to 8 servo motors.
int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};
// Array that stores the pin numbers assigned to each servo.

void setup()
{
    Wire.setPins(21, 22);        // Set the I2C SDA and SCL pins.
    // Wire.setClock()           // Commented line, could be used to set I2C bus speed.
    Wire.begin();               // Initialize I2C communication.

    Serial.begin(115200);       // Start serial communication at 115200 baud rate.

    // Setup for an I2C GPIO expander (probably MCP23017 at address 0x20):
    Wire.beginTransmission(0x20);
    Wire.write(3);              // Register address for I/O configuration.
    Wire.write(0b11111100);     // Set the first 6 pins as outputs.
    Wire.endTransmission();

    Wire.beginTransmission(0x20);
    Wire.write(1);              // Register address for output state.
    Wire.write(0b11111100);     // Initialize outputs to "off" state.
    Wire.endTransmission();

    // Attach each Servo object to its corresponding pin.
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);
    }
}
 
void loop()
{
    String message = readMessage();   // Reads an incoming message from serial.

    if (message.length() > 0) {       // If a message was received...
        int partsCount = 0;
        String parts[10];             // Array to hold split parts of the message.
        int startIndex = 0;
        int commaIndex = message.indexOf(',');  // Look for the first comma.

        // Split the message into parts based on commas.
        while (commaIndex != -1 && partsCount < 10) {
            parts[partsCount++] = message.substring(startIndex, commaIndex);
            startIndex = commaIndex + 1;
            commaIndex = message.indexOf(',', startIndex);
        }

        // Store the last part, if it exists.
        if (startIndex < message.length() && partsCount < 10) {
            parts[partsCount++] = message.substring(startIndex);
        }

        // Validate the message: should have an odd number of elements (key-value pairs + checksum).
        if (partsCount >= 1 && partsCount % 2 == 1) {
            String checksumStr = parts[partsCount - 1];  // Last element is the checksum.
            int checksum = checksumStr.toInt();         // Convert checksum to integer (though it's not used further).

            // Process each key-value pair.
            for (int i = 0; i < partsCount - 1; i += 2) {
                String elementId = parts[i];            // Device identifier.
                String value = parts[i + 1];            // Associated value.

                // Digital output control via I2C.
                if (elementId == "AP") {
                    Wire.beginTransmission(0x20);
                    Wire.write(1);
                    if(value == "1"){
                        Wire.write(0b11111111);         // Turn all outputs ON.
                    } else {
                        Wire.write(0b00000000);         // Turn all outputs OFF.
                    }
                    Wire.endTransmission();
                }

                // Control individual servos based on received ID.
                if (elementId == "S01") {               // LEFT SUCTION GRIPPER
                    servos[0].write(value == "0" ? 120 : 180);
                }
                else if (elementId == "S02") {          // PINCER
                    servos[1].write(value == "0" ? 0 : 65);
                }
                else if (elementId == "S03") {          // RIGHT SHOVEL
                    servos[2].write(value == "0" ? 120 : 0);
                }
                else if (elementId == "S04") {          // LEFT SHOVEL
                    servos[3].write(value == "0" ? 0 : 120);
                }
                else if (elementId == "S05") {          // Another servo-controlled actuator.
                    servos[4].write(value == "0" ? 0 : 90);
                }
            }

        } else {
            Serial.println("Error: Invalid message format");  // If the message is malformed, print an error.
        }
    }
}
