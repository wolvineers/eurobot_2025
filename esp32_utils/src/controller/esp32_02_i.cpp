// #include <Wire.h>
// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include "serial_utils/serial_utils.h"
// #include "Placa.h"
// #include <vector>
// #include <sstream>

// // Servos declaration and initialization
// Servo servos[8];
// int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};

// // Counter for managing messages frequency
// int loop_counter = 0;

// void setup()
// {
//     Wire.setPins(21, 22);        // Set the I2C SDA and SCL pins
//     Wire.begin();                // Initialize I2C communication

//     setupSerial();

//     // Setup for an I2C GPIO expander
//     Wire.beginTransmission(0x20);
//     Wire.write(3);              // Register address for I/O configuration
//     Wire.write(0b11111100);     // Set the last 2 pins as outputs
//     Wire.endTransmission();

//     Wire.beginTransmission(0x20);
//     Wire.write(1);              // Register address for output state
//     Wire.write(0b11111100);     // Initialize outputs to "off" state
//     Wire.endTransmission();

//     // Attach each servo object to its corresponding pin
//     for (int i = 0; i < 8; i++) {
//         servos[i].attach(servoPins[i]);
//     }

    
// }
 
// void loop()
// {

//     std::string message = readMessage().c_str();
//     char encoders_msg[50];


//     // Get each value of the message and assign motors power
//     if (!message.empty()) {

//         std::vector<std::string> message_parts;
//         std::stringstream ss(message);
//         std::string item;

//         // Split the message by commas and store parts
//         while (std::getline(ss, item, ',')) {
//             message_parts.push_back(item);
//         }

//         // Remove the last part as it's a checksum
//         if (!message_parts.empty()) {
//             message_parts.pop_back();
//         }

//         // // Process command pairs
//         for (int i = 0; i < message_parts.size(); i += 2) {
//             std::string element_id = message_parts[i];
//             int value = stoi(message_parts[i+1]);

//             if (element_id == "AP") {
//                 Wire.beginTransmission(0x20);
//                 Wire.write(1);

//                 if(value == 1){ Wire.write(0b11111111); } // Turn all outputs ON.
//                 else { Wire.write(0b00000000); } // Turn all outputs OFF.

//                 Wire.endTransmission();
//             }
            
//             else if (element_id == "S01") { servos[0].write(value == 0 ? 120 : 180); } // LEFT SUCTION GRIPPER
//             else if (element_id == "S02") { servos[1].write(value == 0 ? 0 : 65); }    // PINCER
//             else if (element_id == "S03") { servos[2].write(value == 0 ? 120 : 0); }   // RIGHT SHOVEL
//             else if (element_id == "S04") { servos[3].write(value == 0 ? 0 : 120); }   // LEFT SHOVEL
//             else if (element_id == "S05") { servos[4].write(value == 0 ? 0 : 90); }    // RIGHT SUCTION GRIPPER.
//         }


//         snprintf(encoders_msg, sizeof(encoders_msg),"EA,1");
//         sendMessage(encoders_msg);

//     } else {
//         loop_counter ++;

//         if (loop_counter % 100 == 0) {   
//             snprintf(encoders_msg, sizeof(encoders_msg), "EA,0");
//             sendMessage(encoders_msg);
//         }
//     }

//     delay(20);
// }
