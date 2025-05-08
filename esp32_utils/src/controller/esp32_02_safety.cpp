// #include <Wire.h>
// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include <MPU6050_light.h>
// #include <vector>
// #include <sstream>
// #include "serial_utils/serial_utils.h"
// #include "Placa.h"
// #include <Adafruit_NeoPixel.h>

// // Servos declaration and initialization
// Servo servos[8];
// int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};

// // Counter for managing messages frequency
// int loop_counter = 0;

// // IMU declaration
// MPU6050 mpu(Wire);

// // IMU timer
// long timer = 0;
// volatile bool i2c_busy = false;

// // Emergency button state variable
// bool emergency_button_state = 0;


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

//     // Sensor initialization
//     byte status = mpu.begin();
//     while(status!=0) {}
    
//     // Sensor calibration
//     delay(1000);
//     mpu.calcOffsets(true,true);
    
// }

// bool state = false;
 
// void loop()
// {

//     // === Send IMU message ===

//     //Update IMU data
//     if (!i2c_busy) { i2c_busy = true; mpu.update(); i2c_busy = false; }

//     // Get angle Z
//     float angle_z  = mpu.getAngleZ();
    
//     // Format angle value into a message string
//     char imu_msg[50]; snprintf(imu_msg, sizeof(imu_msg), "IMU,%f", angle_z);

//     // Send the message only every 12 loops to reduce communication overhead
//     loop_counter ++;
//     if (loop_counter % 15 == 0) {   
//         sendMessage(imu_msg);
//     }

//     servos[0].write(50);
//     servos[2].write(140);


//     // === Read message ===

//     std::string message = readMessage().c_str();
//     char end_action[50];

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

//         // Process command pairs
//         for (int i = 0; i < message_parts.size(); i += 2) {
//             std::string element_id = message_parts[i];
//             int value = stoi(message_parts[i+1]);

//             if (element_id == "Emergency") {
//                 emergency_button_state = value;
//                 if (value == 1) {
//                     servos[i].detach();
//                     emergency_button_state = value;
//                 }
//                 else if (value == 0) {
//                     for (int i = 0; i < 8; i++) {servos[i].attach(servoPins[i]);}
//                     emergency_button_state = value;
//                 }
//             }

//             if (emergency_button_state == 0) {
                
//                 if (element_id == "AP") {
//                     while (i2c_busy);  // Espera que el bus estigui lliure
//                     i2c_busy = true;

//                     Wire.beginTransmission(0x20);
//                     Wire.write(1);
    
//                     if(value == 1){ Wire.write(0b11111111); } // Turn all outputs ON.
//                     else { Wire.write(0b00000000); } // Turn all outputs OFF.
    
//                     Wire.endTransmission();

//                     i2c_busy = false;
//                 }
                
//                 else if (element_id == "S01") { servos[6].write(value == 0 ? 162 : 100); } // PINCER
//                 else if (element_id == "S02") { servos[1].write(value == 0 ? 130 : 165); } // LEFT SUCTION GRIPPER
//                 else if (element_id == "S03") { servos[4].write(value == 0 ? 45 : 12); }   // RIGHT SUCTION GRIPPER
//                 else if (element_id == "S04") { 
//                     if (value == 0) servos[0].write(70); 
//                     if (value == 1) servos[0].write(140); 
//                     if (value == 2) servos[0].write(50); 
//                 }  // LEFT SHOVEL
//                 else if (element_id == "S05") { 
//                     if (value == 0) servos[2].write(120); 
//                     if (value == 1) servos[2].write(40); 
//                     if (value == 2) servos[2].write(140);
//                 }  // RIGHT SHOVEL
//             }
//             }

        

//         snprintf(end_action, sizeof(end_action),"EA,1");
//         sendMessage(end_action);
//     }

//     // Tancat
//     // servos[0].write(0); 
//     // servos[2].write(180);
//     // servos[1].write(165);
//     // servos[4].write(12); 

//     // Obert
//     servos[0].write(140); 
//     servos[2].write(40);
//     servos[1].write(130);
//     servos[4].write(45);



//     delay(20);
// }
