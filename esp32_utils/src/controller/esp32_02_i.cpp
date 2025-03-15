// #include <Wire.h>
// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include "Placa.h"
// #include "pid/ESP32PIDMotor.hpp"
// #include "serial_utils/serial_utils.h"

// Servo servos[5];
// int servoPins[] = {GPIO_SERVO3, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};
// int loop_counter;
 
// void setup()
// {
//     setupSerial();
//     Serial.begin(115200);

//     // Attach each servo to the pin
//     for (int i = 0; i < 5; i++) {
//         servos[i].attach(servoPins[i]);
//     }

//     // I2C setup
//     Wire.setPins(21, 22);
//     Wire.begin();

//     loop_counter = 0;

// }
 
 
// void loop()
// {
//     // Read limit switch state

//     loop_counter ++;

//     Wire.beginTransmission(0x20);
//     Wire.write(0);
//     Wire.endTransmission();

//     Wire.requestFrom(0x20, 1);
//     uint8_t val = Wire.read();

//     if (val & (1 << 1)) {
//         // Parar motors
//         // Serial.println("HOLA");
//         if (loop_counter % 25 == 0) {
//             sendMessage("EA,1"); // EA = End action; 0 = false; 1 = true
//         }
//     }


//     // Read message

//     String message = readMessage();

//     /// Get each value of the message and assign action
//     if (message.length() > 0) {
//         int firstComma = message.indexOf(',');
//         int secondComma = message.indexOf(',', firstComma + 1);
//         if (firstComma > 0 && secondComma > firstComma) {
//             String actuatorStr    = message.substring(0, firstComma);
//             String actuatorValStr = message.substring(firstComma + 1, secondComma);

//             char actuator = actuatorStr.charAt(0);
//             if (actuator == 'M') {

//                 Serial.println("Move motors");

//                 int motor = actuatorStr.substring(1).toInt();
//                 int dir   = actuatorValStr.toInt();

//                 if (motor == 1) {
//                     // Move motor 1
//                 } else if (motor == 2) {
//                     // Move motor 2
//                 }



//             } else if (actuator == 'S') {

//                 int servo = actuatorStr.substring(1).toInt() - 4;
//                 int pos   = actuatorValStr.toInt();
                
//                 servos[servo].write(pos);
//                 delay(20);

//             } else if (actuator == 'V') {

//             }
//         }
//     }

//     delay(20);
// }