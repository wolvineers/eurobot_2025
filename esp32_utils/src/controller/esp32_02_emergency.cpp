// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include "Placa.h"
// #include "serial_utils/serial_utils.h"

// Servo servos[8];
// int servoPins[] = {
//   GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4,
//   GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8
// };

// void setup() {
//     setupSerial();

//     for (int i = 0; i < 8; i++) {
//         servos[i].attach(servoPins[i]);
//     }
// }

// void loop() {
//     String message = readMessage();

//     // servos[0].write(90);
//     // delay(1000);
//     // servos[0].write(140);
//     // delay(1000);

//     if (message.length() > 0) {
//         int commaIndex = message.indexOf(',');
//         if (commaIndex != -1) {
//             String command = message.substring(0, commaIndex);
//             String checksum = message.substring(commaIndex + 1);

//             if (command == "STOP") {
//                 for (int i = 0; i < 8; i++) {
//                     servos[i].detach();
//                 }
//             }
//         }
//     }
// }
