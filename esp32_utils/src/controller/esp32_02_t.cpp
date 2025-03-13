// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include "Placa.h"
// #include "serial_utils/serial_utils.h"

// Servo servos[5];
// int servoPins[] = {GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};
 
// void setup()
// {
//     setupSerial();
//     Serial.begin(115200);

//     // Attach each servo to the pin
//     for (int i = 0; i < 5; i++) {
//         servos[i].attach(servoPins[i]);
//     }
// }
 
 
// void loop()
// {
//     // Read message

//     String message = readMessage();

//     /// Get each value of the message and assign action
//     if (message.length() > 0) {
//         int firstComma = message.indexOf(',');
//         int secondComma = message.indexOf(',', firstComma + 1);
//         if (firstComma > 0 && secondComma > firstComma) {
//             String servoStr    = message.substring(0, firstComma);
//             String servoPosStr = message.substring(firstComma + 1, secondComma);
            
//             int servo = servoStr.substring(1).toInt() - 4;
//             int pos   = servoPosStr.toInt(); 

//             Serial.println(servo);
//             Serial.println(pos);

//             servos[servo].write(pos);
//             delay(20);
//         }
//     }
// }