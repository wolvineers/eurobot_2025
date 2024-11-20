#include <Arduino.h>
#include <ESP32Servo.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"

Servo servos[4];
int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4};

void setup() {
    setupSerial();
    Serial.begin(115200);
    for (int i = 0; i < 4; i++) {
        servos[i].attach(servoPins[i]);
    }
}

void loop() {
    String message = readMessage();
    if (message.length() > 0) {
        int firstComma = message.indexOf(',');
        int secondComma = message.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
            String servoNumberStr = message.substring(2, firstComma);
            String servoAngleStr = message.substring(firstComma + 1, secondComma);
            String checksumStr = message.substring(secondComma + 1);
            int servoNumber = servoNumberStr.toInt() - 1;
            int servoAngle = servoAngleStr.toInt();
            int checksum = checksumStr.toInt();
            if (servoNumber >= 0 && servoNumber < 4) {
                servos[servoNumber].write(servoAngle);
                Serial.println("Servo Number: " + String(servoNumber + 1));
                Serial.println("Servo Angle: " + String(servoAngle));
                Serial.println("Checksum: " + String(checksum));
            } else {
                Serial.println("Error: Número de servo inválido");
            }
        } else {
            Serial.println("Error: Formato de mensaje inválido");
        }
    }
}
