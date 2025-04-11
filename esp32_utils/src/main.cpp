#include <Arduino.h>
#include <ESP32Servo.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"

Servo servos[8];
int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};

void setup() {
    setupSerial();
    Serial.begin(115200);
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);
    }
}

void loop() {
    String message = readMessage();

    if (message.length() > 0) {
        int partsCount = 0;
        String parts[10];
        int startIndex = 0;
        int commaIndex = message.indexOf(',');

        Serial.println(message);

        while (commaIndex != -1 && partsCount < 10) {
            parts[partsCount++] = message.substring(startIndex, commaIndex);
            startIndex = commaIndex + 1;
            commaIndex = message.indexOf(',', startIndex);
        }

        if (startIndex < message.length() && partsCount < 10) {
            parts[partsCount++] = message.substring(startIndex);
        }

        if (partsCount >= 1 && partsCount % 2 == 1) {
            String checksumStr = parts[partsCount - 1];
            int checksum = checksumStr.toInt();

            for (int i = 0; i < partsCount - 1; i += 2) {
                String servoIdStr = parts[i];
                String value = parts[i + 1];

                if (servoIdStr.startsWith("S")) {
                    int servoNumber = servoIdStr.substring(1).toInt() - 1;

                    if (servoNumber + 1 == 1) {     // VENTOSA ESQUERRA
                        servos[servoNumber].write(value == "0" ? 120 : 180);
                    }
                    else if (servoNumber + 1 == 2) { // PINÇA
                        servos[servoNumber].write(value == "0" ? 0 : 65);
                    }
                    else if (servoNumber + 1 == 3) { // PALA DRETA
                        servos[servoNumber].write(value == "0" ? 120 : 0);
                    }
                    else if (servoNumber + 1 == 4) { // PALA ESQUERRA
                        servos[servoNumber].write(value == "0" ? 0 : 120);
                    }
                    else if (servoNumber + 1 == 8) {
                        servos[servoNumber].write(value == "0" ? 180 : 0);
                    }
                    else {
                        Serial.println("Error: Invalid servo Number " + servoIdStr);
                    }
                }
            }

        } else {
            Serial.println("Error: Invalid message format");
        }
    }
}
