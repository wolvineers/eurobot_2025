#include <Arduino.h>
#include <ESP32Servo.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"


Servo myServo;
int servoPin = GPIO_SERVO1;

void setup() {
    setupSerial();
    Serial.begin(115200);
    myServo.attach(servoPin);
}

void loop() {
    int pos = 50;
    myServo.write(pos);

    String message = readMessage();

    if (message != "") {
        if (verifyChecksum(message)) Serial.print(message);
        else sendMessage("RSError");
    }

}