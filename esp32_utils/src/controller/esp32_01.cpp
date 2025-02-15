#include <Arduino.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"
#include "pid/ESP32PIDMotor.hpp"
#include "ESP32Encoder.h"
#include "pid/PID_DIG_ESP.hpp"




void setup() {
    Serial.begin(115200);
    setupSerial();

    motor_encoder.attachFullQuad(motor_parameters.gpio_enc_a, motor_parameters.gpio_enc_b);
    motor_encoder.setFilter(1023);
    motor_encoder.clearCount();
}

void loop() {

    int64_t pulses = motor_encoder.getCount();

    void sendMessage(String message); 

    String message = readMessage();
    if (message.length() > 0) {
        Serial.println(message);
    }


}