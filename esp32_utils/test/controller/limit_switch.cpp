#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include "Placa.h"

bool sentit_sortida = false;
Servo servos[8];
int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};
 
void setup()
{
    Wire.setPins(21, 22);
    //Wire.setClock()
    Wire.begin();

    Serial.begin(115200);

    // Posem els pins de sortida a 0
    Wire.beginTransmission(0x20);
    Wire.write(1);
    Wire.write(0b00111111);
    Wire.endTransmission();

    // Config registre config (pins entrada o sortida)
    Wire.beginTransmission(0x20);
    Wire.write(3);
    Wire.write(0b00111111);
    Wire.endTransmission();

    // Attach each servo to the pin
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);
    }
}
 
 
void loop()
{
    Wire.beginTransmission(0x20);
    Wire.write(0);
    Wire.endTransmission();

    Wire.requestFrom(0x20, 1);
    uint8_t val = Wire.read();

    printf("Final carrera 1: %s\n", (val & (1 << 1)) ? "Apretat" : "No apretat");
    printf("Final carrera 2: %s\n", (val & (1 << 5)) ? "Apretat" : "No apretat");
    //Serial.println(val);

    // Canviem el valor dels pins de sortida
    Wire.beginTransmission(0x20);
    Wire.write(1);
    if(sentit_sortida){
        Wire.write(0b00111111);
    } else{
        Wire.write(0b11111111);
    }
    sentit_sortida = !sentit_sortida;
    Wire.endTransmission();

    delay(1000);
}