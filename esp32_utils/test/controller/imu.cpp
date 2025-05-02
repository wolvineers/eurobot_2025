#include <Wire.h>
#include <MPU6050_light.h>
#include <Arduino.h>

MPU6050 mpu(Wire);

long timer = 0;
 
void setup()
{
    Serial.begin(115200);

    // I2C setup
    Wire.setPins(21, 22);
    Wire.begin();

    // Sensor initialization
    byte status = mpu.begin();
    while(status!=0) {}
    
    // Sensor calibration
    delay(1000);
    mpu.calcOffsets(true,true); // Set the 0 angle (do it whenever to recalibrate the sensor)
}
 
 
void loop()
{
    // Update sensor data
    mpu.update();

    // Show data every second
    if(millis() - timer > 1000){        
        Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
        Serial.print("\tY: ");Serial.print(mpu.getAngleY());
        Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
        Serial.println(F("=====================================================\n"));
        timer = millis();
    }
}