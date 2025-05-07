// #include <Wire.h>
// #include <MPU6050_light.h>

// MPU6050 mpu(Wire);

// unsigned long last_check_time = 0;
// float last_angle_z = 0;
// const float MAX_ANGLE_CHANGE = 3.0;  
// const unsigned long CHECK_INTERVAL = 5000;

// void setup() {
//   Serial.begin(115200);
//   Wire.setPins(21, 22);  // Configura els pins I2C
//   Wire.begin();
  
//   byte status = mpu.begin();
//   while (status != 0) {
//     Serial.println("Error iniciant IMU...");
//     delay(1000);
//     status = mpu.begin();
//   }

//   delay(1000);
//   mpu.calcOffsets(true, true);  // Calibrar el sensor
//   Serial.println("IMU llesta.");
  
//   last_angle_z = 0;
//   last_check_time = millis();
// }

// void loop() {
//   mpu.update();

//   Serial.print(" | Z: ");
//   Serial.println(mpu.getAngleZ());  // Imprimir valor Z en temps real

//   float angle_z = mpu.getAngleZ();

//   unsigned long current_time = millis();
//   if (current_time - last_check_time > CHECK_INTERVAL) {
//     float angle_change = fabs(angle_z - last_angle_z);

//     if (angle_change > MAX_ANGLE_CHANGE) {
//       // Si el canvi supera el límit, reiniciem la IMU a 0
//       Serial.println("Canvi brusc detectat. Reiniciant IMU a 0...");
      
//       // Recalibrar la IMU per restablir els offsets
//       mpu.calcOffsets(true, true);

      
//       Serial.println(mpu.getAngleZ());
      

//       Serial.println("IMU reiniciada a 0.");
//     } else {
//       Serial.println("IMU estable.");
//     }

//     last_angle_z = angle_z;
//     last_check_time = current_time;
//   }

//   delay(100);  // Redueix una mica la freqüència d'impressió
// }
