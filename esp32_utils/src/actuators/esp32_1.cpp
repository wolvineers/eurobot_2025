// #include <Arduino.h>
// #include "Placa.h"
// #include "serial_utils/serial_utils.h"
// #include "pid/ESP32PIDMotor.hpp"
// ESP32Encoder encoderC;
// ESP32Encoder encoderD;

// bool F1_horizontal;
// bool F2_horizontal;
// const int ledChannelA = 0;    // Canal PWM, puede ser de 0 a 15
// const int ledChannelB = 1;    // Canal PWM, puede ser de 0 a 15
// const int ledChannelC = 2;    // Canal PWM, puede ser de 0 a 15
// const int ledChannelD = 3;    // Canal PWM, puede ser de 0 a 15
// const int frequency = 5000;  // Frecuencia en Hz
// const int resolution = 8;    // Resolución en bits (de 1 a 15)


// /**********************funcions de motors funcionen amb un numero de 0-255 al speed i 0 o 1 al dir (direccio) */
// void motA(int speed, bool dir){      // motor horizontal 0-fora 1-dins
//     if (dir == 0 and F1_horizontal == false){
//         digitalWrite(GPIO_INA1,1);
//         ledcWrite(ledChannelA, speed);
//         printf("PUM1\n");
//     }else if (dir == 1 and F2_horizontal == 0){
//         digitalWrite(GPIO_INA1,0);
//         ledcWrite(ledChannelA, speed);
//         printf("PUM2\n"); 
//     }else{
//         digitalWrite(GPIO_INA1,dir);
//         ledcWrite(ledChannelA, speed);  
//     }

// }
// void motB(int speed, bool dir){      // motor lift
//     digitalWrite(GPIO_INB1,dir);
//     ledcWrite(ledChannelB, speed);
// }
// void motC(int speed, bool dir){      // motor roda
//     digitalWrite(GPIO_INC1,dir);
//     ledcWrite(ledChannelC, speed);
// }
// void motD(int speed, bool dir){      // motor roda
//     digitalWrite(GPIO_IND1,dir);
//     ledcWrite(ledChannelD, speed);
// }



// void setup() {
    
//     Serial.begin(115200);
    
//     ledcSetup(ledChannelA, frequency, resolution);
//     ledcAttachPin(GPIO_INA2, ledChannelA);
//     ledcSetup(ledChannelB, frequency, resolution);
//     ledcAttachPin(GPIO_INB2, ledChannelB);
//     ledcSetup(ledChannelC, frequency, resolution);
//     ledcAttachPin(GPIO_INC2, ledChannelC);
//     ledcSetup(ledChannelD, frequency, resolution);
//     ledcAttachPin(GPIO_IND2, ledChannelD);
//     pinMode(GPIO_DISABLE_MOTORS, OUTPUT);  //EN motors
//     digitalWrite(GPIO_DISABLE_MOTORS, 0);

//     pinMode(GPIO_ENCA1, INPUT_PULLUP);        // encoder A i B son finals de carrera
//     pinMode(GPIO_ENCA2, INPUT_PULLUP);
//     pinMode(GPIO_ENCB1, INPUT_PULLUP);
//     pinMode(GPIO_ENCB2, INPUT_PULLUP);
//     pinMode(GPIO_INA1, OUTPUT);               // encoder C i D son encoders
//     pinMode(GPIO_INB1, OUTPUT);
//     pinMode(GPIO_INC1, OUTPUT);
//     pinMode(GPIO_IND1, OUTPUT);

//     encoderC.attachFullQuad(GPIO_ENCC1,GPIO_ENCC2);
//     encoderC.setFilter(1023);
//     encoderC.setCount(0);

//     encoderD.attachFullQuad(GPIO_ENCD1,GPIO_ENCD2);
//     encoderD.setFilter(1023);
//     encoderD.setCount(0);
    
// }

// void loop() {


//     int64_t encoderValueC = encoderC.getCount();                   // variable amb valor de encoder C
//     //printf("pulses: %lli\n" , encoderValueC);                      //debug
//     int64_t encoderValueD = encoderD.getCount();                   // variable amb valor de encoder D
//     //printf("pulses: %lli\n" , encoderValueD);                      //debug
//     F1_horizontal = digitalRead(GPIO_ENCA1);                  // variable amb valor de final carrera lift horizontal pinces fora
//     F2_horizontal = digitalRead(GPIO_ENCA2);                  // variable amb valor de final carrera lift horizontal pinces dins
//     bool F1_lift = digitalRead(GPIO_ENCB1);                        // variable amb valor de final carrera lift
//     bool F2_lift = digitalRead(GPIO_ENCB2);                        // variable amb valor de final carrera lift
    
    
    
// }