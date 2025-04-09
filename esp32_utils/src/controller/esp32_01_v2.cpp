#include <Arduino.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"
#include "pid/ESP32PIDMotor.hpp"
#include <vector>
#include <sstream>

ESP32Encoder encoderC;
ESP32Encoder encoderD;

// PWM channels for each motor between 0 and 15
const int ledChannelC = 2;
const int ledChannelD = 3;

// PWM configuration
const int frequency = 5000;  // Frequency Hz
const int resolution = 8;    // Resolution in bits (from 1 to 15)

// Counter for managing messages frequency
int loop_counter;



void motor_right(int speed, bool dir) {
    /*
      Controls the right motor.
    
      Arguments:
        speed (int): PWM value to control the motor speed (0–255).
        dir (bool): Direction of rotation: true (1) = forward; false (0) = backward.
    */

    int real_speed = speed;
    if (dir == 1) { real_speed = 255 - real_speed; }

    digitalWrite(GPIO_INC1, dir);
    ledcWrite(ledChannelC, real_speed);
}

void motor_left(int speed, bool dir) {
    /*
      Controls the left motor.
    
      Arguments:
        speed (int): PWM value to control the motor speed (0–255).
        dir (bool): Direction of rotation: true (1) = backward; false (0) = forward.
    */

    int real_speed = speed;
    if (dir == 1) { real_speed = 255 - real_speed; }

    digitalWrite(GPIO_IND1,dir);
    ledcWrite(ledChannelD, real_speed);
}



void setup() {
    setupSerial(); 

    // Set up PWM channels for motor control
    ledcSetup(ledChannelC, frequency, resolution);
    ledcAttachPin(GPIO_INC2, ledChannelC);

    ledcSetup(ledChannelD, frequency, resolution);
    ledcAttachPin(GPIO_IND2, ledChannelD);


    // Set motor direction pins as outputs
    pinMode(GPIO_INC1, OUTPUT);
    pinMode(GPIO_IND1, OUTPUT);


    // Enable motors (active low)
    pinMode(GPIO_DISABLE_MOTORS, OUTPUT);
    digitalWrite(GPIO_DISABLE_MOTORS, 0);


    // Attach and configure motor encoders
    encoderC.attachFullQuad(GPIO_ENCC1,GPIO_ENCC2);
    encoderC.setFilter(1023);
    encoderC.setCount(0);

    encoderD.attachFullQuad(GPIO_ENCD1,GPIO_ENCD2);
    encoderD.setFilter(1023);
    encoderD.setCount(0);
}

void loop() {

    // === Send message ===

    // Get encoder counts from both motors
    int64_t left_encoder_val  = encoderC.getCount();
    int64_t right_encoder_val = encoderD.getCount();
    
    // Format encoder values into a message string (in centimeters)
    char encoders_msg[50]; snprintf(
        encoders_msg, sizeof(encoders_msg),
        "EL,%ld,ER,%ld",
        (int)(left_encoder_val / MOTOR_POLSOS_PER_CM),
        (int)(right_encoder_val / MOTOR_POLSOS_PER_CM)
    );

    // Send the message only every 12 loops to reduce communication overhead
    loop_counter ++;
    if (loop_counter % 12 == 0) {   
        sendMessage(encoders_msg);
    }


    // === Read message ===

    std::string message = readMessage().c_str();

    // Get each value of the message and assign motors power
    if (!message.empty()) {

        std::vector<std::string> message_parts;
        std::stringstream ss(message);
        std::string item;

        // Split the message by commas and store parts
        while (std::getline(ss, item, ',')) {
            message_parts.push_back(item);
        }

        // Remove the last part as it's a checksum
        if (!message_parts.empty()) {
            message_parts.pop_back();
        }

        // Process command pairs
        for (int i = 0; i < message_parts.size(); i += 2) {
            std::string motor = message_parts[i];
            int power = stoi(message_parts[i+1]);
            int dir   = 0;

            if (motor == "M01") {
                // Handle motor M01
            } else if (motor == "M02") {
                // Handle motor M02
            } else if (motor == "ML") {
                if (power < 0) { dir = 1; }
                motor_left(abs(power), dir);
            } else if (motor == "MR") {
                if (power > 0) { dir = 1; }
                motor_right(abs(power), dir);
            }
        }
    }

    delay(20); // Receive 100 messages per second
}