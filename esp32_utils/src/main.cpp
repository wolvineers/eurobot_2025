<<<<<<< HEAD
#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU6050_light.h>
#include <vector>
#include <sstream>
#include "serial_utils/serial_utils.h"
#include "Placa.h"

// Servos declaration and initialization
Servo servos[8];
int servoPins[] = {GPIO_SERVO1, GPIO_SERVO2, GPIO_SERVO3, GPIO_SERVO4, GPIO_SERVO5, GPIO_SERVO6, GPIO_SERVO7, GPIO_SERVO8};

// Counter for managing messages frequency
int loop_counter = 0;

// IMU declaration
MPU6050 mpu(Wire);

// IMU timer
long timer = 0;

void setup()
{
    Wire.setPins(21, 22);        // Set the I2C SDA and SCL pins
    Wire.begin();                // Initialize I2C communication

    setupSerial();

    // Setup for an I2C GPIO expander
    Wire.beginTransmission(0x20);
    Wire.write(3);              // Register address for I/O configuration
    Wire.write(0b11111100);     // Set the last 2 pins as outputs
    Wire.endTransmission();

    Wire.beginTransmission(0x20);
    Wire.write(1);              // Register address for output state
    Wire.write(0b11111100);     // Initialize outputs to "off" state
    Wire.endTransmission();

    // Attach each servo object to its corresponding pin
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);
    }

    // Sensor initialization
    byte status = mpu.begin();
    while(status!=0) {}
    
    // Sensor calibration
    delay(1000);
    mpu.calcOffsets(true,true);
    
}
 
void loop()
{

    // === Send IMU message ===

    // Get angle Z
    mpu.update();
    float angle_z  = mpu.getAngleZ();
    
    // Format angle value into a message string
    char imu_msg[50]; snprintf(imu_msg, sizeof(imu_msg), "IMU,%f", angle_z);

    // Send the message only every 15 loops to reduce communication overhead
    loop_counter ++;
    if (loop_counter % 15 == 0) {   
        sendMessage(imu_msg);
=======
// #include <Arduino.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"
#include "pid/ESP32PIDMotor.hpp"
#include <vector>
#include <sstream>

#define MAX_RPM 350
#define MAX_PWM 255

ESP32Encoder encoderC;
ESP32Encoder encoderD;

// PWM channels for each motor between 0 and 15
const int ledChannelA = 0;
const int ledChannelB = 1;
const int ledChannelC = 2;
const int ledChannelD = 3;

// PWM configuration
const int frequency = 5000;  // Frequency Hz
const int resolution = 8;    // Resolution in bits (from 1 to 15)

// Counter for managing messages frequency
int loop_counter;

// Motor horizontal lift variables
bool m_horizontal_state;
bool ls_horizontal_outside;
bool ls_horizontal_inside;
int m_horizontal_velocity;
int m_horizontal_direction;
unsigned long m_horizontal_start_time = 0;
unsigned long m_horizontal_duration = 0;

// Motor vertical list variables
bool m_vertical_state;
bool ls_vertical_top;
bool ls_vertical_bottom;
int m_vertical_velocity;
int m_vertical_direction;
unsigned long m_vertical_start_time = 0;
unsigned long m_vertical_duration = 0;

// Emergency Button state variable
bool emergency_button_state = 0;

// End action state variables
bool end_action_v = false;
bool end_action_h = false;
bool is_action_v = false;
bool is_action_h = false;

// Lift states
bool state_m_vertical;
bool state_m_vertical_t;
bool state_m_horizontal;
bool state_m_horizontal_t;

bool end_action_m_vertical;
bool end_action_m_horizontal;

void motor_horizontal_lift(int speed, bool dir) {
    /*
      Controls the horizontal lift motor depending on the limit switches values and send the end action message when finish.

      Arguments:
        speed (int): PWM value to control the motor speed (0-255).
        dir (bool): Direction of rotation: true (1) = backward; false (0) = forward.
    */
   
    digitalWrite(GPIO_INA1, dir);

    // if ( (dir == 0 && !ls_horizontal_outside) || (dir == 1 && !ls_horizontal_inside) ) {
        // Stop motor
        int real_speed = 0;
        if (dir == 1) { real_speed = 255; }
        ledcWrite(ledChannelA, real_speed);
        state_m_horizontal      = false;
        end_action_m_horizontal = true;
    // }
    // else {
    //     int real_speed = speed;
    //     if (dir == 1) { real_speed = 255 - real_speed; }
    //     ledcWrite(ledChannelA, real_speed); 
    // }
}

void motor_vertical_lift(int speed, bool dir){
    /*
      Controls the vertical lift motor depending on the limit switches values and send the end action message when finish.

      Arguments:
        speed (int): PWM value to control the motor speed (0-255).
        dir (bool): Direction of rotation: true (1) = up; false (0) = down.
    */
   
    digitalWrite(GPIO_INB1, dir);

    if ( (dir == 1 && !ls_vertical_top) || (dir == 0 && !ls_vertical_bottom) ) { 
        // Stop motor
        int real_speed = 0;
        if (dir == 1) { real_speed = 255; }
        ledcWrite(ledChannelB, real_speed);
        state_m_vertical      = false;
        end_action_m_vertical = true;
    }
    else { 
        int real_speed = speed;
        if (dir == 1) { real_speed = 255 - real_speed; }
        ledcWrite(ledChannelB, real_speed); 
    }
}

void motor_horizontal_lift_t (int speed, bool dir) {
    /*
      Controls the horizontal lift motor during the decided time and send the end action message when finish.

      Arguments:
        speed (int): PWM value to control the motor speed (0-255).
        dir (bool): Direction of rotation: true (1) = backward; false (0) = forward.
    */
   
    digitalWrite(GPIO_INA1, dir);

    if (millis() - m_horizontal_start_time >= m_horizontal_duration) {
        // Stop motor
        m_horizontal_state = false;
        m_horizontal_start_time = 0;
        int real_speed = 0;
        if (dir == 1) { real_speed = 255; }
        ledcWrite(ledChannelA, real_speed);

        state_m_horizontal_t    = false;
        end_action_m_horizontal = true;
    }
    else {
        int real_speed = speed;
        if (dir == 1) { real_speed = 255 - real_speed; }
        ledcWrite(ledChannelA, real_speed); 
    }
}

void motor_vertical_lift_t (int speed, bool dir) {
    /*
      Controls the vertical lift motor during the decided time and send the end action message when finish.

      Arguments:
        speed (int): PWM value to control the motor speed (0-255).
        dir (bool): Direction of rotation: true (0) = down; false (1) = up.
    */
   
    digitalWrite(GPIO_INB1, dir);

    if ( millis() - m_vertical_start_time >= m_vertical_duration ) { 
        // Stop motor
        int real_speed = 0;
        m_vertical_state = false;
        m_vertical_start_time = 0;
        if (dir == 1) { real_speed = 255; }
        ledcWrite(ledChannelB, real_speed);

        state_m_vertical_t    = false;
        end_action_m_vertical = true;
    }
    else { 
        int real_speed = speed;
        if (dir == 1) { real_speed = 255 - real_speed; }
        ledcWrite(ledChannelB, real_speed); 
    }
}

void motor_right(int speed, bool dir) {
    /*
        Controls the right motor.

        Arguments:
        speed (int): PWM value to control the motor speed (0–255).
        dir (bool): Direction of rotation: true (1) = forward; false (0) = backward.
    */

    int real_speed = speed;
    if (dir == 1) { real_speed = 255 - real_speed; }

    digitalWrite(GPIO_IND1,dir);
    ledcWrite(ledChannelD, real_speed);
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

    digitalWrite(GPIO_INC1, dir);
    ledcWrite(ledChannelC, real_speed);
}

int velocity_to_pwm(float velocity) {
    /*
      Converts the desired velocity (m/s) to a PWM value (0–255).

      Arguments:
        velocity (int): Desired robot velocity in meters per second.

      Returns:
        int: PWM value (0 to MAX_PWM).
    */

    // Convert the velocity to rpm
    float desired_rpm = (velocity * 60.0) / (2 * PI * (DIAMETRE_RODES_MM / 1000 / 2));

    // Convert the rpm to pwm
    float pwm = (desired_rpm / MAX_RPM) * MAX_PWM;

    // Clamp the result between 0 and MAX_PWM
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < 0) pwm = 0;

    return (int)pwm;
}


void setup() {
    setupSerial(); 

    // Set up PWM channels for motor control
    ledcSetup(ledChannelA, frequency, resolution);
    ledcAttachPin(GPIO_INA2, ledChannelA);
    
    ledcSetup(ledChannelB, frequency, resolution);
    ledcAttachPin(GPIO_INB2, ledChannelB);

    ledcSetup(ledChannelC, frequency, resolution);
    ledcAttachPin(GPIO_INC2, ledChannelC);

    ledcSetup(ledChannelD, frequency, resolution);
    ledcAttachPin(GPIO_IND2, ledChannelD);


    // Set motor direction pins as outputs
    pinMode(GPIO_INA1, OUTPUT); 
    pinMode(GPIO_INB1, OUTPUT);
    pinMode(GPIO_INC1, OUTPUT);
    pinMode(GPIO_IND1, OUTPUT);

    // Set limit switch pins as inputs
    pinMode(GPIO_ENCA1, INPUT_PULLUP);
    pinMode(GPIO_ENCA2, INPUT_PULLUP);
    pinMode(GPIO_ENCB1, INPUT_PULLUP);
    pinMode(GPIO_ENCB2, INPUT_PULLUP);


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

    // Initialize motors lift variables
    m_horizontal_state = false;     // False = stop; True = moving
    m_vertical_state   = false;     // False = stop; True = movint

    m_horizontal_velocity  = 0;
    m_horizontal_direction = 0;
    m_vertical_velocity    = 0;
    m_vertical_direction   = 0;

    // Initialize motors lift states
    state_m_vertical     = false;
    state_m_vertical_t   = false;
    state_m_horizontal   = false;
    state_m_horizontal_t = false;

    end_action_m_vertical  = false;
    end_action_m_horizontal = false;
}


void loop() {

    // === Motors lift preparation ===

    // Read limit switches values
    ls_horizontal_outside = digitalRead(GPIO_ENCA1);
    ls_horizontal_inside  = digitalRead(GPIO_ENCA2);
    ls_vertical_bottom    = digitalRead(GPIO_ENCB1);
    ls_vertical_top       = digitalRead(GPIO_ENCB2);


    // === Send encoders message ===

    // Get encoder counts from both motors
    int64_t left_encoder_val  = encoderC.getCount() * -1;    // Invert count (forward = backward)
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
    if (loop_counter % 15 == 0) {   
        sendMessage(encoders_msg);
>>>>>>> origin/eur-34-implement-gyro-in-the-movement
    }


    // === Read message ===

    std::string message = readMessage().c_str();
<<<<<<< HEAD
    char end_action[50];

    // Get each value of the message and assign motors power
    if (!message.empty()) {

=======

    // Get each value of the message and assign motors power
    if (!message.empty() && verifyChecksum(message.c_str())) {
>>>>>>> origin/eur-34-implement-gyro-in-the-movement
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
<<<<<<< HEAD
            std::string element_id = message_parts[i];
            int value = stoi(message_parts[i+1]);

            if (element_id == "AP") {
                Wire.beginTransmission(0x20);
                Wire.write(1);

                if(value == 1){ Wire.write(0b11111111); } // Turn all outputs ON.
                else { Wire.write(0b00000000); } // Turn all outputs OFF.

                Wire.endTransmission();
            }
            
            else if (element_id == "S01") { servos[6].write(value == 0 ? 162 : 100); } // PINCER
            else if (element_id == "S02") { servos[1].write(value == 0 ? 140 : 179); } // LEFT SUCTION GRIPPER
            else if (element_id == "S03") { servos[4].write(value == 0 ? 45 : 10); }   // RIGHT SUCTION GRIPPER
            else if (element_id == "S04") { servos[0].write(value == 0 ? 70 : 140); }  // LEFT SHOVEL
            else if (element_id == "S05") { servos[2].write(value == 0 ? 120 : 40); }  // RIGHT SHOVEL
        }

=======
            std::string motor = message_parts[i];
            float vel = stof(message_parts[i+1]);
            int pwm = velocity_to_pwm(abs(vel));
            int dir = 0;

            char end_action[50];

            if (motor == "Emergency") {
                emergency_button_state = vel;
                if (vel == 1) {
                    motor_left(0, 0);
                    motor_right(0, 0);
                    motor_horizontal_lift(0, 0);
                    motor_vertical_lift(0, 0);
                    motor_horizontal_lift_t(0, 0);
                }
            }

            if (emergency_button_state == 0) {
                if (motor == "M01") {
                    state_m_horizontal = true;
                    m_horizontal_direction = 0;
                    if (vel < 0) { m_horizontal_direction = 1; }
                    m_horizontal_velocity = int(abs(vel));
    
                } else if (motor == "M02") {
                    state_m_vertical = true;
                    m_vertical_direction = 1;
                    if (vel < 0) { m_horizontal_direction = 0; }
                    m_vertical_velocity = int(abs(vel));

                } else if (motor == "M01_t") {
                    if (vel < 0) { m_horizontal_direction = 1; }
                    else {m_horizontal_direction = 0;}

                    state_m_horizontal = true;
                    m_horizontal_velocity = 250;
                    m_horizontal_start_time = millis();
                    m_horizontal_duration = (unsigned int)abs(vel);

                } else if (motor == "M02_t") {
                    if (vel < 0) { m_vertical_direction = 0; } 
                    else { m_vertical_direction = 1; }

                    state_m_vertical = true;
                    m_vertical_velocity = 200;
                    m_vertical_start_time = millis();
                    m_vertical_duration = (unsigned int)abs(vel);
    
                } else if (motor == "ML") {
                    if (vel > 0) { dir = 1; }
                    if (pwm == 0) { encoderC.setCount(0); }
                    motor_left(pwm, dir);
    
                } else if (motor == "MR") {
                    if (vel < 0) { dir = 1; }
                    if (pwm == 0) { encoderD.setCount(0); }
                    motor_right(pwm, dir);
                
                }
            }

        }

        if ((!state_m_horizontal && !state_m_horizontal_t) && end_action_m_vertical) { end_action_m_horizontal = true; }
        if ((!state_m_vertical && !state_m_vertical_t) && end_action_m_horizontal) { end_action_m_vertical = true; }
    }


    // === Control lift motors ===

    if (state_m_horizontal && !state_m_horizontal_t)      { 
        motor_horizontal_lift(m_horizontal_velocity, m_horizontal_direction); 
        
        
            
    }
    else if (!state_m_horizontal && state_m_horizontal_t) { motor_horizontal_lift_t(m_horizontal_velocity, m_horizontal_direction); }

    if (state_m_vertical && !state_m_vertical_t)      { motor_vertical_lift(m_vertical_velocity, m_vertical_direction); }
    else if (!state_m_vertical && state_m_vertical_t) { motor_vertical_lift_t(m_vertical_velocity, m_vertical_direction); }


    if (end_action_m_horizontal && end_action_m_vertical) {
        end_action_m_horizontal = false;
        end_action_m_vertical   = false;
        
        char end_action[50];
>>>>>>> origin/eur-34-implement-gyro-in-the-movement
        snprintf(end_action, sizeof(end_action),"EA,1");
        sendMessage(end_action);
    }

<<<<<<< HEAD
    delay(20);
}
=======


    delay(20); // Receive 100 messages per second
}
>>>>>>> origin/eur-34-implement-gyro-in-the-movement
