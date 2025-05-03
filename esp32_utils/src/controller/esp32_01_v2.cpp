// #include <Arduino.h>
// #include "Placa.h"
// #include "serial_utils/serial_utils.h"
// #include "pid/ESP32PIDMotor.hpp"
// #include <vector>
// #include <sstream>

// #define MAX_RPM 300
// #define MAX_PWM 255

// ESP32Encoder encoderC;
// ESP32Encoder encoderD;

// // PWM channels for each motor between 0 and 15
// const int ledChannelA = 0;
// const int ledChannelB = 1;
// const int ledChannelC = 2;
// const int ledChannelD = 3;

// // PWM configuration
// const int frequency = 5000;  // Frequency Hz
// const int resolution = 8;    // Resolution in bits (from 1 to 15)

// // Counter for managing messages frequency
// int loop_counter;

// // Motor horizontal lift variables
// bool m_horizontal_state;
// bool ls_horizontal_outside;
// bool ls_horizontal_inside;
// int m_horizontal_velocity;
// int m_horizontal_direction;
// unsigned long m_horizontal_start_time = 0;
// unsigned long m_horizontal_duration = 0;

// // Motor vertical list variables
// bool m_vertical_state;
// bool ls_vertical_top;
// bool ls_vertical_bottom;
// int m_vertical_velocity;
// int m_vertical_direction;
// unsigned long m_vertical_start_time = 0;
// unsigned long m_vertical_duration = 0;


// void motor_horizontal_lift(int speed, bool dir) {
//     /*
//       Controls the horizontal lift motor depending on the limit switches values and send the end action message when finish.

//       Arguments:
//         speed (int): PWM value to control the motor speed (0-255).
//         dir (bool): Direction of rotation: true (1) = backward; false (0) = forward.
//     */
   
//     digitalWrite(GPIO_INA1, dir);

//     if ( (dir == 0 && !ls_horizontal_outside) || (dir == 1 && !ls_horizontal_inside) ) {
//         // Stop motor
//         ledcWrite(ledChannelA, 0);
//         m_horizontal_state = false;

//         // Send end of action message
//         char end_action[50];
//         snprintf(end_action, sizeof(end_action),"EA,1");
//         sendMessage(end_action);
//     }
//     else { ledcWrite(ledChannelA, speed); }
// }

// void motor_vertical_lift(int speed, bool dir){
//     /*
//       Controls the vertical lift motor depending on the limit switches values and send the end action message when finish.

//       Arguments:
//         speed (int): PWM value to control the motor speed (0-255).
//         dir (bool): Direction of rotation: true (1) = down; false (0) = up.
//     */
   
//     digitalWrite(GPIO_INB1, dir);

//     if ( (dir == 0 && !ls_vertical_top) || (dir == 1 && !ls_vertical_bottom) ) { 
//         // Stop motor
//         ledcWrite(ledChannelB, 0);
//         m_vertical_state = false;

//         // Send end of action message
//         char end_action[50];
//         snprintf(end_action, sizeof(end_action),"EA,1");
//         sendMessage(end_action);
//     }
//     else { ledcWrite(ledChannelB, speed); }
// }

// void motor_horizontal_lift_t (int speed, bool dir) {
//     /*
//       Controls the horizontal lift motor during the decided time and send the end action message when finish.

//       Arguments:
//         speed (int): PWM value to control the motor speed (0-255).
//         dir (bool): Direction of rotation: true (1) = backward; false (0) = forward.
//     */
   
//     digitalWrite(GPIO_INA1, dir);

//     if (millis() - m_horizontal_start_time >= m_horizontal_duration) {
//         // Stop motor
//         ledcWrite(ledChannelA, 0);
//         m_horizontal_state = false;
//         m_horizontal_start_time = 0;

//         // Send end of action message
//         char end_action[50];
//         snprintf(end_action, sizeof(end_action),"EA,1");
//         sendMessage(end_action);
//     }
//     else { ledcWrite(ledChannelA, speed); }
// }

// void motor_vertical_lift_t (int speed, bool dir) {
//     /*
//       Controls the vertical lift motor during the decided time and send the end action message when finish.

//       Arguments:
//         speed (int): PWM value to control the motor speed (0-255).
//         dir (bool): Direction of rotation: true (1) = down; false (0) = up.
//     */
   
//     digitalWrite(GPIO_INB1, dir);

//     if ( millis() - m_vertical_start_time >= m_vertical_duration ) { 
//         // Stop motor
//         ledcWrite(ledChannelB, 0);
//         m_vertical_state = false;
//         m_vertical_start_time = 0;

//         // Send end of action message
//         char end_action[50];
//         snprintf(end_action, sizeof(end_action),"EA,1");
//         sendMessage(end_action);
//     }
//     else { ledcWrite(ledChannelB, speed); }
// }

// void motor_right(int speed, bool dir) {
//     /*
//         Controls the right motor.

//         Arguments:
//         speed (int): PWM value to control the motor speed (0–255).
//         dir (bool): Direction of rotation: true (1) = forward; false (0) = backward.
//     */

//     int real_speed = speed;
//     if (dir == 1) { real_speed = 255 - real_speed; }

//     digitalWrite(GPIO_IND1,dir);
//     ledcWrite(ledChannelD, real_speed);
// }

// void motor_left(int speed, bool dir) {
//     /*
//       Controls the left motor.
    
//       Arguments:
//         speed (int): PWM value to control the motor speed (0–255).
//         dir (bool): Direction of rotation: true (1) = backward; false (0) = forward.
//     */

//     int real_speed = speed;
//     if (dir == 1) { real_speed = 255 - real_speed; }

//     digitalWrite(GPIO_INC1, dir);
//     ledcWrite(ledChannelC, real_speed);
// }

// int velocity_to_pwm(float velocity) {
//     /*
//       Converts the desired velocity (m/s) to a PWM value (0–255).

//       Arguments:
//         velocity (int): Desired robot velocity in meters per second.

//       Returns:
//         int: PWM value (0 to MAX_PWM).
//     */

//     // Convert the velocity to rpm
//     float desired_rpm = (velocity * 60.0) / (2 * PI * (DIAMETRE_RODES_MM / 1000 / 2));

//     // Convert the rpm to pwm
//     float pwm = (desired_rpm / MAX_RPM) * MAX_PWM;

//     // Clamp the result between 0 and MAX_PWM
//     if (pwm > MAX_PWM) pwm = MAX_PWM;
//     if (pwm < 0) pwm = 0;

//     return (int)pwm;
// }


// void setup() {
//     setupSerial(); 

//     // Set up PWM channels for motor control
//     ledcSetup(ledChannelA, frequency, resolution);
//     ledcAttachPin(GPIO_INA2, ledChannelA);
    
//     ledcSetup(ledChannelB, frequency, resolution);
//     ledcAttachPin(GPIO_INB2, ledChannelB);

//     ledcSetup(ledChannelC, frequency, resolution);
//     ledcAttachPin(GPIO_INC2, ledChannelC);

//     ledcSetup(ledChannelD, frequency, resolution);
//     ledcAttachPin(GPIO_IND2, ledChannelD);


//     // Set motor direction pins as outputs
//     pinMode(GPIO_INA1, OUTPUT); 
//     pinMode(GPIO_INB1, OUTPUT);
//     pinMode(GPIO_INC1, OUTPUT);
//     pinMode(GPIO_IND1, OUTPUT);

//     // Set limit switch pins as inputs
//     pinMode(GPIO_ENCA1, INPUT_PULLUP);
//     pinMode(GPIO_ENCA2, INPUT_PULLUP);
//     pinMode(GPIO_ENCB1, INPUT_PULLUP);
//     pinMode(GPIO_ENCB2, INPUT_PULLUP);


//     // Enable motors (active low)
//     pinMode(GPIO_DISABLE_MOTORS, OUTPUT);
//     digitalWrite(GPIO_DISABLE_MOTORS, 0);


//     // Attach and configure motor encoders
//     encoderC.attachFullQuad(GPIO_ENCC1,GPIO_ENCC2);
//     encoderC.setFilter(1023);
//     encoderC.setCount(0);

//     encoderD.attachFullQuad(GPIO_ENCD1,GPIO_ENCD2);
//     encoderD.setFilter(1023);
//     encoderD.setCount(0);

//     // Initialize motors lift variables
//     m_horizontal_state = false;     // False = stop; True = moving
//     m_vertical_state   = false;     // False = stop; True = movint

//     m_horizontal_velocity  = 0;
//     m_horizontal_direction = 0;
//     m_vertical_velocity    = 0;
//     m_vertical_direction   = 0;
// }


// void loop() {

//     // === Motors lift preparation ===

//     // Read limit switches values
//     ls_horizontal_outside = digitalRead(GPIO_ENCA1);
//     ls_horizontal_inside  = digitalRead(GPIO_ENCA2);
//     ls_vertical_bottom    = digitalRead(GPIO_ENCB1);
//     ls_vertical_top       = digitalRead(GPIO_ENCB2);


//     // === Send encoders message ===

//     // Get encoder counts from both motors
//     int64_t left_encoder_val  = encoderC.getCount() * -1;    // Invert count (forward = backward)
//     int64_t right_encoder_val = encoderD.getCount();
    
//     // Format encoder values into a message string (in centimeters)
//     char encoders_msg[50]; snprintf(
//         encoders_msg, sizeof(encoders_msg),
//         "EL,%ld,ER,%ld",
//         (int)(left_encoder_val / MOTOR_POLSOS_PER_CM),
//         (int)(right_encoder_val / MOTOR_POLSOS_PER_CM)
//     );

//     // Send the message only every 12 loops to reduce communication overhead
//     loop_counter ++;
//     if (loop_counter % 15 == 0) {   
//         sendMessage(encoders_msg);
//     }

//     motor_left(150,1);


//     // === Read message ===

//     std::string message = readMessage().c_str();

//     // Get each value of the message and assign motors power
//     if (!message.empty()) {
//         std::vector<std::string> message_parts;
//         std::stringstream ss(message);
//         std::string item;

//         // Split the message by commas and store parts
//         while (std::getline(ss, item, ',')) {
//             message_parts.push_back(item);
//         }

//         // Remove the last part as it's a checksum
//         if (!message_parts.empty()) {
//             message_parts.pop_back();
//         }

//         // Process command pairs
//         for (int i = 0; i < message_parts.size(); i += 2) {
//             std::string motor = message_parts[i];
//             float vel = stof(message_parts[i+1]);
//             int pwm = velocity_to_pwm(abs(vel));
//             int dir = 0;

//             char end_action[50];

//             if (motor == "M01") {
//                 if (vel < 0) { m_horizontal_direction = 1; }   // Negative velocity = backward movement
//                 m_horizontal_state = true;
//                 m_horizontal_velocity = (int)vel;

//                 // motor_horizontal_lift(int(vel), dir);

//                 // snprintf(encoders_msg, sizeof(end_action),"EA,1");
//                 // sendMessage(end_action);

//             } else if (motor == "M02") {
//                 if (vel < 0) { m_vertical_direction = 1; }   // Negative velocity = down movement
//                 m_vertical_state = true;
//                 m_vertical_velocity = (int)vel;
                
//                 // motor_vertical_lift(int(vel), dir);

//                 // snprintf(encoders_msg, sizeof(end_action),"EA,1");
//                 // sendMessage(end_action);

//             } else if (motor == "M01_t") {
//                 if (vel < 0) { m_horizontal_direction = 1; }

//                 m_horizontal_state = true;
//                 m_horizontal_velocity = 150;
//                 m_horizontal_start_time = millis();
//                 m_horizontal_duration = (unsigned int)vel;
                
//             } else if (motor == "M02_t") {
//                 m_vertical_state = true;
//                 m_vertical_velocity = 150;
//                 m_vertical_start_time = millis();
//                 m_vertical_duration = (unsigned int)vel;

//                 if (vel < 0) { 
//                     m_vertical_direction = 1; 
//                     m_vertical_velocity = 10;
//                 }

//             } else if (motor == "ML") {
//                 if (vel > 0) { dir = 1; }
//                 if (pwm == 0) { encoderC.setCount(0); }
//                 motor_left(pwm, dir);

//             } else if (motor == "MR") {
//                 if (vel < 0) { dir = 1; }
//                 if (pwm == 0) { encoderD.setCount(0); }
//                 motor_right(pwm, dir);
//             }
//         }
//     }


//     // === Control lift motors ===

//     if (m_horizontal_state && m_horizontal_start_time == 0)      { motor_horizontal_lift(m_horizontal_velocity, m_horizontal_direction); }
//     else if (m_horizontal_state && m_horizontal_start_time != 0) { motor_horizontal_lift_t(m_horizontal_velocity, m_horizontal_direction); }

//     if (m_vertical_state && m_vertical_start_time == 0)      { motor_vertical_lift(m_vertical_velocity, m_vertical_state); }
//     else if (m_vertical_state && m_vertical_start_time != 0) { motor_vertical_lift_t(m_vertical_velocity, m_vertical_state); }

//     delay(20); // Receive 100 messages per second
// }