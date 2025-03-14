#include <Arduino.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"
#include "pid/ESP32PIDMotor.hpp"


// Define the PID parameters. Check the PID timer dependency for more information
PID_params parameters = {
 .kp = 0.02,       // Proportional
 .ki = 0.008,      // Integral
 .kd = 0.002,      // Derivative


 .integral_acc_saturation = -1,    // Integral saturation, used for example if a motor gets stuck, to avoid it spinning for a long time after unstucking it
 .derivative_saturation = -1,      // Derivative saturation, to limit the effects of sudden changes, for example at start
#warning "Chance code to ignore these, should be forced"
 .enable_output_saturation = true, // Output saturation, limits PID output, as the actuator (or whatever) may only accept a range of values. The motor can only take (-1.0, 1.0)
 .output_saturation_upper = 1,     // Upper limit of the saturation, can be negative, but not less than the lower
 .output_saturation_lower = -1,    // Bottom limit of the saturation, can be positive, but not more than the upper
};


// Example of a function to be used to control the motor PID. It's called before running the PID, and receives the current and last pulses of the encoder, that can be used to
// calculate the current motor speed or the distance. It has to return the speed to drive the motor, in pulses per PID cycle.
// Warning! called from interrupt context, can't take too much time, don't use delays nor serial prints.
float set_PID_speed(int64_t current_pulses, int64_t previous_pulses){
 // The speed can be calculated by substracting the pulses
 int64_t delta_pulses = current_pulses - previous_pulses;
 // The speed in rpm depends on the PID period and the encoder pulses per revolution
 const float cycle_time = 10 / 1000;   // 10 ms
 const int pulses_per_rev = 1496;
 float rpm = delta_pulses / cycle_time * 60 / pulses_per_rev;  // Revolutions per minute


 // Return the speed, in pulses per cycle
 return 20.0;
}


// Variable to set the motor speed


PID_Motor_params motor_params1 = {
 .gpio_en = (gpio_num_t) GPIO_INA1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
 .gpio_ph = (gpio_num_t) GPIO_INA2,               // Motor phase, driver input to set the direction to drive the motor.
 .gpio_enc_a = (gpio_num_t) GPIO_ENCA1,            // Encoder first output
 .gpio_enc_b = (gpio_num_t) GPIO_ENCA2,             // Encoder second output


 .motor_direction = true,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.


 .speed_input_var = NULL,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
 //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
 .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended


 .PID_parameters = parameters,             // PID parameters, defined before


 .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};


PID_Motor_params motor_params2 = {
 .gpio_en = (gpio_num_t) GPIO_INB1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
 .gpio_ph = (gpio_num_t) GPIO_INB2,               // Motor phase, driver input to set the direction to drive the motor.
 .gpio_enc_a = (gpio_num_t) GPIO_ENCB1,            // Encoder first output
 .gpio_enc_b = (gpio_num_t) GPIO_ENCB2,             // Encoder second output


 .motor_direction = true,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.


 .speed_input_var = NULL,//&motor_speed_r,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
 //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
 .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended


 .PID_parameters = parameters,             // PID parameters, defined before


 .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};


PID_Motor_params motor_params3 = {
 .gpio_en = (gpio_num_t) GPIO_INC1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
 .gpio_ph = (gpio_num_t) GPIO_INC2,               // Motor phase, driver input to set the direction to drive the motor.
 .gpio_enc_a = (gpio_num_t) GPIO_ENCC1,            // Encoder first output
 .gpio_enc_b = (gpio_num_t) GPIO_ENCC2,             // Encoder second output


 .motor_direction = true,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.


 .speed_input_var = NULL,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
 //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
 .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended


 .PID_parameters = parameters,             // PID parameters, defined before


 .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};


PID_Motor_params motor_params4 = {
 .gpio_en = (gpio_num_t) GPIO_IND1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
 .gpio_ph = (gpio_num_t) GPIO_IND2,               // Motor phase, driver input to set the direction to drive the motor.
 .gpio_enc_a = (gpio_num_t) GPIO_ENCD1,            // Encoder first output
 .gpio_enc_b = (gpio_num_t) GPIO_ENCD2,             // Encoder second output


 .motor_direction = false,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.


 .speed_input_var = NULL,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
 //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
 .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended


 .PID_parameters = parameters,             // PID parameters, defined before


 .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};




// We create the motor object
PID_Motor motor1(motor_params1);
PID_Motor motor2(motor_params2);
PID_Motor motorR(motor_params3);
PID_Motor motorL(motor_params4);

float power_left, power_right;
int   loop_counter;

void setup() {
    setupSerial();  
    Serial.begin(115200);
    
    pinMode(22, OUTPUT);
    digitalWrite(22, 0);

    motor1.initialize_timer();
    motor2.initialize_timer();
    motorR.initialize_timer();
    motorL.initialize_timer();

    motorR.restart_pulses();
    motorL.restart_pulses();

    power_left  = 0.0;
    power_right = 0.0;

    loop_counter = 0;
}

void loop() {

    // Send messages

    /// Prepare the message to send with the encoder value expressed in centimeters
    char encoder_l[20]; snprintf(encoder_l, sizeof(encoder_l), "EL,%ld", (int)(motorL.get_pulses() / MOTOR_POLSOS_PER_CM));
    char encoder_r[20]; snprintf(encoder_r, sizeof(encoder_r), "ER,%ld", (int)(motorR.get_pulses() / MOTOR_POLSOS_PER_CM));

    /// Send less messages per second to avoid delays in communication
    loop_counter ++;
    if (loop_counter % 12 == 0) {   
        sendMessage(encoder_l);
        sendMessage(encoder_r);
    }

    /// Restart pulses at the end of the move
    if (power_left == 0 && power_right == 0) {
        motorR.restart_pulses();
        motorL.restart_pulses();
    }


    // Read message

    String message = readMessage();

    /// Get each value of the message and assign motors power
    if (message.length() > 0) {
        int firstComma = message.indexOf(',');
        int secondComma = message.indexOf(',', firstComma + 1);
        if (firstComma > 0 && secondComma > firstComma) {
            String motorStr    = message.substring(0, firstComma);
            String motorPowStr = message.substring(firstComma + 1, secondComma);
            
            float power = motorPowStr.toInt(); 

            if      (motorStr == "ML") { power_left  = power; }
            else if (motorStr == "MR") { power_right = power; }

            motorL.setpoint = power_left;
            motorR.setpoint = power_right;
        }
    }

    delay(20); /// Receive 100 messages per second
}