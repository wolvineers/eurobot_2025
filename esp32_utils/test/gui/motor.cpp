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
float motor_speed = 0.0;

PID_Motor_params motor_paramsl = {
  .gpio_en = (gpio_num_t) GPIO_INA1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
  .gpio_ph = (gpio_num_t) GPIO_INA2,               // Motor phase, driver input to set the direction to drive the motor. 
  .gpio_enc_a = (gpio_num_t) GPIO_ENCA1,            // Encoder first output
  .gpio_enc_b = (gpio_num_t) GPIO_ENCA2,             // Encoder second output

  .motor_direction = false,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.

  .speed_input_var = NULL,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
  //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
  .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended

  .PID_parameters = parameters,             // PID parameters, defined before

  .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};

PID_Motor_params motor_paramsr = {
  .gpio_en = (gpio_num_t) GPIO_INB1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
  .gpio_ph = (gpio_num_t) GPIO_INB2,               // Motor phase, driver input to set the direction to drive the motor. 
  .gpio_enc_a = (gpio_num_t) GPIO_ENCB1,            // Encoder first output
  .gpio_enc_b = (gpio_num_t) GPIO_ENCB2,             // Encoder second output

  .motor_direction = true,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.

  .speed_input_var = NULL,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
  //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
  .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended

  .PID_parameters = parameters,             // PID parameters, defined before

  .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};


// We create the motor object
PID_Motor motorL(motor_paramsl);
PID_Motor motorR(motor_paramsr);


void setup() {

    setupSerial();
    Serial.begin(115200);

  // If the driver has to be enabled, do it here
  pinMode(22, OUTPUT);
  digitalWrite(22, 0);

  // Create a speed curve, that will move the motor 2050 pulses, at a maximum speed of 50 pulses / cycle, and with an acceleration of 10 pulses / cycle^2
  //motorR.move_distance(4000, 0, 50, 10);
  //motorL.move_distance(4000, 0, 50, 10);

  // Start the motor timer, which starts the speed control
  motorR.initialize_timer();
  motorL.initialize_timer();
}

void loop() {
    // Print the curve execution
    if (!motorR.is_move_distance_finished()) printf("Setpoint, %f, input %f, Output: %f, pos: %li\n", motorR.setpoint, motorR.input, motorR.output, motorR.get_pulses());
    vTaskDelay(10);
    if (!motorL.is_move_distance_finished()) printf("Setpoint, %f, input %f, Output: %f, pos: %li\n", motorL.setpoint, motorL.input, motorL.output, motorL.get_pulses());
    vTaskDelay(10);

    String message = readMessage();
    if (message.length() > 0) {
        int firstComma = message.indexOf(',');
        int secondComma = message.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
            String motorNumberStr = message.substring(2, firstComma);
            String motorAngleStr = message.substring(firstComma + 1, secondComma);
            String checksumStr = message.substring(secondComma + 1);
            int motorNumber = motorNumberStr.toInt();
            int motorAngle = motorAngleStr.toInt();  // Valor entre -100 i 100
            int checksum = checksumStr.toInt();

            Serial.println("Motor Number: " + String(motorNumber + 1));
            Serial.println("Motor %: " + String(motorAngle));
            Serial.println("Checksum: " + String(checksum));

            if (motorNumber == 1) {
                motorL.setpoint = motorAngle; // Mou el motor L
            } else if (motorNumber == 2) {
                motorR.setpoint = motorAngle; // Mou el motor R
            }
            else{
                Serial.println("else");
            }

        } else {
            Serial.println("Error: Número de motor invàlid");
        }
    } else {
        Serial.println("Error: Formato de mensaje inválido");
    }
}







