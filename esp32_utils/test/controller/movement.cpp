#include <Arduino.h>
#include <ESP32Servo.h>
#include "Placa.h"
#include "serial_utils/serial_utils.h"
#include "pid/ESP32PIDMotor.hpp"


float motor_speed = 0.0;

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

PID_Motor_params motor_paramsl = {
 .gpio_en = (gpio_num_t) GPIO_INA1,               // Motor enable, driver input to control with PWM and turns on and off the motor.
 .gpio_ph = (gpio_num_t) GPIO_INA2,               // Motor phase, driver input to set the direction to drive the motor.
 .gpio_enc_a = (gpio_num_t) GPIO_ENCA1,            // Encoder first output
 .gpio_enc_b = (gpio_num_t) GPIO_ENCA2,             // Encoder second output


 .motor_direction = false,                 // Motor direction, inverts the encoder and direction of the motor. For example, two motors on opposite sides.


 .speed_input_var = &motor_speed,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
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


 .speed_input_var = &motor_speed,          // The variable has to be passed as reference. Has priority over the function. When using the speed curve, it gets overriden.
 //.speed_input_function = set_PID_speed,  // The fuction that controls the speed. Has less priority than the variable. Not used in this example
 .speed_input_function = NULL,             // If not used, set the value to NULL. If both are NULL, it can be controlled directly modifying setpoint, but using the variable is recommended


 .PID_parameters = parameters,             // PID parameters, defined before


 .timer_period_us = 10 * 1000,             // PID cycle period, in us.
};

PID_Motor motorL(motor_paramsl);
PID_Motor motorR(motor_paramsr);


void setup() {
    setupSerial();
    Serial.begin(115200);

    pinMode(22, OUTPUT);
    digitalWrite(22, 0);

    motorL.initialize_timer();
    motorR.initialize_timer();

}

void loop() {
    

    String message = readMessage();
    if (message.length() > 0) {
        int firstComma = message.indexOf(",");
        int secondComma = message.indexOf(',', firstComma + 1);

        if (firstComma != -1 && secondComma != -1) {
            String distanceStr = message.substring(message.indexOf('D') + 2, firstComma);
            
            String powerStr = message.substring(message.indexOf('P') + 2, secondComma);

            String checksumStr = message.substring(secondComma + 1);
            
            int distance = distanceStr.toInt();
            int power = powerStr.toInt();  
            int checksum = checksumStr.toInt();
            
            Serial.println("Distance: " + String(distance) + ", Power: " + String(power));
            motorL.move_distance(MOTOR_POLSOS_PER_CM * distance, 0, power, 10);
            Serial.println(MOTOR_POLSOS_PER_CM * distance);            
        }
        else {
            Serial.println("Error: Invalid message format");
        }
    }
}
