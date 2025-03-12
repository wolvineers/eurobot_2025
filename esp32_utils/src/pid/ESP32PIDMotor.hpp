#ifndef ESP32PIDMotor
#define ESP32PIDMotor

// Check if using arduino
#ifdef ARDUINO
#include <Arduino.h>  // Add the library
#endif

// Incldue the digital pid
#include "PID_DIG_ESP.hpp"

// Include the encoder library
#include "ESP32Encoder.h"

// Type definitions
// PID motor class structure
typedef struct PID_Motor_params {
    // Pin assignation
    gpio_num_t gpio_en;         // Motor enable, receives PWM signal
    gpio_num_t gpio_ph;         // Motor phase, sets the motor direction
    gpio_num_t gpio_enc_a;      // Encoder pin A
    gpio_num_t gpio_enc_b;      // Encoder pin B

    // Motor direction
    bool motor_direction;       // Invert the direction

    // Pointer to variable and function to set speed. The variable takes preference
    float * speed_input_var;         // Pointer to the speed input variable, takes preference over the function
    float (*speed_input_function)(int64_t current_pulses, int64_t previous_pulses);  // Function called to set the motor speed, less preference than the variable

    // PID parameters
    PID_params PID_parameters;

    // PID timer period, in us
    uint32_t timer_period_us;
} PID_Motor_params;

/**
 * @brief Class that creates a trapezoidal speed curve for a motor. That is, an acceleration period, a constant speed period, and a
 *        deacceleration period, that their area is the distance to move. It is symetrical. The acceleration avoids sudden moves,
 *        that can reduce encoder precision. It outputs the average speed of the step, instead of the initial or final, for better control.
 */
class speed_curve{
    private:
    int64_t initial_pulses = 0;         // Pulses at the beginning of the curve execution
    int64_t end_pulses = 0;             // Pulses at the end of the curve execution

    float acceleration = 0;             // Acceleration of the curve in the acceleration part, in pulses/cycle²
    float initial_end_speed = 0;        // Initial and end speed of the curve, in pulses/cycle
    float maximum_speed = 0;            // Maximum speed of the curve, in pulses/cycle

    // Time of the curve's zone limits
    float acceleration_period_end = 0;  // Cycle at which the acceleration ends, can be between cycles
    bool constant_period_exists = true; // If a constant period exists, if false the acceleration doesn't have time to reach max speed
    float constant_period_end = 0;      // Cycle at which the constant speed ends, can be between cycles
    float deaccel_period_end = 0;       // Cycle at which the deacceleration speed ends, can be between cycles

    // PID control
    float setpoint = 0;                 // Setpoint to set the PID
    uint32_t cycle_time = 0;            // Cycle number of the PID

    public:

    bool is_curve_finished = false;     // Stores whether if the curve has finished. It shouldn't be overriden.

    /**
     * @brief Creates a trapezoidal speed curve
     * 
     * @param starting_pulses The encoder pulses when the curve is created, read the encoder and pass it here.
     * @param pulses_to_move The total pulses to move, as encoder pulses, so distance should be converted.
     * @param minimum_speed The minimum speed when starting to accelerate and finishing to deaccelerate. Not implemented. In pulses per step.
     * @param top_speed The top speed of the curve, of the constant speed period. It may not reach it for small distances. In pulses per step.
     * @param accel The acceleration and deacceleration of the curve, in pulses per step^2.
     */
    speed_curve(int64_t starting_pulses = 0, int64_t pulses_to_move = 0, float minimum_speed = 0, float top_speed = 0, float accel = 0);

    /**
     * @brief Get the speed of the curve, and advances by one step.
     * 
     * @returns The speed to drive the motor, in pulses per step. It is the average speed of the step.
     */
    float get_speed();
};

/**
 * @brief PID motor class, creates an object of PID_Motor class calling PID_Motor <name>(<PID_Motor_params>).
 *        It controls a motor with a PID, using a timer for executing the PID and controlling the motor.
 *        It works in pulses, so transform distances to pulses while using the class. For now, each motor
 *        needs its own timer. Each motor needs its own PWM.
*/
class PID_Motor : public PID_timer{
    private:
        // Parameter structure
        PID_Motor_params PID_Motor_parameters = {.gpio_en = GPIO_NUM_NC, .gpio_ph = GPIO_NUM_NC, .gpio_enc_a = GPIO_NUM_NC, .gpio_enc_b = GPIO_NUM_NC, 
                                                 .motor_direction = false, .speed_input_var = NULL, .speed_input_function = NULL, .PID_parameters = {0},
                                                 .timer_period_us = 0};

<<<<<<< HEAD
        // Encoder object
        ESP32Encoder motor_encoder;

=======
>>>>>>> development
        // Pulses of the encoder on the last PID cycle
        int64_t previous_pulses = 0;

        // Variable to store the PWM number to use
        static uint PWM_number_next;    // Next PWM number to use
        uint PWM_number = 0;            // PWM number of this motor

        // Speed curve
        speed_curve * motor_speed_curve = NULL;     // Stores the current speed curve
        bool delete_curve_finished = false;         // Whether if the curve should be deleted automatically when it has finished
        bool * finished_curve_pointer = NULL;       // Pointer to set to true upon curve completion
        void (* finished_callback)(void) = NULL;    // Pointer to callback to call when curve is finished, before it is deleted.
        bool already_finished = false;              // To chceck if it is the first time the curve has ended, internal

    
    public:
<<<<<<< HEAD
=======

        // Encoder object
        ESP32Encoder motor_encoder;

>>>>>>> development
        /**
         * @brief Motor with digital PID constructor.
         * 
         * @param motor_parameters Parameters of the motor and PID. Use the speed input variable or function to control the motor speed, or use the
         *                         speed curve to move a distance. The speed input method can't be changed later, so if both methods should be used,
         *                         set the variable / function and use the move_distance when needded, which overrides the inputs.
         */
        PID_Motor(PID_Motor_params motor_parameters = {.gpio_en = GPIO_NUM_NC, .gpio_ph = GPIO_NUM_NC, .gpio_enc_a = GPIO_NUM_NC, .gpio_enc_b = GPIO_NUM_NC, 
                                                       .motor_direction = false, .speed_input_var = NULL, .speed_input_function = NULL, .PID_parameters = {
                                                            .kp = 0, .ki = 0, .kd = 0, .integral_acc_saturation = -1, .derivative_saturation = -1,
                                                            .enable_output_saturation = false, .output_saturation_upper = -1, .output_saturation_lower = -1},
                                                       .timer_period_us = 0});
        
        // Override pre and post cycle methods from PID_timer
        void pre_cycle_method(float * input, float * setpoint) override;
        void post_cycle_method(float output) override;

        // Speed curve methods
        /**
         * @brief Creates a new trapezoidal speed curve, in order to move a certain distance. Will then ignore the variable and function inputs.
         *        In order to regain control of the speed, use the end_move_distance. Upon completion, it will keep the PID setpoint to zero.
         * 
         * @param pulses_to_move Distance to move, in encoder pulses
         * @param minimum_speed Minimum speed at the start and end of the movement, in pulses per cycle. Not implemented at the moment
         * @param top_speed Maximum speed, speed during the constant speed movement, in pulses per cycle. May not reach it with small movements
         * @param accel Acceleration, in pulses per cycle squared
         * @param auto_delete Delete the curve automatically upon completion and return the control to the variable / function input. Beware
         *                    that, if the input is not zero, it will start to move.
         * @param curve_finished Pointer to boolean that will be set to one once the curve finishes. It will be set to zero automatically.
         * @param finished_callback Pointer to a function without parameters that will be called upon curve completion, before deleting it
         *                          if auto_delete is true, so the inputs of the motor can be set.
         */
        void move_distance(int64_t pulses_to_move = 0, float minimum_speed = 0, float top_speed = 0, float accel = 0, bool auto_delete = false,
                           bool * curve_finished = NULL, void (* finished_callback)(void) = NULL);

        /**
         * @brief Method to check if the movement started by move_distance is finished.
         * 
         * @returns If the movement is finished, or it wasn't started, it returns true. Otherwise, it returns false
         */
        bool is_move_distance_finished();

        /**
         * @brief Ends the distance movement initiated in move_distance, even if it was still moving, and returns the control to the variable
         *        and function inputs. You can check with is_move_distance_finished whether if movement is finished before calling this method.
         */
        void end_move_distance();

        /**
         * @brief Get the amount of pulses of the encoder
         * 
         * @returns Number of pulses, signed
         */
        int64_t get_pulses();

        /**
<<<<<<< HEAD
         * @brief Restart the encoder's pulses to zero
=======
         * @brief Restart the encoder's pulses to zero and PID
>>>>>>> development
         */
        void restart_pulses();
};

#endif