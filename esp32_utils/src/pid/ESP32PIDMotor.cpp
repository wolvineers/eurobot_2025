#include "ESP32PIDMotor.hpp"

#include <math.h>   // For sqrt

#ifndef Arduino_h
  #error "Library not implemented in esp-idf for now"
#endif

// Curve speed methods
// Constructor
speed_curve::speed_curve(int64_t starting_pulses, int64_t pulses_to_move, float minimum_speed, float top_speed, float accel){
  // Store the parameters
  initial_pulses = starting_pulses;
  end_pulses = starting_pulses + pulses_to_move;
  initial_end_speed = minimum_speed;  // Not implemented for now
  maximum_speed = top_speed;
  acceleration = accel;

  // Calculate the times of the different zones
  float acceleration_area = maximum_speed * maximum_speed / acceleration;   // Total distance of acceleration and deacceleration if they reach the maximum speed
  if(acceleration_area >= pulses_to_move){
    // Doesn't accelerate to top speed, area is lower
    acceleration_area = pulses_to_move;
    constant_period_exists = false;
  } else{
    constant_period_exists = true;
  }
  printf("Accel area: %f\n", acceleration_area);

  // Obtains the acceleration, constant speed, and deacceleration end times in periods
  acceleration_period_end = sqrt(acceleration_area/acceleration);
  if(constant_period_exists){
    constant_period_end = (pulses_to_move - acceleration_area) / maximum_speed + acceleration_period_end;
  } else{
    constant_period_end = acceleration_period_end;
  }
  deaccel_period_end = constant_period_end + acceleration_period_end; // It is symetrical

  printf("Periods: accel %f const %f exists %i deaccel %f\n", acceleration_period_end, constant_period_end, constant_period_exists?1:0, deaccel_period_end);

  // Initialize the PID variables
  cycle_time = 0;
  setpoint = 0;

  is_curve_finished = false;
}

// Get speed method. It averages the initial value and the end value of the period, taking into account zone changes, to traverse the distance accurately
float speed_curve::get_speed(){
  // Check first if the curve has ended to reduce execution time
  if(cycle_time > deaccel_period_end){
    setpoint = 0.0f;
    is_curve_finished = true;
    return setpoint;
  }

  // Check the point where the curve 
  if(cycle_time < acceleration_period_end){
    // Accelerating
    if((cycle_time + 1) <= acceleration_period_end){
      // All the period is acceleration
      setpoint = ((cycle_time + 0.5) * acceleration);   // Speed is the average of the start and end of the curve, or speed at the middle point
    } else{
      // Add the speed portion of the acceleration
      setpoint = ((cycle_time + acceleration_period_end) * acceleration / 2) * (acceleration_period_end - cycle_time); // Average acceleration speed, adding the proportional part to the setpoint
      // Add the constant period
      if(constant_period_exists){
        if((cycle_time + 1) <= constant_period_end){
          // The rest of the period is constant
          setpoint += maximum_speed * (cycle_time + 1 - acceleration_period_end); // Add the rest of the speed component
        } else{
          // Add the speed portion of the constant speed
          setpoint += maximum_speed * (constant_period_end - acceleration_period_end);
        }
      }
      // Add the portion of the deacceleration, if it exists
      if((cycle_time + 1) > constant_period_end){
        if((cycle_time + 1) <= deaccel_period_end){
          // The cycle ends in deacceleration. Get the average speed during deacceleration, and add the proportional part to the setpoint
          setpoint += (((deaccel_period_end - constant_period_end) + (deaccel_period_end - (cycle_time + 1))) * acceleration) / 2 * ((cycle_time + 1) - constant_period_end);
        } else{
          // The cycle ends in stop
          setpoint += (deaccel_period_end - constant_period_end) * acceleration / 2 * (deaccel_period_end - constant_period_end);
        }
      }
    }
  } else if(constant_period_exists && (cycle_time < constant_period_end)){
    // Constant speed
    if((cycle_time + 1) <= constant_period_end){
      // All the period is at constant speed
      setpoint = maximum_speed;
    } else{
      // Add the speed portion of the constant speed
      setpoint = maximum_speed * (constant_period_end - cycle_time);
      // Add the portion of the deacceleration
      if((cycle_time + 1) <= deaccel_period_end){
        // The cycle ends in deacceleration
        // Speed is (maximum + (deaccel_end - (cycle + 1)) * accel), so average of maximum and deaccelerated
        setpoint += (maximum_speed + ((deaccel_period_end - (cycle_time + 1)) * acceleration)) / 2 * ((cycle_time + 1) - constant_period_end);
      } else{
        // The cycle ends in stop
        // Speed is half of maximum * period
        setpoint += (maximum_speed / 2) * (deaccel_period_end - constant_period_end);
      }
    }
  } else{
    // Deacceleration
    if((cycle_time + 1) <= deaccel_period_end){
      // The cycle ends in deacceleration
      setpoint = (deaccel_period_end - (cycle_time + 0.5)) * acceleration;  // Calculating the speed with the distance from stop and acceleration
    } else{
      // The cycle ends in stop
      setpoint = ((deaccel_period_end - cycle_time) * acceleration) / 2 * (deaccel_period_end - cycle_time);  // (Initial speed) / 2 * remaining time
    }
  }

  cycle_time++;

  return setpoint;
}

// PID motor
// PID motor static variables initialization
uint PID_Motor::PWM_number_next = 0; // Use from the first one, may change later

// PID motor methods
// Constructor
PID_Motor::PID_Motor(PID_Motor_params motor_parameters) :
                     PID_timer(motor_parameters.PID_parameters, motor_parameters.timer_period_us, -1, NULL, NULL){
  PID_Motor_parameters = motor_parameters;      // Structs can be copied directly

  // Configure the pins as inputs and outputs
  pinMode(motor_parameters.gpio_en, OUTPUT);
  digitalWrite(motor_parameters.gpio_en, LOW);  // Mantain motor disabled
  pinMode(motor_parameters.gpio_ph, OUTPUT);
  digitalWrite(motor_parameters.gpio_ph, motor_parameters.motor_direction); // Set the directions

  // Initialize the PWM with a frequency of 20 kHz (inaudible) and 10 bit
  PWM_number = PWM_number_next++;
  ledcSetup(PWM_number, 20000, 10);
  ledcAttachPin(motor_parameters.gpio_en, PWM_number);
  ledcWrite(PWM_number, 0);

  // Initialize encoder
  motor_encoder.attachFullQuad(motor_parameters.gpio_enc_a, motor_parameters.gpio_enc_b);
  motor_encoder.setFilter(1023);
  motor_encoder.clearCount();
}

// Method to assign the input and the setpoint
void PID_Motor::pre_cycle_method(float * input, float * setpoint){
  // Get the input, motor speed from encoder pulses
  int64_t pulses = motor_encoder.getCount();
  *input = (float)(pulses - previous_pulses);
  if(PID_Motor_parameters.motor_direction) *input = - *input;

  // Get the setpoint
  if(motor_speed_curve != NULL){
    // Follow the speed curve
    *setpoint = motor_speed_curve->get_speed();
    // Check if it is the first time that the curve has finished
    if(!already_finished && is_move_distance_finished()){
      // Handle the curve completion
      if(finished_curve_pointer) *finished_curve_pointer = true;
      if(finished_callback) finished_callback();
      if(delete_curve_finished) end_move_distance();
      already_finished = true;
    }
  } else if(PID_Motor_parameters.speed_input_var != NULL){
    // Use the variable, has preference
    *setpoint = *PID_Motor_parameters.speed_input_var;
  } else if(PID_Motor_parameters.speed_input_function != NULL){
    // Use the function
    *setpoint = PID_Motor_parameters.speed_input_function(pulses, previous_pulses);
  }

  // Save the pulses
  previous_pulses = pulses;
  ESP_EARLY_LOGI("PID", "i: %d, s: %d, p: %d", *input, *setpoint, *pulses);
}

// Method to act on the motor according to the output
void PID_Motor::post_cycle_method(float output){
  if(output == 0.0f){
    // Output 0
    ledcWrite(PWM_number, 0);
  } else if(signbit(output)){
    // Negative output, inverse direction
    gpio_set_level(PID_Motor_parameters.gpio_ph, !PID_Motor_parameters.motor_direction);
    ledcWrite(PWM_number, - output * (1 << 10));
  } else{
    // Positive output, direct direction
    gpio_set_level(PID_Motor_parameters.gpio_ph, PID_Motor_parameters.motor_direction);
    ledcWrite(PWM_number, output * (1 << 10));
  }
}

// Speed curve methods
// Creates the curve and adds it to the pointer
void PID_Motor::move_distance(int64_t pulses_to_move, float minimum_speed, float top_speed, float accel, bool auto_delete, bool * curve_finished, void (* finished_callback)(void)){
  speed_curve * previous_pointer = motor_speed_curve; // Free it later to avoid undefined behaviour if an interrupt happens
  motor_speed_curve = new speed_curve(this->get_pulses(), pulses_to_move, minimum_speed, top_speed, accel);
  if(previous_pointer != NULL) delete previous_pointer;
  delete_curve_finished = auto_delete;
  finished_curve_pointer = curve_finished;
  if(finished_curve_pointer) *finished_curve_pointer = false;
  finished_callback = finished_callback;
  already_finished = false;
}

// Get if the curve movement has finished
bool PID_Motor::is_move_distance_finished(){
  if(motor_speed_curve == NULL || motor_speed_curve->is_curve_finished){
    return true;
  } else{
    return false;
  }
}

// Delete the curve object
void PID_Motor::end_move_distance(){
  if(motor_speed_curve != NULL){
    delete motor_speed_curve;
    motor_speed_curve = NULL;
  }
}

// Ouputs the motor encoder pulses
int64_t PID_Motor::get_pulses(){
  int64_t var = motor_encoder.getCount();
  //printf("Pulses: %li %li\n", var, previous_pulses);
  return motor_encoder.getCount();
}

// Restarts the encoder pulses, sets them to zero
void PID_Motor::restart_pulses(){
  motor_encoder.clearCount();
}