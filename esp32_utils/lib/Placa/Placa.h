#ifndef PLACA_ROBOT
#define PLACA_ROBOT

// Definicions físiques
// Rodes
#define DIAMETRE_RODES_MM       65.0
#define PERIMETRE_RODES_CM      DIAMETRE_RODES_MM/10.0*3.14159265358979323846
// Motors
/* Configuration for 350RP 12V Motors
#define MOTOR_REDUCCIO          34.0
#define MOTOR_POLSOS_PER_REV    11.0 * 4 * MOTOR_REDUCCIO                           // Polsos simples * 4 canvis/simple * reducció
#define MOTOR_POLSOS_PER_CM     MOTOR_POLSOS_PER_REV / PERIMETRE_RODES_CM           // Polsos/rev / cm/rev
#define MOTOR_VEL_MAX_RPM       350     
#define MOTOR_VEL_MAX_CM_S      MOTOR_VEL_MAX_RPM / 60.0 * PERIMETRE_RODES_CM       // Màxim rpm / 60s/min / cm/rev
#define MOTOR_VEL_MAX_POLS_S    MOTOR_VEL_MAX_RPM / 60.0 * MOTOR_POLSOS_PER_REV     // Màxim rpm / 60s/min * pols/rev
*/
// Bernat 6V 100RPM Motors
#define MOTOR_REDUCCIO          74.83
#define MOTOR_POLSOS_PER_REV    11.0 * 4 * MOTOR_REDUCCIO                           // Polsos simples * 4 canvis/simple * reducció
#define MOTOR_POLSOS_PER_CM     MOTOR_POLSOS_PER_REV / PERIMETRE_RODES_CM           // Polsos/rev / cm/rev
#define MOTOR_VEL_MAX_RPM       100     
#define MOTOR_VEL_MAX_CM_S      MOTOR_VEL_MAX_RPM / 60.0 * PERIMETRE_RODES_CM       // Màxim rpm / 60s/min / cm/rev
#define MOTOR_VEL_MAX_POLS_S    MOTOR_VEL_MAX_RPM / 60.0 * MOTOR_POLSOS_PER_REV     // Màxim rpm / 60s/min * pols/rev

// Alimentació
#define RES_I_SENSE             1780
#define RATIO_I_SENSE_V         17700.0 / RES_I_SENSE                               // Corrent de la bateria (A) / lectura (V). A les faltes augmenta uns 0,5 V
#define RATIO_SENSE_ALM         (249.0 + 10.0) / 10.0
// Temperatura
#define  TEMP_COEFICIENT_MV      10.0                                                // Coeficient del sensor de temperatura, en mV/ºC
#define TEMP_OFFSET_ZERO_MV     500                                                 // Sortida del sensor de temperatura a 0 ºC

// ESP1
// Motors
#define GPIO_DISABLE_MOTORS 22
#define GPIO_INA1           13
#define GPIO_INA2           14
#define GPIO_INB1           33
#define GPIO_INB2           19
#define GPIO_INC1           16
#define GPIO_INC2           12
#define GPIO_IND1           21
#define GPIO_IND2           32
#define GPIO_ENCA1          23
#define GPIO_ENCA2          5
#define GPIO_ENCB1          26
#define GPIO_ENCB2          18
#define GPIO_ENCC1          17
#define GPIO_ENCC2          4
#define GPIO_ENCD1          27
#define GPIO_ENCD2          25
// ADC
#define GPIO_ADC_1          34
#define GPIO_ADC_2          35
// Corrent de bat.
#define GPIO_I_SENSE        36

// ESP2
// Servos
#define GPIO_SERVO1         5
#define GPIO_SERVO2         13
#define GPIO_SERVO3         23
#define GPIO_SERVO4         14
#define GPIO_SERVO5         33
#define GPIO_SERVO6         19
#define GPIO_SERVO7         18
#define GPIO_SERVO8         26
// Tires LED
#define GPIO_LED1           16
#define GPIO_LED2           4
// Ventiladors
#define GPIO_VENT1          17
#define GPIO_VENT2          12
// Lidar
#define GPIO_ENC_LA         32
#define GPIO_ENC_LB         25
#define GPIO_MOT_L          27
// Gpio expander
#define GPIO_INT_GPIO       35
// I2C
#define GPIO_SDA            21
#define GPIO_SCL            22
// Temperatura
#define GPIO_TEMP           34
// Tensió d'alimentació
#define GPIO_SENSE_ALM      36
// Enable bateria
#define GPIO_BAT_EN         39

#endif