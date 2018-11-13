#ifndef __motor_h__
#define __motor_h__

#include <stdint.h>

typedef struct {
    // motor id: 0 for MotorL, 1 for MotorR
	uint8_t				id;		
    // mode 0: single encoder, mode 1: quadrature, mode 2: x4 counting (xor quadrature)
	int8_t      encoderMode;		

    // encoder 
	uint8_t 	encA_prev;
	uint8_t 	encB_prev;
	int16_t 	position;  			// the "encoder" value
 
    int16_t velocity;

	int16_t 			velocity_dmd;
    uint8_t             controlMode;    // not quite sure what this might be for

    // GAINS
    int16_t Kp;
    int16_t Ki;
    int16_t Kd;
    int16_t Kvp;
    int16_t Kvi;

    int16_t position_prev;

    // ERROR
    int16_t verror;
    int16_t error_prev;
    int16_t verrorsum;

    // Out
    int16_t command;
} Motor;

// exported functions
void motor_velocity_control(Motor *motor);
void motor_position_control(Motor *motor, int16_t cmd);
void motor_init(Motor *motor, uint8_t id);
void motor_encoder_update(Motor *motor, uint8_t encA, uint8_t encB);

#endif
