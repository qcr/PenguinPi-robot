#include "motor.h"
#include "global.h"

void
motor_velocity_control(Motor *motor)
{
    // Scale the set point to match the non-pid commands
    // motor control interval is 20ms
    // setpoint after scaling is the number of ticks per time step (of the loop)
    int16_t vdmd = motor->velocity_dmd/5;

    // estimate velocity (encs/interval)
    motor->velocity = motor->position - motor->position_prev;
    motor->position_prev = motor->position;

    // Error
    motor->verror = vdmd - motor->velocity;
    motor->verrorsum += motor->verror;

    // PI velocity controller
    motor->command = motor->Kvp * motor->verror +
                     motor->Kvi * motor->verrorsum;

    /*
    debugmessage("sp %d, dt %d, e %d, u %d, mc0 %d, mc %d", 
        setPoint, deltaDegrees, pid->error, pid->output, mc0, pid->command);
        */

    // Clip the commands
    if (motor->command > 100)
        motor->command = 100;
    else if (motor->command < -100)
        motor->command = -100;

#ifdef notdef
    // If the input is 0, reset the controller
    if (setPoint == 0) {
        motor->command = 0;
        motor->error = 0;
        motor->error_prev = 0;
    }
#endif
}

void
motor_init(Motor *motor, uint8_t id)
{
    motor->command = 0;
    motor->velocity = 0;
    motor->verror = 0;
    motor->velocity_dmd = 0;
    motor->verrorsum = 0;
    motor->Kvp = 0;
    motor->Kvi = 1;
    motor->id = id;
    motor->encoderMode = 0;
}

#ifdef notdef
// PIC: not sure I believe this approach, just sample B on an A edge...
// encoder state transition lookup tables
static int8_t 	mAQuadTable[4][4] = {{ 0, +1, -1,  2},
							 {-1,  0,  2, +1},
							 {+1,  2,  0, -1},
							 { 2, -1, +1,  0}};

static int8_t  mBQuadTable[4][4] = {{ 0, -1, +1,  2},
							 {+1,  0,  2, -1},
							 {-1,  2,  0, +1},
							 { 2, +1, -1,  0}};
#endif

void 
motor_encoder_update( Motor *motor, uint8_t encA, uint8_t encB )
{
	
    switch(motor->encoderMode) {
    case 0: //single encoder mode, on pin 1
		if ( motor->encA_prev ^ encA ){
			if( motor->command > 0){
				motor->position++;
			}else if( motor->command < 0 ){
				motor->position--;
			}
			motor->encA_prev = encA;
        }
        break;

#ifdef notdef
    case 1: { //standard quadrature
		uint8_t lastEncSum 	= (motor->enc1PinState<<1)|(motor->enc2PinState);
		uint8_t encSum 		= (enc_1_val<<1)|(enc_2_val);
		int8_t  effect;
		
		if ( motor->which_motor == 1 ) {
			//Motor B
			effect 			= mBQuadTable[lastEncSum][encSum];
		}
		else {
			//Motor A
			effect 			= mAQuadTable[lastEncSum][encSum];
		}
		
		motor->position 	+= effect;
	
		motor->enc1PinState = enc_1_val;
		motor->enc2PinState = enc_2_val;
        break;
    }
    case 2: { //x4 counting (xor'ed both channels)
		uint8_t x4 = enc_1_val ^ enc_2_val;
		if(motor->enc1PinState ^ x4){
			if(motor->dir == 1){
				motor->position++;
				motor->lastDir = 1;
			}else if(motor->dir == -1){
				motor->position--;
				motor->lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if(motor->lastDir == 1) 		motor->position++;
				else if(motor->lastDir == -1) 	motor->position--;
			}
			motor->enc1PinState = x4;
		}
	}
#endif
        }
}

#ifdef notdef
void fn_dbg_motor ( Motor *motor ){
	char 	fstring[32];
	
	if ( motor->dir == -1 ) 	uart_puts_P("  dir: -1\n");
	else if ( motor->dir == 1 )	uart_puts_P("  dir:  1\n");
	else 						uart_puts_P("  dir:  0\n");					
	
	sprintf(fstring, "  enc: %6d\n", motor->position);
	uart_puts(fstring);
	uart_puts(fstring);
	uart_puts(fstring);			
	sprintf(fstring, "  dps: %6d\n", motor->velocity_dmd);
	uart_puts(fstring);		
	
}
#endif
#ifdef notdef
int16_t 
motor_position_control(int16_t setPoint, Motor *motor)
{
	//based upon Application Note AVR221
	int16_t error = setPoint - motor->degrees;

	int16_t pTerm, dTerm;
	int32_t iTerm;
	
	//P term, also limit overflow
	if(error > motor->maxError)			pTerm = INT16_MAX;
	else if(error < -motor->maxError)	pTerm = -INT16_MAX;
	else								pTerm = motor->gainP * error;
	
	//I term, also limit overflow
	int32_t newSum = motor->errorSum + error;
	if(newSum > motor->maxErrorSum){
		motor->errorSum = motor->maxErrorSum;
		iTerm = INT32_MAX/2;
	}else if(newSum < -motor->maxErrorSum){
		motor->errorSum = -motor->maxErrorSum;
		iTerm = -(INT32_MAX/2);
	}else{
		motor->errorSum = newSum;
		iTerm = motor->gainI * motor->errorSum;
	}

	//D term //put this on a slower interval
	dTerm = motor->gainD * (motor->lastVal - motor->degrees);
	
	motor->lastVal = motor->degrees;
	
	int32_t result = (pTerm + iTerm + dTerm)/PID_SCALE;
	
	if(result > INT16_MAX) result = INT16_MAX;
	else if(result < -INT16_MAX) result = -INT16_MAX;
	
	return (int16_t)result;		
}
#endif
