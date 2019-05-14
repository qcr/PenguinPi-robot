/*
 * PCA6416A.h
 *
 * Created: 19/12/2016 12:24:20 PM
 *  Author: n8855897
 */ 


#ifndef PCA6416A_H_
#define PCA6416A_H_

#define PCA6416A_0 0b01000000
#define PCA6416A_1 0b01000010

//Register selection commands
#define INPUT_0 0x00
#define INPUT_1 0x01
#define OUTPUT_0 0x02
#define OUTPUT_1 0x03
#define P_INVERT_0 0x04
#define P_INVERT_1 0x05
#define CONFIG_0 0x06
#define CONFIG_1 0x07

//Segments
#define SEGMENTA_0 0b00100000
#define SEGMENTB_0 0b10000000
#define SEGMENTC_0 0b00010000
#define SEGMENTD_0 0b00000100
#define SEGMENTE_0 0b00000001
#define SEGMENTF_0 0b00000010
#define SEGMENTG_0 0b00001000
#define SEGMENTDP_0 0b01000000

#define SEGMENTA_1 0b00010000
#define SEGMENTB_1 0b01000000
#define SEGMENTC_1 0b00100000
#define SEGMENTD_1 0b00000010
#define SEGMENTE_1 0b00000001
#define SEGMENTF_1 0b00000100
#define SEGMENTG_1 0b00001000
#define SEGMENTDP_1 0b10000000

#define DIGIT0_0 SEGMENTA_0|SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0
#define DIGIT0_1 SEGMENTB_0|SEGMENTC_0
#define DIGIT0_2 SEGMENTA_0|SEGMENTB_0|SEGMENTG_0|SEGMENTE_0|SEGMENTD_0
#define DIGIT0_3 SEGMENTA_0|SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTG_0
#define DIGIT0_4 SEGMENTB_0|SEGMENTC_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_5 SEGMENTA_0|SEGMENTC_0|SEGMENTD_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_6 SEGMENTA_0|SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_7 SEGMENTA_0|SEGMENTB_0|SEGMENTC_0
#define DIGIT0_8 SEGMENTA_0|SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_9 SEGMENTA_0|SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_A SEGMENTA_0|SEGMENTB_0|SEGMENTC_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_B SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_C SEGMENTA_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0
#define DIGIT0_D SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTG_0
#define DIGIT0_E SEGMENTA_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_F SEGMENTA_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0

#define DIGIT0_H SEGMENTB_0|SEGMENTC_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_I SEGMENTE_0|SEGMENTF_0
#define DIGIT0_J SEGMENTB_0|SEGMENTC_0|SEGMENTD_0
#define DIGIT0_L SEGMENTD_0|SEGMENTE_0|SEGMENTF_0
#define DIGIT0_N SEGMENTC_0|SEGMENTE_0|SEGMENTG_0
#define DIGIT0_O SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTG_0
#define DIGIT0_P SEGMENTA_0|SEGMENTB_0|SEGMENTE_0|SEGMENTF_0|SEGMENTG_0
#define DIGIT0_R SEGMENTE_0|SEGMENTG_0
#define DIGIT0_U SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTE_0|SEGMENTF_0
#define DIGIT0_Y SEGMENTB_0|SEGMENTC_0|SEGMENTD_0|SEGMENTF_0|SEGMENTG_0


#define DIGIT1_0 SEGMENTA_1|SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1
#define DIGIT1_1 SEGMENTB_1|SEGMENTC_1
#define DIGIT1_2 SEGMENTA_1|SEGMENTB_1|SEGMENTG_1|SEGMENTE_1|SEGMENTD_1
#define DIGIT1_3 SEGMENTA_1|SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTG_1
#define DIGIT1_4 SEGMENTB_1|SEGMENTC_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_5 SEGMENTA_1|SEGMENTC_1|SEGMENTD_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_6 SEGMENTA_1|SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_7 SEGMENTA_1|SEGMENTB_1|SEGMENTC_1
#define DIGIT1_8 SEGMENTA_1|SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_9 SEGMENTA_1|SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_A SEGMENTA_1|SEGMENTB_1|SEGMENTC_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_B SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_C SEGMENTA_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1
#define DIGIT1_D SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTG_1
#define DIGIT1_E SEGMENTA_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_F SEGMENTA_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1

#define DIGIT1_H SEGMENTB_1|SEGMENTC_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_I SEGMENTE_1|SEGMENTF_1
#define DIGIT1_J SEGMENTB_1|SEGMENTC_1|SEGMENTD_1
#define DIGIT1_L SEGMENTD_1|SEGMENTE_1|SEGMENTF_1
#define DIGIT1_N SEGMENTC_1|SEGMENTE_1|SEGMENTG_1
#define DIGIT1_O SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTG_1
#define DIGIT1_P SEGMENTA_1|SEGMENTB_1|SEGMENTE_1|SEGMENTF_1|SEGMENTG_1
#define DIGIT1_R SEGMENTE_1|SEGMENTG_1
#define DIGIT1_U SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTE_1|SEGMENTF_1
#define DIGIT1_Y SEGMENTB_1|SEGMENTC_1|SEGMENTD_1|SEGMENTF_1|SEGMENTG_1

#endif /* PCA6416A_H_ */