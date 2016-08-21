/**
 * isr.h
 * Gigatron motor control Arduino code for interrupts.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-01-10    syler   renamed from shared.h to isr.h
 * @date    2016-03-27    syler   define encoder interrupts and pins, fixed ISRs for left and right encoders
 *
 **/

#ifndef __ISR_H
#define __ISR_H

//$ left encoder
#define R_ENCODER_INTERRUPT 4
#define R_ENCODER_PIN_A 19
#define R_ENCODER_PIN_B 17

//$ right encoder
#define L_ENCODER_INTERRUPT 5
#define L_ENCODER_PIN_A 18
#define L_ENCODER_PIN_B 16
#define L_ENCODER_REVERSED //$ define that the left encoder is reversed to handle directionality

//$ motors
#define L_MOTOR_PWM_PIN 9
#define L_MOTOR_REVERSE_PIN 30

#define R_MOTOR_PWM_PIN 10
#define R_MOTOR_REVERSE_PIN 31

//$ steering servo
#define STEERING_PWM_PIN_1 5
#define STEERING_PWM_PIN_2 6
#define STEERING_POT_PIN A0

//$ RC decoder
#define RC_STEERING_INTERRUPT 0
#define RC_STEERING_PIN 2

#define RC_THROTTLE_INTERRUPT 1
#define RC_THROTTLE_PIN 3

#define RC_KILL_INTERRUPT 2
#define RC_KILL_PIN 21

//$ Mode indicator LED
#define AUTONOMOUS_LED_PIN 37

extern volatile unsigned long _pw0_us, _pw1_us, _pw2_us;
extern volatile unsigned long _pw0_last_t, _pw1_last_t, _pw2_last_t;
extern volatile long _ticks_left, _ticks_right;		//$ number of ticks for each encoder

void RCSteeringISR();
void RCThrottleISR();
void RCKillISR();
void LeftEncoderISR();   //$ left encoder interrupt service routine
void RightEncoderISR();  //$ right encoder interrupt service routine

#endif


