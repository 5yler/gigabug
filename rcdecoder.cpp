/**
 * rcdecoder.cpp
 * Gigatron motor control Arduino code for RC decoder.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

RCDecoder::RCDecoder(int interrupt, int minV, int maxV) {
  _interrupt = interrupt;
  _minV = minV;
  _maxV = maxV;
  
  
  if (_interrupt == RC_STEERING_INTERRUPT) {
    attachInterrupt(RC_STEERING_INTERRUPT, RCSteeringISR, CHANGE);
  } else if (_interrupt == RC_THROTTLE_INTERRUPT) {
    attachInterrupt(RC_THROTTLE_INTERRUPT, RCThrottleISR, CHANGE);
  } else if (_interrupt == RC_KILL_INTERRUPT) {
    attachInterrupt(RC_KILL_INTERRUPT, RCKillISR, CHANGE);
  }
}

unsigned char RCDecoder::GetVal() {
  long pw;
  if (_interrupt == RC_STEERING_INTERRUPT) {
    pw = _pw0_us;
  } else if (_interrupt == RC_THROTTLE_INTERRUPT) {
    pw = _pw1_us;
  }if (_interrupt == RC_KILL_INTERRUPT) {
    pw = _pw2_us;
  }
  
  //Serial.println(pw); //RC decoder vals
  //dp(pw);
  pw = (pw - _minV) << 8;
  pw /= (_maxV - _minV);
  if (pw < 0) pw = 0;
  if (pw > 255) pw = 255;

  //$ convert from 0-255 range to 0-180 range
  pw *= 180;
  pw /= 255;
  return (unsigned char) pw;
}

