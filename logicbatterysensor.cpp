/**
 * logicbatterysensor.cpp
 * Gigatron sensor code for logic battery level.
 *
 * @author Chris Desnoyers <cjdesno@mit.edu>
 *
 * @date 2017-01-21
 *
 **/

 #include <Arduino.h>
 #include "classes.h"

 #define ADC_TO_PIN_VOLTAGE (5./1024.)

LogicBatterySensor::LogicBatterySensor(int sense_pin, long r_top, long r_bottom) {
  _sense_pin = sense_pin;
  _r_top = r_top;
  _r_bottom = r_bottom;
  pinMode(_sense_pin, INPUT);
}

double LogicBatterySensor::GetLogicVoltage() {
  int pin_reading = analogRead(sense_pin);
  return pin_reading * ADC_TO_PIN_VOLTAGE * (r_top + r_bottom)/((double) r_bottom);
}