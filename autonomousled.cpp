/**
 * autonomousled.cpp
 * LED indicator code to display Gigatron's current mode of operation.
 *   -On:       Autonomous (_autonomous == 2)
 *   -Blinking: Semiautomatic (_autonomous == 1)
 *   -Off:      Remote Control (_autonomous == 0)
 *
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 **/

#include <Arduino.h>
#include "classes.h"

#define AUTONOMOUS 2
#define SEMIAUTOMATIC 1
#define REMOTE_CONTROL 0

AutonomousLED::AutonomousLED(int led_pin, int *autonomous_mode_ptr, unsigned long period_millis) {
  _led_pin = led_pin;
  _autonomous_mode_ptr = autonomous_mode_ptr;
  _period_millis = period_millis;
  _led_on = false;
  _last_changed = 0;
  pinMode(_led_pin, OUTPUT);
  digitalWrite(_led_pin, LOW);
  this.Update();
}

void AutonomousLED::Update() {
  switch(*autonomous_mode_ptr) {
    case AUTONOMOUS:
      _led_on = true;
      digitalWrite(_led_pin, HIGH);
      break;
    case SEMIAUTOMATIC:
      unsigned long current_time_millis = millis();
      if (current_time_millis - _last_changed >= _period_millis) {
        if (_led_on) {
          _led_on = true;
          digitalWrite(_led_pin, HIGH);
        } else {
          _led_on = false;
          digitalWrite(_led_pin, LOW);
        }
        _last_changed = current_time_millis;
      }
      break;
    case REMOTE_CONTROL:
      _led_on = false;
      digitalWrite(_led_pin, LOW);
      break;
    default:
      break;
}
