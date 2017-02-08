/********************************************************************
  Software License Agreement (BSD License)

  Copyright (c) 2017, Cult Classic Racing.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/**
 * pidcontroller.cpp
 * Gigatron motor control Arduino code for PID controller.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "isr.h"
#include "classes.h"

PIDController::PIDController(long kp, long ki, long kd, long out_max, long out_min) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _last_in = 0;
  _integral = 0;
  _out_max = out_max;
  _out_min = out_min;
}

void PIDController::ResetGains(long kp, long ki, long kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _last_in = 0;
  _integral = 0;
}

void PIDController::ResetIntegrator() {
  _last_in = 0;
  _integral = 0;
}

int PIDController::Update(int ref, int in) {
  int error = ref - in;

  _integral += error;
  
  long tmp = _integral * _ki >> 8;
  if (tmp > _out_max) _integral = (_out_max << 8) / _ki;
  if (tmp < _out_min) _integral = (_out_min << 8) / _ki;
  
  int deriv = _last_in - in;
  _last_in = in;
  tmp = _ki * _integral + _kp * error + _kd * deriv;
  tmp = tmp >> 8;
  if (tmp > _out_max) tmp = _out_max;
  if (tmp < _out_min) tmp = _out_min;
  return tmp;
}

