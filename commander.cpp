
/**
 * commander.cpp
 * Gigatron motor control Arduino code Jetson and RC commanders.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"
#include "commander.h"


RCCommander::RCCommander(RCDecoder *sp, RCDecoder *pos, RCDecoder *kill) {
  _sp = sp;
  _pos = pos;
  _kill = kill;
}

int RCCommander::GetLeftRPMCmd() {
  int left_command = 2 * (int) _sp->GetVal() - 255;

  //$ check edge cases
  if (left_command > 250) left_command = 250;
  if (left_command < -250) left_command = -250;
  
  return left_command;
}

int RCCommander::GetRightRPMCmd() {
  int right_command = 2 * (int) _sp->GetVal() - 255;

  //$ check edge cases
  if (right_command > 250) right_command = 250;
  if (right_command < -250) right_command = -250;
  
  return right_command;
}

unsigned char RCCommander::GetAngleCmd() {
  unsigned long pw = _pos->GetVal();
  return (unsigned char) pw;
}

unsigned char RCCommander::GetKillCmd() {
  return _kill->GetVal();
}

//$
JetsonCommander::JetsonCommander(ros::NodeHandle *nh) {

  _nh = nh;

/*
  _jetsonMode = true;
  _semiautomaticMode = false;
*/

  _autonomous = 0;
  _estop = false;

  _rpm_left = 0; 
  _rpm_right = 0; 
  _angle = 128;
}

int JetsonCommander::GetLeftRPMCmd() {
  return (int) _rpm_left;
}

int JetsonCommander::GetRightRPMCmd() {
    return (int) _rpm_right;
}

unsigned char JetsonCommander::GetAngleCmd() {

  return _angle;

}



