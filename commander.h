/**
 * commander.h
 * Gigatron motor control Arduino code Jetson and RC commanders.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "classes.h"
#include "isr.h"
#include <ros.h>

class Commander {
public:
  virtual int GetLeftRPMCmd() {return 0;}
  virtual int GetRightRPMCmd() {return 0;}
  virtual unsigned char GetAngleCmd() {return 0;}
  virtual unsigned char GetKillCmd() {return 0;}
};

class RCCommander: public Commander {
public:
  RCCommander(RCDecoder *sp, RCDecoder *pos,  RCDecoder *kill);
  int GetLeftRPMCmd();
  int GetRightRPMCmd();
  unsigned char GetAngleCmd();
  unsigned char GetKillCmd();

private:
  RCDecoder *_sp, *_pos, *_kill;
};

class JetsonCommander: public Commander { //$ wooo
public:
  JetsonCommander(ros::NodeHandle *nh);
  int GetLeftRPMCmd();
  int GetRightRPMCmd();
  unsigned char GetAngleCmd();
  // double _angle;
  unsigned char _angle;
  long _rpm_left, _rpm_right;
  
  int _autonomous;
/*
  boolean _jetsonMode; //$ true for Jetson control, false for RC
  boolean _semiautomaticMode; //$ true for Jetson control, false for RCprivate:
*/
  ros::NodeHandle *_nh;
};
  
#endif


