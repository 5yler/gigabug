/**
 * context.h
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 * @date    2016-01-10    syler   moved PID controller to separate class
 *
 **/

#ifndef __CONTEXT_H
#define __CONTEXT_H

#include <Arduino.h>
#include "isr.h"
#include "classes.h"
#include "commander.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

//$ motor commands
#include <gigatron_hardware/MotorCommand.h>

//$ debugging messages
#include <gigatron_hardware/Radio.h>
#include <gigatron_hardware/Steering.h>
#include <gigatron_hardware/Motors.h>
#include <std_msgs/UInt8.h>

class Context {
public:
  Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          int lRev, int rRev,
          PIDController *lSp, PIDController *rSp,
          PIDController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          AutonomousLED *autonomousled,
          gigatron_hardware::Radio *radio_msg,
          ros::Publisher *radio_pub,
          gigatron_hardware::Steering *steer_msg,
          ros::Publisher *steer_pub,
          gigatron_hardware::Motors *mot_msg,
          ros::Publisher *mot_pub,
          std_msgs::UInt8 *stop_msg,
          ros::Publisher *stop_pub
          );
  void ConfigureLoop(int sInterval, int pInterval, int pubInterval);
  void Start();

private:
  Commander *_commander;
  DCServo *_servo;
  SpeedSensor *_left, *_right;
  int _lPwm, _rPwm, _lRev, _rRev;
  PIDController *_lSp, *_rSp, *_pos;
  int _sInterval, _pInterval, _pubInterval;
  
  unsigned long _last_st, _last_pt, _last_pub;

  //$
  ros::NodeHandle *_nh;
  JetsonCommander *_jcommander;

  AutonomousLED *_autonomousled;

  gigatron_hardware::Radio *_radio_msg;
  ros::Publisher *_radio_pub;

  gigatron_hardware::Steering *_steer_msg;
  ros::Publisher *_steer_pub;

  gigatron_hardware::Motors *_mot_msg;
  ros::Publisher *_mot_pub;

  std_msgs::UInt8 *_stop_msg;
  ros::Publisher *_stop_pub;

};

#endif


