/**
 * context.cpp
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 * @date    2016-01-10    syler   moved PID controller to separate class
 * 
 **/

#include <Arduino.h>
#include "Servo.h"
#include "isr.h"
#include "classes.h"
#include "commander.h"
#include "context.h"
 
Servo leftMotor;
Servo rightMotor;

Context::Context(Commander *commander, DCServo *servo,
  SpeedSensor *left, SpeedSensor *right,
  int lPwm, int rPwm,
  int lRev, int rRev,
  PIDController *lSp, PIDController *rSp,
  PIDController *pos,
  ros::NodeHandle *nh,
  JetsonCommander *jcommander,
  gigatron_hardware::Radio *radio_msg,
  ros::Publisher *radio_pub,
  gigatron_hardware::Steering *steer_msg,
  ros::Publisher *steer_pub,
  gigatron_hardware::Motors *mot_msg,
  ros::Publisher *mot_pub,
  std_msgs::UInt8 *mode_msg,
  ros::Publisher *mode_pub
  ) {
  _commander = commander;
  _servo = servo;
  _left = left;
  _right = right;
  _lPwm = lPwm;
  _rPwm = rPwm;
  _lRev = lRev;
  _rRev = rRev;  
  _lSp = lSp;
  _rSp = rSp;
  _pos = pos;

  _nh = nh; //$ ROS node handle
  _jcommander = jcommander; //$ Jetson commander

  //$ ROS publishers and messages
  _radio_msg = radio_msg; 
  _radio_pub = radio_pub;
  _steer_msg = steer_msg; 
  _steer_pub = steer_pub;
  _mot_msg = mot_msg; 
  _mot_pub = mot_pub;
  _mode_pub = mode_pub;
  
  pinMode(_lPwm, OUTPUT);
  pinMode(_rPwm, OUTPUT);

  // motor controller
  leftMotor.attach(_lPwm);
  rightMotor.attach(_rPwm);
}

/*$ Configure time intervals for speed (drive motor) and 
  position (steering servo) loops.
  @param  sInterval  [ms] speed loop interval 
  @param  pInterval  [ms] position loop interval 
  */
  void Context::ConfigureLoop(int sInterval, int pInterval) {
    _sInterval = sInterval;
    _pInterval = pInterval;
  }


  void Context::Start() {

    //$ clear messages
    _radio_msg->speed_left = 0;
    _radio_msg->speed_right = 0;
    _radio_msg->angle = 128;
    _radio_msg->kill = 0;

    _steer_msg->angle = 128;
    _steer_msg->angle_command = 128;

    _mot_msg->rpm_left = 0;
    _mot_msg->rpm_right = 0;
    _mot_msg->usec_left = 1500;
    _mot_msg->usec_right = 1500;

    _mode_msg->data = 0;

    _last_st = _last_pt = millis();

  //unsigned int oldMode = _jcommander->_autonomous;
    unsigned int oldMode = 2;

    for (;;) {
    _nh->spinOnce(); //$ spin node handle
    
    unsigned long t = millis();
    unsigned long d_st = t - _last_st;
    unsigned long d_pt = t - _last_pt;

        // KILLSWITCH ENGAGE \m/
    if (_commander->GetKillCmd() > 75) {
      if (_jcommander->_autonomous == 0) { //$ RC
        _jcommander->_autonomous = oldMode;
        //$ HALP IT'S GOING IN REVERSE
//        digitalWrite(_lRev, LOW);
        //digitalWrite(_rRev, HIGH); 
        //digitalWrite(_lRev, HIGH); 

      }
    }
    else {
      if (_jcommander->_autonomous > 0) { //$ AUTO or SEMIAUTOMATIC
        oldMode = _jcommander->_autonomous;
      }
      _jcommander->_autonomous = 0;
    }

    //$ left and right speed commands
    int lSpC;
    int rSpC;

      //left and right microsecond write values for motor controller
    unsigned int luSec;
    unsigned int ruSec;
    
    if (d_st > _sInterval) {  //$ speed (drive motor) loop
      //$ left and right speed commands

      int lRPM_sensed = _left->GetRPM();
      int rRPM_sensed = _right->GetRPM();
      //$ get values from RC commander or Jetson commander
      if (_jcommander->_autonomous > 1) { //$ fully autonomous mode

        //$ commanded values
        int lRPM_cmd = _jcommander->GetLeftRPMCmd();
        int rRPM_cmd = _jcommander->GetRightRPMCmd();
        

        //$ update PID controllers
       lSpC = - _lSp->Update(lRPM_cmd, lRPM_sensed);
       rSpC = - _rSp->Update(rRPM_cmd, rRPM_sensed);

      }
      else { //$ RC mode and semiautomatic mode
        lSpC = _commander->GetLeftRPMCmd();
        rSpC = _commander->GetRightRPMCmd();
      }

      //$ convert to motor controller format of
      //$ servo-style timed pulses (1250-1750)
      luSec = (unsigned int) 1500 + lSpC;
      ruSec = (unsigned int) 1500 + rSpC;

      //$ write to motor controller
      leftMotor.writeMicroseconds(luSec);
      rightMotor.writeMicroseconds(ruSec);

      _last_st = t;
      
      //$ write wheel velocities
      _mot_msg->rpm_left = lRPM_sensed;
      _mot_msg->rpm_right = rRPM_sensed;
      _mot_msg->usec_left = ruSec;
      _mot_msg->usec_right = luSec;

      //$ publish message
      _mot_pub->publish(_mot_msg);

      //$ write radio values
      _radio_msg->speed_left = _commander->GetLeftRPMCmd();
      _radio_msg->speed_right = _commander->GetRightRPMCmd();
      _radio_msg->angle = _commander->GetAngleCmd();
      _radio_msg->kill = _commander->GetKillCmd();

      //$ publish radio message
      _radio_pub->publish(_radio_msg);

      //$ publish mode message
      _mode_msg->data = _jcommander->_autonomous;
      _mode_pub->publish(_mode_msg);

    }

    if (d_pt > _pInterval) { //$ position (steering servo) loop
      unsigned char pC;
      if (_jcommander->_autonomous == 0) { //$ RC mode
        pC = _commander->GetAngleCmd();
      }
      else  { //$ mixed mode and fully autonomous mode
        pC = _jcommander->GetAngleCmd();
      }  
      unsigned char pS = _servo->GetPosLinearized();

      int vel = _pos->Update(pC, pS); //$ update PID controller

      //$ command analogWrite/digitalWrite
      _servo->SetVelocity(vel);

      _last_pt = t;

      //$ steering servo position and Hall effect readings
      double servoPWM = (double) pS;
      double steeringAngle = STEERING_ANGLE_RANGE * (servoPWM / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;

      //$ write steering angle and servo PWM command to message
      _steer_msg->angle = pS;
      _steer_msg->angle_command = pC;
      
      //$ publish message
      _steer_pub->publish(_steer_msg);

    }
  }
}



