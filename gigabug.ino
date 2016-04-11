/**
   gigabug.ino
   Gigatron motor control debugging Arduino code.

   @author  Bayley Wang       <bayleyw@mit.edu>
   @author  Syler Wagner      <syler@mit.edu>
   @author  Chris Desnoyers   <cjdesno@mit.edu>
   @author  Daniel Gonzalez   <dgonz@mit.edu>

   @date    2016-03-12    syler   creation with custom Arduino debugging message formats

 **/

//old servo: 5 pwm, 4 dir, a0 pot
//new servo: 5 pwm1, 6 pwm2, a0 pot

#include "classes.h"
#include "commander.h"
#include "context.h"
#include "isr.h"
#define USE_USBCON
#include <ros.h>
//#include <ros/node_handle.h>
//#include <ArduinoHardware.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>   //$ for steering odometry stuff and mode
#include <std_msgs/UInt16.h>    //$ for mode switching

//$ debugging messages
#include <gigatron/Radio.h>
#include <gigatron/Steering.h>
#include <gigatron/Motors.h>



#define LOOP_INTERVAL 10
#define S_LOOP_INTERVAL 100

//$ steering pot calibration
int minADU = 463; //462, all right
int midADU = 537; //$ value at zero steering angle
int maxADU = 587; //638, all left

//$ pin numbers
int lPwm = 9;
int rPwm = 10;
int lRev = 30; //pin 30 left reverse
int rRev = 31; //pin 31 right reverse

ros::NodeHandle nh;       //$ node handle

// JetsonCommander(ros::NodeHandle *nh);
JetsonCommander jc(&nh);  //$ Jetson commander

//PIDController lSp(2, 1, 1, 255, 0);
//PIDController rSp(2, 1, 1, 255, 0);

PIDController lSp(50, 0, 1, 250, 0);
PIDController rSp(20, 0, 1, 250, 0);

PIDController pPos(150, 0, 15, 255, -255); //250, 1, 50

gigatron::Radio radio_msg;
gigatron::Steering steer_msg;
gigatron::Motors mot_msg;


void CmdCallback(const geometry_msgs::Vector3& cmd) {

  jc._angle = cmd.x;
  jc._left_vel = cmd.y;
  jc._right_vel = cmd.z;
}

/*$ Swith between radio RC and autonomous/Jetson RC mode.
*/
void SwitchCallback(const std_msgs::UInt16& mode) {

  jc._autonomous = mode.data;
  if (mode.data == 2) { //$ set pins for fully autonomous throttle
    //digitalWrite(lRev, HIGH);
    //digitalWrite(rRev, HIGH);
  }
  /*
    if (mode.data == 0) {
     jc._jetsonMode = false;
     jc._semiautomaticMode = false;
    }
    else if (mode.data == 2) {
     jc._jetsonMode = true;
     jc._semiautomaticMode = false;
    }
    else if (mode.data == 1) { //$ manual throttle, autonomous steering
     jc._jetsonMode = false;
     jc._semiautomaticMode = true;
    }*/
}

/*$ Set PID controller gains for both drive motors with a
  Vector3 ROS message (kp, ki, kd) published on the /gains
  topic.
*/
void GainsCallback(const geometry_msgs::Vector3& gain) {
  long kp = (long) gain.x;
  long ki = (long) gain.y;
  long kd = (long) gain.z;
  lSp.ResetGains(kp, ki, kd);
  rSp.ResetGains(kp, ki, kd);
}

void setup() {

  /*$
     For some reason rosserial_arduino is very particular about the baud
     rate setting, and will break in confusing ways if you don't set it
     right. The value below should match that in the launch file for
     Arduino control, in gigatron/launch/arduino_control.launch
         <param name="baud" value="<BAUD>"/>

     If it breaks, try one of these:
     300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, or 115200
     Do not try: 14400, 28800 (these will break)
   * */

   /*$
     For some reason rosserial_arduino breaks if you do both 
     Serial.begin(<BAUD>) and nh.initNode(). So either do:
     1. Serial.begin(<BAUD>)
     2. nh.getHardware()->setBaud(38400);
        nh.initNode();
     Both of the two options seem to work equally well.
   * */
    Serial.begin(115200);
//
//  Serial.begin(19200);
  //nh.getHardware()->setBaud(460800);
////
  //nh.initNode();

  //$ set up publishers
  ros::Publisher radio_pub("arduino/radio", &radio_msg);
  nh.advertise(radio_pub);
  ros::Publisher mot_pub("arduino/motors", &mot_msg);
  nh.advertise(mot_pub);
  ros::Publisher steer_pub("arduino/steering", &steer_msg);
  nh.advertise(steer_pub);

  //$ set up subscribers
  ros::Subscriber<geometry_msgs::Vector3> sub("control", CmdCallback);
  nh.subscribe(sub);
  ros::Subscriber<std_msgs::UInt16> switchsub("switch", SwitchCallback);
  nh.subscribe(switchsub);
  ros::Subscriber<geometry_msgs::Vector3> gainsub("gains", GainsCallback);
  nh.subscribe(gainsub);

  
  // RCDecoder(int interrupt, int minV, int maxV);
  RCDecoder pos(0, 984, 1996);
  //Was 1480, expanded to add reverse
  RCDecoder sp(1, 1020, 1990);
  //Was 1480, expanded to add reverse
  RCDecoder kill(2, 996, 1988);

  // SpeedSensor(int interrupt, int poles, int interval);
  SpeedSensor left(5, 14, S_LOOP_INTERVAL);
  SpeedSensor right(4, 14, S_LOOP_INTERVAL);

  // DCServo(int pwmPin1, int pwmPin2, int posPin);
  DCServo servo(5, 6, A0);

  // DCServo::ConfigSensor(int minV, int maxV);
  servo.ConfigPot(minADU, midADU, maxADU);
  RCCommander rc(&sp, &pos, &kill);

  /* Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          PIDController *lSp, PIDController *rSp,
          PIDController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          gigatron::Radio *radio_msg,
          ros::Publisher *radio_pub,
          gigatron::Steering *steer_msg,
          ros::Publisher *steer_pub,
          gigatron::Motors *mot_msg,
          ros::Publisher *mot_pub
          ) */
  Context context(&rc, &servo, &left, &right, lPwm, rPwm, lRev, rRev, &lSp, &rSp, &pPos, &nh, &jc, &radio_msg, &radio_pub, &steer_msg, &steer_pub, &mot_msg, &mot_pub);

  // Context::ConfigureLoop(int sInterval, int pInterval);
  context.ConfigureLoop(S_LOOP_INTERVAL, LOOP_INTERVAL);
  TCCR3B &= ~7;
  TCCR3B |= 2;


  context.Start(); // the actual looping happens here

}

void loop() {
}

