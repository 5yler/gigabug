/**
 * dcservo.cpp
 * Gigatron motor control Arduino code for steering servo.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

DCServo::DCServo(int pwmPin) {
  _servo.attach(pwmPin);
  _last_pos = 90;
}

// void DCServo::ConfigPot(int minV, int midV, int maxV) {
//   _minV = minV;
//   _midV = midV;
//   _maxV = maxV;
// }

//$ 90 is center, 0 is turned right, 180 is far left
void DCServo::SetPos(int pos) {
  _servo.write(pos);
  _last_pos = pos;
}

unsigned char DCServo::GetPos() {
  /*
  long adu = analogRead(_posPin);
  //dp(adu); //$ uncomment for pot calibration
  long tmp = (adu - _minV) << 8 ;;
  tmp /= (_maxV - _minV);
  if (tmp < 0) tmp = 0;
  if (tmp > 255) tmp = 255;
  //dp(tmp);
  return (unsigned char) tmp;
  */
  return (unsigned char) _last_pos;
}

//$ takes pot limits and middle value and linearizes the output
/* DGonz's measurements as of 9:49pm 9/24/2015
 * 439 in, 0 out
 * 549 in, 127 out
 * 622 in, 255 out
 */
// unsigned char DCServo::GetPosLinearized() {
//   long adu = analogRead(_posPin);
//   //dp(adu); //$ uncomment for pot calibration
//   long tmp;
//   if (adu < _midV) {
//     tmp = (adu - _minV) << 7 ;;
//     tmp /= (_midV - _minV);
//   }
//   else {
//     tmp = (adu - _midV) << 7 ;
//     tmp /= (_maxV - _midV);
//     tmp += 127;
//   }
//   if (tmp < 0) tmp = 0;
//   if (tmp > 255) tmp = 255;
//   //dp(tmp);
//   unsigned char res = (unsigned char) tmp;
// //  dp (res);
//   return res;
// }

