
#ifndef __MOTORCONTROL_H
#define __MOTORCONTROL_H

/**
 * Channel configurations.
 * NOTE: this is not specific to car steering, and could be reused in other Arduino MC applications
 */

#include "pins_arduino.h"

struct Channel {
  int sensorAPin;
  int dirPin;
  int motorPin;
  int brakePin;
  int curSensingAPin;
};

//NOTE: this couplies a specific analog in to a specific channel (In2 -> A2 -> ChA, In3 -> A3 -> ChB)
Channel ChA = {
  A2, 12, 3,  9, A0};
Channel ChB = {
  A3, 13, 11, 8, A1};

#endif

