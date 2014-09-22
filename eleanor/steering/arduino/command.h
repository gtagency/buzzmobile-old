
#ifndef __COMMAND_H
#define __COMMAND_H

#include "Arduino.h"

struct CommandState {
  /** Input state (from environment).  Setting these will be ignored. */
  int sensorVal;
  int currentVal;
  int cmdByte;
  
  /** Settable state, based on command function */
  boolean brakeReleased;
};

struct Command {
  int cmdByte;
  void (*doCommand)(CommandState& state);
};

extern Command commands[];
extern int commandCount;

Command *findCommand(int cmdByte);
#endif
