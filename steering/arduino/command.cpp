
#include "command.h"

void noop(CommandState& state) {}
Command NullCommand = {-1, noop};

Command *findCommand(int cmdByte) {
  Command *found = NULL; //&NullCommand;
  for (int ii = 0; ii < commandCount; ii++) {
    if (commands[ii].cmdByte == cmdByte) {
      found = &commands[ii];
      break;
    }
  }
  return found;
}


