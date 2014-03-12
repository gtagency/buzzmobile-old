/*************************************************************
 Steering control for the buzzmobile car.  This was originally
 built for a Arduino Uno r3 with MotorShield, controlling a
 linear actuator hooked up to Channel A.
 
 Command: 
 
 A host controls the steering by issuing one or more commands,
 which changes the state of the controller, and observes the
 state of the linear actuator by reading the response.
 The host communicates with the controller over a 9600 baud
 serial connection.

 Available commands:
   l - turn left
   r - turn right
   s - full stop (start state)

 When the controller receives a recognized command, it sets the
 state of the controller until the next command is received.
 
 Response:
 
 The controller runs on a 20ms loop; on each iteration, the
 controller reads its sensors and attempts to process a command.

 Response format (given by the following regexp):
  s[0-9]+u[0-9]+(c[lrs?])? 
 
 The response is constructed as described above; command is
 optional and only sent back if a command was processed on that
 iteration.  Recognized commands are reported as their command
 Unrecognized commands are reported as ?.

 Configuration:
 
 This controller is configured to accept steering position
 over the analog in port, and limit steering to within
 acceptable positions.  To adjust these, change maxLeft
 and maxRight in the code.
 
*************************************************************/

#include "motorcontrol.h"
#include "command.h"

//Teensy 2.0
//B2 = 2 = DIR
//B7 = 4 = PWM
//F7 = 19(A2) = sensor
Channel teensy = {A2, 2, 4, 0, 0};
Channel *ch = &teensy;

/** Max left and right sensor readings */
//NOTE: values in raw readings from sensor.
// assumes right = smaller numbers, left = larger numbers
int maxRight = 330;
int maxLeft = 730;


/** Steering commands */
void turnLeft (CommandState& state);
void turnRight(CommandState& state);
void fullStop (CommandState& state);

Command commands[] = {
  {'l', turnLeft},
  {'r', turnRight},
  {'s', fullStop}
};

int commandCount = sizeof(commands)/sizeof(commands[0]);

void turnLeft(CommandState& state) {
  if (state.sensorVal > maxLeft) {
    fullStop(state);
    return;
  }
  //release the break, if it hasnt been released
  if (!state.brakeReleased) {
    digitalWrite(ch->brakePin, LOW);
    state.brakeReleased = true;
  }
  //establish forward direction of channel and spin motor at full speed
  //TODO: support partial speeds
  digitalWrite(ch->dirPin, LOW);
  analogWrite(ch->motorPin, 255);
}

void turnRight(CommandState& state) {
  if (state.sensorVal < maxRight) {
    fullStop(state);
    return;
  }
  //release the break, if it hasnt been released
  if (!state.brakeReleased) {
    digitalWrite(ch->brakePin, LOW);
    state.brakeReleased = true;
  }

  //establish forward direction of channel and spin motor at full speed
  //TODO: support partial speeds
  digitalWrite(ch->dirPin, HIGH);
  analogWrite(ch->motorPin, 255);
}

void fullStop(CommandState& state) {
  analogWrite(ch->motorPin, 0);
  digitalWrite(ch->brakePin, HIGH);
  state.brakeReleased = false;
}

void setup() {
  
  Serial.begin(9600);
  
  //Setup pins for channel
//  pinMode(ch->sensorPinA, INPUT);
  pinMode(ch->dirPin, OUTPUT);
  pinMode(ch->motorPin, OUTPUT);
  pinMode(ch->brakePin, OUTPUT);
  
  //write the number of available commands
  Serial.write(0x30 + commandCount);
  Serial.write(" commands\n");
}

CommandState state = {0};

void loop() {

  //report sensor state
  state.sensorVal  = analogRead(ch->sensorAPin);
  state.currentVal = analogRead(ch->curSensingAPin);

  char buf[32] = {0};
  sprintf(buf, "s%du%d", state.sensorVal, state.currentVal);
  Serial.write(buf);

  int inByte = Serial.read();
  //respond to state
  if (inByte != -1 && state.cmdByte != inByte) {
    Command *cmd = findCommand(inByte);
    Serial.write('c');
    if (cmd) {
      state.cmdByte = inByte;
      Serial.write(inByte);
      cmd->doCommand(state);
    } else {
      //? means we didn't recognize the command
      Serial.write("?");
    }
  }
  
  if ((state.sensorVal < maxRight && state.cmdByte == 'r') || 
      (state.sensorVal > maxLeft && state.cmdByte == 'l')) {
    fullStop(state);
  }
  Serial.write("\n");
  //this delay appears to be required to prevent the serial port from
  // locking up
  delay(20);
}

