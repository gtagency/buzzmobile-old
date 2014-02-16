/*************************************************************
 Steering control for the buzzmobile car.  This was originally
 built for a Arduino Uno r3 with MotorShield, controlling a
 linear actuator hooked up to Channel A.
 
 A host controls the steering by issuing one or more commands.

 Available commands:
   l - turn left
   r - turn right
   s - full stop
 
 Commands are sent over a 9600 baud serial connection.  Each
 command will last 20ms (hard delay in loop()).  The host must
 not send commands faster than 20ms, or this controller will
 get confused and behave erratically.

 Recognized commands are written back across the COM port.
 Strings of the same command are processed in batch, to provide
 continuous steering.  Each command in a batch is reported back
 to the host as the command is run, and each batch is terminated
 with a space and the word "done".
 
 Unrecognized commands are reported as ?.  Unrecongized commands
 are still batched in the same manner (i.e. same commands are
 processed in a batch) which may result in a string of multiple ?
 followed by " done".
 
 This controller is configured to accept steering position
 over the analog in port, and limit steering to within
 acceptable positions.  To adjust these, change maxLeft
 and maxRight in the code.
 
*************************************************************/

#include "motorcontrol.h"
#include "command.h"

/** Use channel A */
Channel *ch = &ChA;

/** Max left and right sensor readings */
//NOTE: values in raw readings from sensor.
// assumes left = smaller numbers, right = larger numbers
int maxLeft = 200;
int maxRight = 800;


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
  if (state.sensorVal < maxLeft) {
    fullStop(state);
    return;
  }
  //release the break, if it hasnt been released
  if (!state.brakeReleased) {
    digitalWrite(ch->brakePIn, LOW);
    state.brakeReleased = true;
  }
  //establish forward direction of channel and spin motor at full speed
  //TODO: support partial speeds
  digitalWrite(ch->dirPin, HIGH);
  analogWrite(ch->motorPin, 255);
}

void turnRight(CommandState& state) {
  if (state.sensorVal > maxRight) {
    fullStop(state);
    return;
  }
  //release the break, if it hasnt been released
  if (!state.brakeReleased) {
    digitalWrite(ch->brakePIn, LOW);
    state.brakeReleased = true;
  }

  //establish forward direction of channel and spin motor at full speed
  //TODO: support partial speeds
  digitalWrite(ch->dirPin, LOW);
  analogWrite(ch->motorPin, 255);
}

void fullStop(CommandState& state) {
  analogWrite(ch->motorPin, 0);
  digitalWrite(ch->brakePIn, HIGH);
  state.brakeReleased = false;
}

void setup() {
  
  Serial.begin(9600);
  
  //Setup pins for channel
//  pinMode(ch->sensorPinA, INPUT);
  pinMode(ch->dirPin, OUTPUT);
  pinMode(ch->motorPin, OUTPUT);
  pinMode(ch->brakePIn, OUTPUT);
  
  //write the number of available commands
  Serial.write(0x30 + commandCount);
  Serial.write(" commands\n");
}

void loop() {
  
  int inByte = Serial.peek();
  if (inByte == -1) {
     return;
  }

  CommandState state = {0};
  do {
    int inByte = Serial.read();
    //set the state
    state.sensorVal  = analogRead(ch->sensorAPin);
    state.currentVal = analogRead(ch->curSensingAPin);
    //uncomment to monitor the sensor value
    //    char buf[32] = {0};
    //    sprintf(buf, "%d\n", state.sensorVal);
    //    Serial.write(buf);
    state.cmdByte = inByte;
    Command *cmd = findCommand(inByte);
    if (cmd) {
      Serial.write(inByte);  
      cmd->doCommand(state);
    } else {
      //? means we didn't recognize the command
      Serial.write("?");
    }
    //NOTE: this delay must match the delay on the sender side, or else
    // bad things.
    delay(20);
  } while (Serial.peek() == inByte);
  //indicate that the command is done
  Serial.write(" done\n");
  //always leave loop with the brake engaged
  if (state.brakeReleased) {
    digitalWrite(ch->brakePIn, HIGH);
    analogWrite(ch->motorPin, 0);
  }
}

