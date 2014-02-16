/*************************************************************

*************************************************************/

#include "motorcontrol.h"
#include "command.h"

/** Use channel A */
Channel *ch = &ChA;

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

//NOTE: values in raw readings from sensor.  assumes left = smaller numbers, right = larger numbers
int maxLeft = 200;
int maxRight = 800;

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
    delay(50);
  } while (Serial.peek() == inByte);
  //indicate that the command is done
  Serial.write(" done\n");
  //always leave loop with the brake engaged
  if (state.brakeReleased) {
    digitalWrite(ch->brakePIn, HIGH);
    analogWrite(ch->motorPin, 0);
  }
}

