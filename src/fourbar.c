#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "fourbar.h"
#include "constants.h"
#include "dr4b.h"

PID four_bar = {
  .Kp = 0.062, .Ki = 0.002, .Kd = 0.002, .error = 0, .previous_error = 0, .integral = 0,  //.09 .002 .006
  .derivative = 0, .target = STACKANGLE, .actual = 0, .output_power = 0
};

extern bool manualMode;
extern PID dr4b;
bool mogoProtocol = 0;
bool coneMode = 0; //0 for no cone held, 1 for cone held
bool shouldStack = 0;
uint8_t stackedCones = 0;
bool isStacking; //Stores whether the four-bar has tried to go up, to avoid throwing it down when the DR4B goes below a threshold
extern int8_t autoloaderMode;
extern bool safeAngle;
extern bool isOpControl;

void fourBar(){
  if(joystickGetDigital(1, 7, JOY_UP)){
    mogoProtocol = !mogoProtocol;
    delay(300);
  }

  if (lcdReadButtons(LCDSCREEN) == 1){
    lcdPrint(LCDSCREEN, 1, "4bar: %d", four_bar.actual);
    lcdPrint(LCDSCREEN, 2, "Target: %d", four_bar.target);
  }
//  if(analogRead(SHIFTPOT) > 500){ //filters trash values
    four_bar.actual = analogRead(SHIFTPOT);
//  }
  //four_bar.actual = encoderGet(shiftEncoder);
  four_bar.previous_error = four_bar.error;
  four_bar.error = four_bar.target - four_bar.actual;  //Proportional term
  four_bar.derivative = four_bar.error - four_bar.previous_error;  //Derivative term
  if(abs(four_bar.output_power < 127)){
    four_bar.integral += four_bar.error;  //Integral term
  }
  if(abs(four_bar.error) < 30){
    four_bar.integral = 0;
  }
  if(abs(four_bar.error) > 50){
    if (four_bar.integral > 8000){
      four_bar.integral = 8000;
    } else if (four_bar.integral < -8000){
      four_bar.integral = -8000;
    }
  }

  if(four_bar.target == STACKANGLE){
    four_bar.integral = 0;
  }

  four_bar.output_power = four_bar.Kp * four_bar.error + four_bar.Ki * four_bar.integral +\
   four_bar.Kd * four_bar.derivative;

  motorSet(SHIFTER, four_bar.output_power);

  if(isOpControl){
    if((shouldStack && (((dr4b.actual - dr4b.target) > -3) || isStacking)) || \
    (mogoProtocol && (dr4b.actual - dr4b.target) > - 3) || (mogoProtocol && (manualMode))){
      four_bar.target = STACKANGLE;
      isStacking = 1;
    } else if (((shouldStack + coneMode == 0))  && \
      ((dr4b.target == LIFTBOTTOM || manualMode) && (dr4b.actual < (4 + LIFTBOTTOM)))){
      four_bar.target = PICKUPANGLE;
    } else {
      four_bar.target = HOLDANGLE;
    }
  }

  // if(coneMode == 0){  //If not holding a cone
  //   if(mogoProtocol){
  //     four_bar.target = STACKANGLE;
  //   }else if(shouldStack == 0){
  //     four_bar.target = PICKUPANGLE;  //Bring the fourbar to pickup spot
  //   } else {
  //     four_bar.target = HOLDANGLE;
  //   }
  // } else { //If holding a cone
  //     four_bar.target = STACKANGLE;
  //   } else {
  //     four_bar.target = HOLDANGLE;
  //   }
  // }
}

bool shouldDrop = 0;

void roller(){
  if(!digitalRead(BUMPER) & !coneMode){  //Converts input to 1 for if cone is held, 0 if not
    coneMode = 1;
  }

  if(joystickGetDigital(1, 6, JOY_UP)){  //Going up to stack
    if(coneMode == 1){
      shouldStack = 1;
    }
  } else if (joystickGetDigital(1, 6, JOY_DOWN)){  //Going down
    shouldStack = 0;
    mogoProtocol = 0;
    isStacking = 0;
    autoloaderMode = 0;
    safeAngle = 0;
  }

  if(shouldStack && (four_bar.actual > STACKANGLE - 700) && isOpControl){
    delay(430);
    shouldDrop = 1;
  }
  if(mogoProtocol && !(joystickGetDigital(1, 7, JOY_DOWN))){
    motorSet(ROLLER, 0);
  } else if(shouldDrop){  //Drop cone
    motorSet(ROLLER, -127);
    delay(300);  //Enough time for cone to be totally removed
    motorSet(ROLLER, 0);
    coneMode = 0;
    if(isOpControl){
      isStacking = 0;
      safeAngle = 0;
      if(shouldStack){
        stackedCones +=1;
      }
      shouldStack = 0;
      if(stackedCones == 13){
        mogoProtocol = 1;
      }
    }
  }
   else if(coneMode == 0){
    motorSet(ROLLER, 127);  //Intake
  } else {
    motorSet(ROLLER, 35);
  }
}
