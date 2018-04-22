#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"

bool mogoPosition = 0;
bool prevMogoPosition = 0;
bool positionAcquired = 1; //if action to axquire position is done

bool button5Pressed = 0;
extern bool isOpControl;
bool upOrDown = 0; //0 is up, 1 is down
extern bool usingLoader;

void mogoLift(){
  if(isOpControl){
    if(joystickGetDigital(1, 5, JOY_UP)){
      motorSet(MOGOLIFT, 127);
      upOrDown = 0;
    } else if (joystickGetDigital(1, 5, JOY_DOWN)){
      motorSet(MOGOLIFT, -127);
      upOrDown = 1;
      usingLoader = 0;
    } else if (joystickGetDigital(1, 7, JOY_DOWN)){
      motorSet(MOGOLIFT, -50);
    } else if(upOrDown) {
      motorSet(MOGOLIFT, -20);
    } else {
      motorSet(MOGOLIFT, 25);
    }
  } else {
    if(mogoPosition != prevMogoPosition){
      positionAcquired = 0;
      prevMogoPosition = mogoPosition;
    }
    if(!positionAcquired){
      if(mogoPosition == 1){  //mogo lift is meant to be out
        motorSet(MOGOLIFT, -127);
        delay(1450);
        upOrDown = 1;
        positionAcquired = 1;
      } else if (mogoPosition == 0){
        motorSet(MOGOLIFT, 127);
        delay(1300);
        upOrDown = 0;
        positionAcquired = 1;
      }
    } else {
      if(upOrDown) {
        motorSet(MOGOLIFT, -20);
      } else if(joystickGetAnalog(2, 2) < -40) {
        motorSet(MOGOLIFT, 55);
      } else {
        motorSet(MOGOLIFT, 25);
      }
    }

  }
}
