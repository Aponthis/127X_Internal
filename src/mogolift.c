#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"

bool mogoPosition = 0;
int mogoMeasure = 0;
bool panicMode = 0; //Puts mobile goal control into manual, in case of stuck cone or other circumstance
bool button5Pressed = 0;
extern bool isOpControl;
extern uint8_t stackedCones;
bool upOrDown = 0; //0 is up, 1 is down

void mogoLift(){
  if(isOpControl){
    if(joystickGetDigital(1, 5, JOY_UP)){
      motorSet(MOGOLIFT, 127);
      upOrDown = 0;
    } else if (joystickGetDigital(1, 5, JOY_DOWN)){
      motorSet(MOGOLIFT, -127);
      upOrDown = 1;
    } else if (joystickGetDigital(1, 7, JOY_DOWN)){
      motorSet(MOGOLIFT, -50);
    } else if(upOrDown) {
      motorSet(MOGOLIFT, -20);
    } else {
      motorSet(MOGOLIFT, 5);
    }
  } else {

    mogoMeasure = encoderGet(mogoEncoder);

    if((joystickGetDigital(1, 8, JOY_UP) && joystickGetDigital(1, 8, JOY_DOWN)) && !button5Pressed){
      panicMode = 1;
      button5Pressed = 1;
    } else if ((joystickGetDigital(1, 8, JOY_UP)) + joystickGetDigital(1, 8, JOY_DOWN) == 0){
      button5Pressed = 0;
    }

    if(mogoPosition == 0){  //Attempts to acquire mogo holding position
      if(abs(mogoMeasure - MOGOTOP) == 0){
        motorSet(MOGOLIFT, 0);
      } else if(mogoMeasure < (MOGOTOP - 2)){
        motorSet(MOGOLIFT, 127);
      } else if (mogoMeasure < (MOGOTOP - 1)){
        motorSet(MOGOLIFT, 20);
      } else {
        motorSet(MOGOLIFT, 18);
      }
    }
    if(mogoPosition == 1){  //Attempts to acquire bottom position
      if(mogoMeasure > (MOGOBOTTOM + 5)){
        if(stackedCones < 4){
          motorSet(MOGOLIFT, -100);
        } else {
          if (mogoMeasure > -55){
            motorSet(MOGOLIFT, -110);
          } else {
            motorSet(MOGOLIFT, -10);
          }
        }
      } else if (mogoMeasure > (MOGOBOTTOM + 100)){
        motorSet(MOGOLIFT, -40);
      } else if (mogoMeasure > MOGOBOTTOM){
        motorSet(MOGOLIFT, -15);
      } else {
        motorSet(MOGOLIFT, -10);
      }
    }
  }
}
