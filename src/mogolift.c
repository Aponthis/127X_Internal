#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"

bool mogoPosition = 0;
int mogoMeasure = 0;

void mogoLift(){

  mogoMeasure = encoderGet(mogoEncoder);

  if(mogoPosition == 0){  //Attempts to acquire mogo holding position
    if(abs(mogoMeasure - MOGOTOP) == 0){
      motorSet(MOGOLIFT, 0);
    } else if(mogoMeasure < (MOGOTOP - 2)){
      motorSet(MOGOLIFT, 127);
    } else if (mogoMeasure < (MOGOTOP - 1)){
      motorSet(MOGOLIFT, 20);
    } else {
      motorSet(MOGOLIFT, 15);
    }
  }
  if(mogoPosition == 1){  //Attempts to acquire bottom position
    if(mogoMeasure > (MOGOBOTTOM + 5)){
      motorSet(MOGOLIFT, -100);
    } else if (mogoMeasure > (MOGOBOTTOM + 100)){
      motorSet(MOGOLIFT, -40);
    } else if (mogoMeasure > MOGOBOTTOM){
      motorSet(MOGOLIFT, -10);
    } else {
      motorSet(MOGOLIFT, 0);
    }
  }
}
