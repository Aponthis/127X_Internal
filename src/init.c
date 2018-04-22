#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"
#include "autonselect.h"
#include "main.h"

void initializeIO() {
}

void initialize() {
  //shiftEncoder = encoderInit(SHIFTTOP, SHIFTBOTTOM, true);

  analogCalibrate(SHIFTPOT);

  pinMode(LIGHTS, OUTPUT);

  liftEncoder = encoderInit(LIFTTOPPORT, LIFTBOTTOMPORT, false);
  mogoEncoder = encoderInit(MOGOTOPPORT, MOGOBOTTOMPORT, true);
  driveEncoder = encoderInit(DRIVETOP, DRIVEBOTTOM, true);
  encoderReset(driveEncoder);
  gyro = gyroInit(GYRO, 196);  //392 before

  lcdInit(LCDSCREEN);  //LCD initialization
  lcdSetBacklight(LCDSCREEN, true);

  while(analogRead(SIGNALWIRE) < 500){
    lcdPrint(LCDSCREEN, 1, "CHECK EXPANDER");
    if(lcdReadButtons(LCDSCREEN) > 0){
      break;
    }
    delay(20);
  }

  autonSelect();  //Defined in own file

}
