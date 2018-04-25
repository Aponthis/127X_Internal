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

  liftEncoder = encoderInit(LIFTTOPPORT, LIFTBOTTOMPORT, false);
  driveEncoder = encoderInit(DRIVETOP, DRIVEBOTTOM, true);

  leftEncoder = encoderInit(LEFTTOPPORT, LEFTBOTTOMPORT, true);
  rightEncoder = encoderInit(RIGHTTOPPORT, RIGHTBOTTOMPORT, true);

  gyro = gyroInit(GYRO, 196);

  lcdInit(LCDSCREEN);  //LCD initialization
  lcdSetBacklight(LCDSCREEN, true);

  while(analogRead(SIGNALWIRE) < 500){
    lcdPrint(LCDSCREEN, 1, "CHECK EXPANDER");
    if(lcdReadButtons(LCDSCREEN) > 0){
      break;
    }
    delay(20);
  }


  Gyro gyro = gyroInit(GYRO, 196);  /////////////////////////////////////////////Changed this value from 0
  delay(500);
  gyroReset(gyro);

  autonSelect();  //Defined in own file

}
