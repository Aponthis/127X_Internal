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

  pinMode(BUMPER, INPUT);

  liftEncoder = encoderInit(LIFTTOPPORT, LIFTBOTTOMPORT, true);
  mogoEncoder = encoderInit(MOGOTOPPORT, MOGOBOTTOMPORT, true);
  driveEncoder = encoderInit(DRIVETOP, DRIVEBOTTOM, false);
  encoderReset(driveEncoder);
  gyro = gyroInit(GYRO, 392);  //720 ticks is a rotation

  lcdInit(LCDSCREEN);  //LCD initialization
  lcdSetBacklight(LCDSCREEN, true);

  autonSelect();  //Defined in own file

}
