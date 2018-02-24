#include "main.h"
#include "dr4b.h"
#include "motors.h"
#include "sensors.h"

extern PID four_bar;
extern uint8_t stackedCones;
extern bool coneMode;
int8_t autoloaderMode = 0; //0 if not using autoloader and down, 1 if raised slightly, 2 if using autoloader
bool safeAngle = 0;  //If chain bar has achieved safe angle

PID dr4b = { //Sets values for lift PID
  .Kp = 3.50, .Ki = 0.11, .Kd = 2.70, .error = 0, .previous_error = 0, .integral = 0,  //3.8 0.12 15.5
  .derivative = 0, .target = LIFTBOTTOM, .actual = LIFTBOTTOM, .output_power = 0
};

float Ka = 8.00;
int8_t assistMode = -1;  //-1 for bottom of lift, 1 for going up
bool isOpControl = 0;

void manualLift(){
  if(joystickGetDigital(1, 5, JOY_UP)){
    motorSet(LEFTLIFT, 127);
    motorSet(RIGHTLIFT1, 127);
    motorSet(RIGHTLIFT2, 127);
  } else if (joystickGetDigital(1, 5, JOY_DOWN)){
    motorSet(LEFTLIFT, -127);
    motorSet(RIGHTLIFT1, -127);
    motorSet(RIGHTLIFT2, -127);
  }  else if (mogoProtocol && encoderGet(liftEncoder) < MOGOHEIGHT){
    motorSet(LEFTLIFT, 30);
    motorSet(RIGHTLIFT1, 30);
    motorSet(RIGHTLIFT2, 30);
  }  else {
    motorSet(LEFTLIFT, 0);
    motorSet(RIGHTLIFT1, 0);
    motorSet(RIGHTLIFT2, 0);
  }
  lcdPrint(LCDSCREEN, 2, "Cones: %d", stackedCones);
}

void dr4bLift(){
  lcdPrint(LCDSCREEN, 2, "Cones: %d", stackedCones);
  if(joystickGetDigital(1, 5, JOY_DOWN) && autoloaderMode != 0){
    autoloaderMode = 0;
  } else if (joystickGetDigital(1, 5, JOY_UP) && autoloaderMode == 0){
    if(autoloaderMode == 0){
      autoloaderMode = 1;
    }
    delay(100);
    if(joystickGetDigital(1, 5, JOY_UP)){
      delay(100);
        if(joystickGetDigital(1,5, JOY_UP)){
          delay(100);
          if(joystickGetDigital(1, 5, JOY_UP)){
            autoloaderMode = 2;
          }
        }
    }
  }

  lcdPrint(LCDSCREEN, 1, "Lift: %d", dr4b.actual);
  //lcdPrint(LCDSCREEN, 2, "Power: %d", dr4b.output_power);
  if(((coneMode) || mogoProtocol) && isOpControl){  //previously had safe angle
    safeAngle = 1;
    assistMode = 1;
        switch(stackedCones){
          case 0:
          dr4b.target = 3;
          break;
          case 1:
          dr4b.target = 7;
          break;
          case 2:
          dr4b.target = 13;
          break;
          case 3:
          dr4b.target = 18;
          break;
          case 4:
          dr4b.target = 25;
          break;
          case 5:
          dr4b.target = 31;
          break;
          case 6:
          dr4b.target = 37;
          break;
          case 7:
          dr4b.target = 41;
          break;
          case 8:
          dr4b.target = 48;
          break;
          case 9:
          dr4b.target = 56;
          break;
          case 10:
          dr4b.target = 62;
          break;
          case 11:
          dr4b.target = 71;
          break;
          case 12:
          dr4b.target = 85;
          break;
          case 13:
          dr4b.target = 85;
          break;
          case 14:
          dr4b.target = 85;
          break;
          case 15:
          dr4b.target = 85;
      }
      if (mogoProtocol && dr4b.target < 26){
        dr4b.target = 26;
      }
  } else if (isOpControl && !safeAngle) {
    if(four_bar.actual < (STACKANGLE - 600)){  //weeeeee ooooooooooooo could cause an issue, look here later!!!!@#!@#!@#@!#
      assistMode = -1;
      if(autoloaderMode == 2){
        dr4b.target = 24;
      } else if(autoloaderMode == 1){
        dr4b.target = 10;
      } else {
        dr4b.target = LIFTBOTTOM;
      }
    }
  }

  dr4b.actual = encoderGet(liftEncoder);
  dr4b.previous_error = dr4b.error;
  dr4b.error = dr4b.target - dr4b.actual;  //Proportional term
  dr4b.derivative = dr4b.error - dr4b.previous_error;  //Derivative term
  if(abs(dr4b.output_power < 127)){
    dr4b.integral += dr4b.error;  //Integral term
  }
  if(abs(dr4b.error) < 2){
    dr4b.integral = 0;
  }
  if(abs(dr4b.error) > 5){
    if (dr4b.integral > 200){
      dr4b.integral = 200;
    } else if (dr4b.integral < -200){
      dr4b.integral = -200;
    }
  }

  dr4b.output_power = dr4b.Kp * dr4b.error + dr4b.Ki * dr4b.integral + \
   dr4b.Kd * dr4b.derivative + Ka * assistMode;

  motorSet(LEFTLIFT, dr4b.output_power);
  motorSet(RIGHTLIFT1, dr4b.output_power);
  motorSet(RIGHTLIFT2, dr4b.output_power);
}
