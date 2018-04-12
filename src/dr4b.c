#include "main.h"
#include "dr4b.h"
#include "motors.h"
#include "sensors.h"
#include "deadband.h"

extern PID four_bar;
extern uint8_t stackedCones;
extern bool coneMode;
int8_t autoloaderMode = 0; //0 if not using autoloader and down, 1 if raised slightly, 2 if using autoloader
bool safeAngle = 0;  //If chain bar has achieved safe angle

PID dr4b = { //Sets values for lift PID
  .Kp = 5.50, .Ki = 0.022, .Kd = 0.30, .error = 0, .previous_error = 0, .integral = 0,  //3.5 .11 2.7
  .derivative = 0, .target = LIFTBOTTOM, .actual = LIFTBOTTOM, .output_power = 0
};

float Ka = 8.00;
int8_t assistMode = -1;  //-1 for bottom of lift, 1 for going up
bool isOpControl = 0;

void manualLift(){
  motorSet(RIGHTLIFT, deadband(joystickGetAnalog(2, 3)));
  motorSet(LEFTLIFT, -deadband(joystickGetAnalog(2, 3)));
}

void dr4bLift(){
  lcdPrint(LCDSCREEN, 2, "Cones: %d", stackedCones);
  if(joystickGetDigital(1, 5, JOY_DOWN) && autoloaderMode != 0){
    autoloaderMode = 0;
  } else if (joystickGetDigital(1, 5, JOY_UP)){
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

  lcdPrint(LCDSCREEN, 1, "Gyro: %d", gyroGet(gyro));
  //lcdPrint(LCDSCREEN, 2, "Power: %d", dr4b.output_power);
  if(((coneMode) || mogoProtocol) && isOpControl){  //previously had safe angle
    safeAngle = 1;
    assistMode = 1;
     if(autoloaderMode == 2){  //if stacking from autoloader
        switch(stackedCones){
          case 0:
          dr4b.target = 6;
          break;
          case 1:
          dr4b.target = 10;
          break;
          case 2:
          dr4b.target = 16;
          break;
          case 3:
          dr4b.target = 25;
          break;
          case 4:
          dr4b.target = 29;
          break;
          case 5:
          dr4b.target = 35;
          break;
          case 6:
          dr4b.target = 40;
          break;
          case 7:
          dr4b.target = 44;
          break;
          case 8:
          dr4b.target = 50;
          break;
          case 9:
          dr4b.target = 58;
          break;
          case 10:
          dr4b.target = 65;
          break;
          case 11:
          dr4b.target = 77;
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
    } else {
      switch(stackedCones){  //field cones
        case 0:
        dr4b.target = 3;
        break;
        case 1:
        dr4b.target = 7;
        break;
        case 2:
        dr4b.target = 14;
        break;
        case 3:
        dr4b.target = 18;
        break;
        case 4:
        dr4b.target = 25;
        break;
        case 5:
        dr4b.target = 33;
        break;
        case 6:
        dr4b.target = 39;
        break;
        case 7:
        dr4b.target = 44;
        break;
        case 8:
        dr4b.target = 49;
        break;
        case 9:
        dr4b.target = 57;
        break;
        case 10:
        dr4b.target = 63;
        break;
        case 11:
        dr4b.target = 73;
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
  motorSet(RIGHTLIFT, dr4b.output_power);
}
