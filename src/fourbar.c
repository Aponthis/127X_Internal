#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "fourbar.h"
#include "constants.h"
#include "dr4b.h"
#include "deadband.h"

PID four_bar = {
  .Kp = 0.082, .Ki = 0.001, .Kd = 0.0015, .error = 0, .previous_error = 0, .integral = 0,  //.09 .002 .006
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
extern bool macroMode;
extern int liftPower;
int dr4bInitial;
extern bool usingLoader; //if using autoloader or not
int shifterPower;

void fourBar(){
  //lcdPrint(LCDSCREEN, 1, "pow: %d", shifterPower);
  if(isOpControl && !macroMode){
    shifterPower = deadband(joystickGetAnalog(2, 2));

    // if(encoderGet(liftEncoder) > 23 && (analogRead(SHIFTPOT) < 3000) && (shifterPower < 0)){
    //   shifterPower = -5;
    // }

    if(analogRead(SHIFTPOT) < 2000){
      shifterPower -= 12;
    } else if(analogRead(SHIFTPOT) > 2800){
      shifterPower += 12;
    }
    motorSet(LEFTSHIFTER, -shifterPower);
    motorSet(RIGHTSHIFTER, shifterPower);
  } else if (macroMode){ //macro routine
    dr4bInitial = encoderGet(liftEncoder);
    motorSet(ROLLER, -127);
    if(dr4bInitial < 27){  //For low values
      if(usingLoader){
        while(encoderGet(liftEncoder) < 18){
          liftPower = 105;
          delay(20);
          if(joystickGetDigital(2, 8, JOY_UP)){
            break;
          }
        }
        while(encoderGet(liftEncoder) < (dr4bInitial + 1)){
          liftPower = 105;
          delay(20);
          if(joystickGetDigital(2, 8, JOY_UP)){
            break;
          }
        }
        // while(encoderGet(liftEncoder) < (dr4bInitial + 2)){
        //   liftPower = 30;
        //   delay(20);
        //   if(joystickGetDigital(2, 8, JOY_UP)){
        //     break;
        //   }
        // }
      } else {
        while(encoderGet(liftEncoder) < (dr4bInitial + 1)){
          liftPower = 105;
          delay(20);
          if(joystickGetDigital(2, 8, JOY_UP)){
            break;
          }
        }
        // while(encoderGet(liftEncoder) < (dr4bInitial + 2)){
        //   liftPower = 40;
        //   delay(20);
        //   if(joystickGetDigital(2, 8, JOY_UP)){
        //     break;
        //   }
        // }
      }
    } else if (dr4bInitial < 50){
      while(encoderGet(liftEncoder) < (dr4bInitial + 1)){
        liftPower = 105;
        delay(20);
        if(joystickGetDigital(2, 8, JOY_UP)){
          break;
        }
      }
      // while(encoderGet(liftEncoder) < (dr4bInitial + 1)){
      //   liftPower = 30;
      //   delay(20);
      //   if(joystickGetDigital(2, 8, JOY_UP)){
      //     break;
      //   }
      // }
    } else {
      while(encoderGet(liftEncoder) < (dr4bInitial + 1)){
        liftPower = 105;
        delay(20);
        if(joystickGetDigital(2, 8, JOY_UP)){
          break;
        }
      }
      // while(encoderGet(liftEncoder) < (dr4bInitial + 2)){
      //   liftPower = 30;
      //   delay(20);
      //   if(joystickGetDigital(2, 8, JOY_UP)){
      //     break;
      //   }
      // }
    }
    liftPower = -10;
    motorSet(LEFTSHIFTER, -127);  //sends four-bar down
    motorSet(RIGHTSHIFTER, 127);
    delay(300);
    macroMode = 0;
    motorSet(ROLLER, 20);
  } else {
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

    motorSet(LEFTSHIFTER, -four_bar.output_power);
    motorSet(RIGHTSHIFTER, four_bar.output_power);

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
}

bool shouldDrop = 0;

bool button6Pressed = 0;
bool button7Pressed = 0;

void roller(){
  if(isOpControl && !macroMode){
    if(joystickGetDigital(2, 6, JOY_UP)){
      button6Pressed = 1;
    }

    if(button6Pressed && joystickGetDigital(2, 6, JOY_UP)){
      motorSet(ROLLER, 127);
    } else if (button6Pressed) {
      delay(0);
      motorSet(ROLLER, 20);
      button6Pressed = 0;
    }

    if(joystickGetDigital(2, 6, JOY_DOWN)){
      button7Pressed = 1;
    }

    if(button7Pressed && joystickGetDigital(2, 6, JOY_DOWN)){
      motorSet(ROLLER, -127);
    } else if (button7Pressed) {
      delay(400);
      motorSet(ROLLER, 20);
      button7Pressed = 0;
    }

  } else if (macroMode){
    //does not assign a power to the roller
  } else {
    if(shouldDrop){
      motorSet(ROLLER, -127);
      delay(900);
      shouldDrop = 0;
    } else if (analogRead(SHIFTPOT) > (PICKUPANGLE - 200)){
      motorSet(ROLLER, 127); //if down to pick up, intake!
    } else {
      motorSet(ROLLER, 25);
    }

  }
}
