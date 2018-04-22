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
  .Kp = 4.9, .Ki = 0.030, .Kd = 0.85, .error = 0, .previous_error = 0, .integral = 0,  //3.5 .11 2.7
  .derivative = 0, .target = LIFTBOTTOM, .actual = LIFTBOTTOM, .output_power = 0
};

bool isOpControl = 0;
extern bool macroMode;
int liftPower;

void manualLift(){
  lcdPrint(LCDSCREEN, 2, "Lift at %d", encoderGet(liftEncoder));
  motorSet(RIGHTLIFT, -liftPower);
  motorSet(LEFTLIFT, liftPower);

  if(!macroMode){
    liftPower = deadband(joystickGetAnalog(2, 3));
  } else {
    //do not give motors specific value
  }
}

void dr4bLift(){
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
  if(abs(dr4b.error) > 8){
    if (dr4b.integral > 200){
      dr4b.integral = 200;
    } else if (dr4b.integral < -200){
      dr4b.integral = -200;
    }
  }

  dr4b.output_power = dr4b.Kp * dr4b.error + dr4b.Ki * dr4b.integral + \
   dr4b.Kd * dr4b.derivative;

  motorSet(LEFTLIFT, dr4b.output_power);
  motorSet(RIGHTLIFT, -dr4b.output_power);
}
