#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"
#include "delayuntil.h"
#include "mogolift.h"
#include "dr4b.h"
#include "fourbar.h"

extern bool shouldDrop;
extern bool shouldStack;
extern PID dr4b;
extern PID four_bar;
uint8_t autonMode = 0;
uint8_t autonVariation = 0;


bool canCancel = 0;
int cancelAfter;

void lift(int targetValue, int delayAfterward, int cancelTime){  //Input for lift
  dr4b.target = targetValue;
  canCancel = 1;
  cancelAfter = cancelTime;
  while(abs(dr4b.target - dr4b.actual) > 3){
    delay(20);
    if(abs(dr4b.target == LIFTBOTTOM) && abs(dr4b.target - dr4b.actual)<200){  //Gives error if dropping cone
      break;
    }
  }
  canCancel = 0;
  delay(delayAfterward);
}

void liftAuto(void * parameter){ //Lift height task
  while(isAutonomous()){
		dr4bLift();
	  delay(20);
	}
}

PID drive_straight = { //Sets values for lift PID
  .Kp = DRIVEKP, .Ki = DRIVEKI, .Kd = DRIVEKD, .error = 0, .previous_error = 0, .integral = 0,
  .derivative = 0, .target = 0, .actual = 0, .output_power = 0
};

bool driveOrTurn = 0; //0 if driving forward, 1 if turning

void drive(int targetValue, int delayAfterward, int cancelTime){
	encoderReset(driveEncoder);

  driveOrTurn = 0;

  drive_straight.target = targetValue;
  canCancel = 1;
  cancelAfter = cancelTime;
  //delayUntil(abs(left_wheels.error) < 25);
  //delayUntil(abs(drive_straight.error) < 25);
  //while(abs(drive_straight.error) > 25){
  //  delay(20);
  //}
  canCancel = 0;
  delay(delayAfterward);

}

PID drive_turn = {
  .Kp = 1.150, .Ki = 0.080, .Kd = 3.00, .error = 0, .previous_error = 0, .integral = 0,
  .derivative = 0, .target = 0, .actual = 0, .output_power = 0
};

void turn(int targetDegrees, int delayAfterward, int cancelTime){
		driveOrTurn = 1;
		drive_turn.target = targetDegrees;
	  canCancel = 1;
	  cancelAfter = cancelTime;
	  while(abs(drive_turn.target - drive_turn.actual) > 1){
      delay(20);
    }
	  canCancel = 0;
	  delay(delayAfterward);
}

int ticks = 0;

void fourBarAuto(void * parameter){
    while(isAutonomous()){
    fourBar();
    delay(20);
  }
}

void rollerAuto(void * parameter){
  while(isAutonomous()){
    roller();
    lcdPrint(LCDSCREEN, 2, "%d", motorGet(ROLLER));
    delay(20);
  }
}

void driveAuto(void * parameter){ //Drive state task
  while(isAutonomous()){
		if(driveOrTurn == 0){
	    drive_straight.actual = encoderGet(driveEncoder);
	    drive_straight.previous_error = drive_straight.error;
	    drive_straight.error = drive_straight.target - drive_straight.actual;  //Proportional term
	    drive_straight.derivative = drive_straight.previous_error - drive_straight.error;  //Derivative term
	    if(abs(drive_straight.output_power < 127)){
	      drive_straight.integral += drive_straight.error;  //Integral term
	    }
	    if(abs(drive_straight.error) < 20){
	      drive_straight.integral = 0;
	    }
			// if(abs(drive_straight.error) > 20){
			// 	if (drive_straight.integral > 6000){
			// 		drive_straight.integral = 6000;
			// 	} else if (drive_straight.integral < -6000){
			// 		drive_straight.integral = -6000;
			// 	}
			// }

      drive_straight.output_power = drive_straight.Kp * drive_straight.error + drive_straight.Ki\
       * drive_straight.integral - drive_straight.Kd * drive_straight.derivative;



			motorSet(LEFTDRIVE, -drive_straight.output_power);
 		  motorSet(RIGHTDRIVE, -drive_straight.output_power);

 		  //lcdPrint(LCDSCREEN, 1, "Left error: %f", left_wheels.error);  //Prints error to LCD
		} else {
			drive_turn.actual = gyroGet(gyro);
	    drive_turn.previous_error = drive_turn.error;
	    drive_turn.error = drive_turn.target - drive_turn.actual;  //Proportional term
	    drive_turn.derivative = drive_turn.previous_error - drive_turn.error;  //Derivative term
	    if(abs(drive_turn.output_power < 127)){
	      drive_turn.integral += drive_turn.error;  //Integral term
	    }
	    if(abs(drive_turn.error) < 2){
	      drive_turn.integral = 0;
	    }

			if(abs(drive_turn.error) > 20){
				if (drive_turn.integral > 500){
					drive_turn.integral = 500;
				} else if (drive_turn.integral < -500){
					drive_turn.integral = -500;
				}
			}

			drive_turn.output_power = drive_turn.Kp * drive_turn.error + drive_turn.integral * \
			 drive_turn.Ki - drive_turn.derivative * drive_turn.Kd;

			motorSet(LEFTDRIVE, drive_turn.output_power);
 		  motorSet(RIGHTDRIVE, -drive_turn.output_power);

 		  //lcdPrint(LCDSCREEN, 1, "Gyro error: %f", drive_turn.error);
		}
		delay(20);
  }
}

bool autonRunning = 1;

void clockAuto(void * parameter){  //Time to complete a task after which point the routine will give up to avoid burning out
  while(isAutonomous()){
			if(canCancel == 1){
	    while(cancelAfter > 0){
	      delay(20);
	      cancelAfter -= 20;
        lcdPrint(LCDSCREEN, 2, "%d", cancelAfter);
	    }
	    if(cancelAfter <= 0){
	      autonRunning = 0;
	    }
  	}
  delay(20); //Don't hog processing power
	}
}

void mogoAutonomous(void * parameter){
  while(1){
    mogoLift();
    delay(20);
  }
}

void mogo(int8_t whatPosition, bool isSlow){ //0 stowed, 1 up, 2 down if going down slowly, then isSlow = true
  mogoPosition = whatPosition;
}

void autonomous() {
  TaskHandle liftAutoTask = taskCreate(liftAuto,  //Creates task for DR4B PID
                                TASK_DEFAULT_STACK_SIZE,
                                NULL,
                                TASK_PRIORITY_DEFAULT);
  TaskHandle driveAutoTask = taskCreate(driveAuto,  //Creates task for drive distance
                                TASK_DEFAULT_STACK_SIZE,
                                NULL,
                                TASK_PRIORITY_DEFAULT);
  TaskHandle fourBarAutoTask = taskCreate(fourBarAuto,  //Creates task for drive distance
                                TASK_DEFAULT_STACK_SIZE,
                                NULL,
                                TASK_PRIORITY_DEFAULT);
  TaskHandle rollerAutoTask = taskCreate(rollerAuto,  //Creates task for drive distance
                                TASK_DEFAULT_STACK_SIZE,
                                NULL,
                                TASK_PRIORITY_DEFAULT);
  // TaskHandle clockAutoTask = taskCreate(clockAuto,  //Creates task for wathcdog clock
  //                               TASK_DEFAULT_STACK_SIZE,
  //                               NULL,
  //                               TASK_PRIORITY_DEFAULT);
  TaskHandle mogoAutonomousTask = taskCreate(mogoAutonomous,   //Creates task for managing mogo lift height
                                TASK_DEFAULT_STACK_SIZE,
                                NULL,
                                TASK_PRIORITY_DEFAULT);

  gyroReset(gyro);  //Resets gyro accumulated count


  switch(autonMode){
    case 0 :  //If autonMode is 0, which is default
    delay(15000);
    break;
    case 1 :  //Stationary goal auton

    lift(LIFTTOP, 100, 2500);  //Bring lift up to flip out intake

    //turn(45, 500, 2000);  //If offset to left

      lift(STATGO1, 200, 1500);

    if(autonVariation == 2){  //If waiting variant
      lift(STATGO2, 100, 1000);
      delay(3000);  //Wait
    }

    //lift(LIFTBOTTOM, 800, 2500); //Pick up cone

    //turn(0, 500, 2500);

    drive(1060, 2000, 5000);  //Drive toward statgo


    lift(STATGODROP, 100, 1500);


    if(autonVariation == 1 || autonVariation == 2){
      drive(-1000, 400, 3000);
      delay(15000);
    } else if (autonVariation == 3){
      drive(-800, 1000, 3000);

      turn(-100, 4000, 7000);

      drive(4000, 6000, 10000);
    } else {
      drive(-500, 400, 2000);
    }

    if(autonVariation == 3){  //Turning left

    } else if (autonVariation == 4){  //Turning right

    }

    break;
    delay(15000);
    break;
    case 2 :  //Mogo with cone

    shouldStack = 1;
    lift(MOGOHEIGHT + 10, 000, 2500);  //Bring lift up to get out of way for mobile goal lift
    four_bar.target = STATGOANGLE;

    delay(000);

    mogoPosition = 1; //puts out mobile goal lift

    delay(600);

    four_bar.target = STACKANGLE;

    drive(2005, 1800, 3000);  //Drives toward mobile goal

    mogoPosition = 0;  //Picks up mobile goal

    delay(1500);

    lift((MOGOHEIGHT - 10), 100, 1000);
    shouldDrop = 1;
    delay(200);
    lift(MOGOHEIGHT, 100, 1000);

    //lift(LIFTBOTTOM, 200, 2500);

    //drive(200, 600, 1500);

    //shouldStack = 0;

    //shouldStack = 1; //Raise four-bar
    //delay(1000);
    //shouldDrop = 1;  //Drops cone
    //delay(300);

    drive(-1200, 2000, 3500);

    if(autonVariation == 1){  //5pt zone L
      turn(-195, 000, 4000);  //turns around

      lift(MOGOHEIGHT, 200, 2000);

      drive(200, 500, 1000);

      mogoPosition = 1; //extends mobile goal lift
      delay(1400);

      drive(-800, 1000, 6000);  //drives away
    } else if (autonVariation == 2){  //5pt zone R

      turn(195, 000, 4000);  //turns around

      lift(MOGOHEIGHT, 200, 2000);

      drive(200, 500, 1000);

      mogoPosition = 1; //extends mobile goal lift
      delay(1400);

      drive(-800, 1000, 6000);  //drives away
    } else if (autonVariation == 3){  //10pt zone L
      turn(-193, 100, 4000);  //turns around

      drive(1050, 500, 4000);

      lift(MOGOHEIGHT, 200, 2000);

      delay(700);

      mogoPosition = 1; //extends mobile goal lift
      delay(1400);

      drive(-800, 1000, 6000);  //drives away
    } else if (autonVariation == 4){  //10pt zone R

      turn(182, 100, 4000);  //turns around

      drive(1050, 500, 4000);

      lift(MOGOHEIGHT, 200, 2000);

      delay(700);

      mogoPosition = 1; //extends mobile goal lift
      delay(1400);

      drive(-800, 1000, 6000);  //drives away
    }

    if(autonVariation == 5){  //20 pt zone
      drive(-2650, 1500, 6000);  //Drives back

      turn(-140, 1200, 1500); //Turns parallel to bar, facing midpoint

      drive(1400, 1200, 3000);  //Drives parallel to bar

      turn(-225, 800, 1500);

      drive(4000, 1400, 3000);  //Drives over bar

      mogoPosition = 2; //extends out mobile goal
      delay(1000);

      drive(-600, 400, 1500);
      mogoPosition = 0;// pulls mobile goal lift away

    }
    delay(15000);
    break;
    case 3 :
    drive(4000, 6000, 10000);  //Drive forward

    delay(15000);
    break;
    case 4 :
    delay(15000);
    break;
    case 5 :
    delay(15000);
    break;
  }

  taskSuspend(liftAutoTask);
  taskSuspend(driveAutoTask);
  taskSuspend(clockAutoTask);
  taskSuspend(mogoAutonomousTask);
  taskSuspend(fourBarAutoTask);
  taskSuspend(rollerAutoTask);
  motorStopAll();
  delay(20);
}
