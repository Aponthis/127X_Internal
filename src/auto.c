#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"
#include "delayuntil.h"
#include "mogolift.h"
#include "dr4b.h"
#include "fourbar.h"
#include "deadband.h"

extern bool shouldDrop;
extern bool shouldStack;
extern PID dr4b;
extern PID four_bar;
uint8_t autonMode = 0;
uint8_t autonVariation = 0;


bool canCancel = 0;
int cancelAfter;

void lift(int targetValue, int delayAfterward, int cancelTime, bool waitForIt){  //Input for lift
  dr4b.target = targetValue;
  canCancel = 1;
  cancelAfter = cancelTime;
  while(abs(dr4b.target - dr4b.actual) > 3){
    delay(20);
    if((abs(dr4b.target == LIFTBOTTOM) && abs(dr4b.target - dr4b.actual)<200) && waitForIt){  //Gives error if dropping cone
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

PID drive_position = { //Sets values for drive position PID
  .Kp = 0.180, .Ki = 0.005, .Kd = 0.003, .error = 0, .previous_error = 0, .integral = 0,
  .derivative = 0, .target = 0, .actual = 0, .output_power = 0
};

PID drive_velocity = { //Sets values for drive velocity PID
  .Kp = 0.30, .Ki = 0.03, .Kd = 0, .error = 0, .previous_error = 0, .integral = 0,
  .derivative = 0, .target = 60, .actual = 0, .output_power = 0
};

PID drive_turn = {
  .Kp = 2.450, .Ki = 0.008, .Kd = 0.00, .error = 0, .previous_error = 0, .integral = 0,
  .derivative = 0, .target = 0, .actual = 0, .output_power = 0
};

bool driveOrTurn = 0; //0 if driving forward, 1 if turning

void drive(int targetValue, int delayAfterward, int cancelTime){
	encoderReset(driveEncoder);

  driveOrTurn = 0;

  drive_position.target = targetValue;
  canCancel = 1;
  cancelAfter = cancelTime;
  delay(200);
    while(abs(drive_position.target - encoderGet(driveEncoder)) > 80){
     delay(20);
   }
   delay(150);

  canCancel = 0;
  delay(delayAfterward);

}
int fullPower = 0;

void driveImprecise(int targetValue, int delayAfterward, int cancelTime){
	encoderReset(driveEncoder);

  driveOrTurn = 0;

  drive_position.target = targetValue;
  canCancel = 1;
  cancelAfter = cancelTime;
  delay(200);
    while(abs(drive_position.target - encoderGet(driveEncoder)) > 550){
     delay(20);
   }
   delay(450);

  canCancel = 0;
  delay(delayAfterward);

}

void turn(int targetDegrees, int delayAfterward, int cancelTime){
		driveOrTurn = 1;
		drive_turn.target = targetDegrees;
	  canCancel = 1;
	  cancelAfter = cancelTime;
	  while(abs(drive_turn.target - drive_turn.actual) > 3){
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
    delay(20);
  }
}

int leftDrivePower = 0;
int rightDrivePower = 0;

int lastPosition;  //for drive velocity PID
int currentPosition;

void driveAuto(void * parameter){ //Drive state task
  while(isAutonomous()){

    lcdPrint(LCDSCREEN, 1, "Drive act: %d", drive_position.actual);
    lcdPrint(LCDSCREEN, 2, "Drive target: %d", drive_position.target);
    drive_turn.actual = gyroGet(gyro);
    drive_turn.error = drive_turn.target - drive_turn.actual;  //Proportional term
    drive_turn.previous_error = drive_turn.error;
    drive_turn.derivative = drive_turn.previous_error - drive_turn.error;  //Derivative term
    if(abs(drive_turn.output_power < 127)){
      drive_turn.integral += drive_turn.error;  //Integral term
    }
    if(abs(drive_turn.error) < 2){
      drive_turn.integral = 0;
    }

    // if(abs(drive_turn.error) > 20){
    //   if (drive_turn.integral > 500){
    //     drive_turn.integral = 500;
    //   } else if (drive_turn.integral < -500){
    //     drive_turn.integral = -500;
    //   }
    // }
    drive_turn.output_power = drive_turn.Kp * drive_turn.error + drive_turn.integral * \
     drive_turn.Ki - drive_turn.derivative * drive_turn.Kd;

		if(driveOrTurn == 0){
      drive_position.actual = encoderGet(driveEncoder);
      //lcdPrint(LCDSCREEN, 2, "LDrive: %d", leftDrivePower);
      //lcdPrint(LCDSCREEN, 1, "Driving");


      if(drive_position.target - drive_position.actual < 300){
  	    drive_position.previous_error = drive_position.error;
  	    drive_position.error = drive_position.target - drive_position.actual;  //Proportional term
  	    drive_position.derivative = drive_position.previous_error - drive_position.error;  //Derivative term
  	    if(abs(drive_position.output_power < 127)){
  	      drive_position.integral += drive_position.error;  //Integral term
  	    }
  	    if(abs(drive_position.error) < 40){
  	      drive_position.integral = 0;
  	    }

        drive_position.output_power = drive_position.Kp * drive_position.error + drive_position.Ki\
         * drive_position.integral - drive_position.Kd * drive_position.derivative;

        if(drive_position.output_power > 127){
         drive_position.output_power = 127;
        } else if(drive_position.output_power < -127){
         drive_position.output_power = -127;
        }

        leftDrivePower = drive_position.output_power - drive_turn.output_power;
        rightDrivePower = drive_position.output_power + drive_turn.output_power;

        if(leftDrivePower > 127){     //Makes sure both sides aren't just maxxed out without a difference in velocity between sides for angle correction
          rightDrivePower -= (leftDrivePower - 127);
          leftDrivePower -= (leftDrivePower - 127);
        }
        if(rightDrivePower > 127){
          leftDrivePower -= (rightDrivePower - 127);
          rightDrivePower -= (rightDrivePower - 127);
        }
        if(leftDrivePower < -127){
          rightDrivePower -= (leftDrivePower + 127);
          leftDrivePower -= (leftDrivePower + 127);
        }
        if(rightDrivePower < -127){
          leftDrivePower -= (rightDrivePower + 127);
          rightDrivePower -= (rightDrivePower + 127);
        }

        if(fullPower == 1){
          leftDrivePower = 127;
          rightDrivePower = 127;
        } else if (fullPower == -1){
          leftDrivePower = -127;
          rightDrivePower = -127;
        }

  			motorSet(LEFTDRIVE, -deadband(leftDrivePower));
   		  motorSet(RIGHTDRIVE, -deadband(rightDrivePower));

   		  //lcdPrint(LCDSCREEN, 1, "Drive: %d", drive_position.actual);  //Prints error to LCD
      } else {
        lastPosition = currentPosition;
        currentPosition = encoderGet(driveEncoder);

        drive_velocity.actual = currentPosition - lastPosition;

        drive_velocity.error = drive_velocity.target - drive_velocity.actual;
        drive_velocity.previous_error = drive_velocity.error;
        drive_velocity.error = drive_velocity.target - drive_velocity.actual;  //Proportional term
        drive_velocity.derivative = drive_velocity.previous_error - drive_velocity.error;  //Derivative term
        if(abs(drive_velocity.output_power < 127)){
          drive_velocity.integral += drive_velocity.error;  //Integral term
        }
        if(abs(drive_velocity.error) < 10){
          drive_velocity.integral = 0;
        }

        drive_velocity.output_power += drive_velocity.Kp * drive_velocity.error + drive_velocity.Ki\
         * drive_velocity.integral - drive_velocity.Kd * drive_velocity.derivative;

        // if(drive_velocity.output_power > 127){
        //   drive_velocity.output_power = 127;
        // } else if(drive_velocity.output_power < -127){
        //   drive_velocity.output_power = -127;
        // }
        leftDrivePower = drive_velocity.output_power - drive_turn.output_power;
        rightDrivePower = drive_velocity.output_power + drive_turn.output_power;

        if(leftDrivePower > 127){     //Makes sure both sides aren't just maxed out without a difference in velocity between sides for angle correction
          rightDrivePower -= (leftDrivePower - 127);
          leftDrivePower -= (leftDrivePower - 127);
        }
        if(rightDrivePower > 127){
          leftDrivePower -= (rightDrivePower - 127);
          rightDrivePower -= (rightDrivePower - 127);
        }
        if(leftDrivePower < -127){
          rightDrivePower -= (leftDrivePower + 127);
          leftDrivePower -= (leftDrivePower + 127);
        }
        if(rightDrivePower < -127){
          leftDrivePower -= (rightDrivePower + 127);
          rightDrivePower -= (rightDrivePower + 127);
        }

        if(fullPower == 1){
          leftDrivePower = 127;
          rightDrivePower = 127;
        } else if (fullPower == -1){
          leftDrivePower = -127;
          rightDrivePower = -127;
        }

        motorSet(LEFTDRIVE, -deadband(leftDrivePower));
        motorSet(RIGHTDRIVE, -deadband(rightDrivePower));
      }
		} else {
      //If only turning, not moving forward

      //lcdPrint(LCDSCREEN, 1, "Turning");
			motorSet(LEFTDRIVE, (drive_turn.output_power));
 		  motorSet(RIGHTDRIVE, -(drive_turn.output_power));

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
        //lcdPrint(LCDSCREEN, 2, "%d", cancelAfter);
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

    four_bar.target = STATGOANGLE;

    lift(STATGO1, 200, 200, 1);

    if(autonVariation == 2){  //If waiting variant
      lift(STATGO2, 100, 1000, 1);
      delay(3000);  //Wait
    }

    driveImprecise(1250, 1000, 5000);  //Drive toward statgo

    if(autonVariation == 1 || autonVariation == 2){
      shouldDrop = 1;

      delay(200);

      four_bar.target = STACKANGLE;

      drive(-800, 400, 3000);

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
    case 2 :  //Mogo with cone  ------------------------------------------------------------------------------------------------

    shouldStack = 1;
    lift(MOGOHEIGHT, 000, 2500, 1);  //Bring lift up to get out of way for mobile goal lift
    four_bar.target = STATGOANGLE;

    delay(200);

    mogoPosition = 1; //puts out mobile goal lift

    delay(400);

    four_bar.target = STACKANGLE;

    drive(3000, 000, 3000);  //Drives toward mobile goal

    mogoPosition = 0;  //Picks up mobile goal

    delay(800);

    lift((MOGOHEIGHT - 10), 000, 1000, 1);

    shouldDrop = 1;

    lift(MOGOHEIGHT, 000, 1000, 0);

    //lift(LIFTBOTTOM, 200, 2500);

    //drive(200, 600, 1500);

    //shouldStack = 0;

    //shouldStack = 1; //Raise four-bar
    //delay(1000);
    //shouldDrop = 1;  //Drops cone
    //delay(300);



    if(autonVariation == 1){  //5pt zone L
      drive(-2800, 000, 3500);

      turn(-195, 000, 4000);  //turns around

      lift(MOGOHEIGHT, 200, 2000, 0);

      drive(200, 500, 1000);

      mogoPosition = 1; //extends mobile goal lift
      delay(1100);

      drive(-1000, 0000, 6000);  //drives away
    } else if (autonVariation == 2){  //5pt zone R
      drive(-2800, 000, 3500);

      turn(195, 000, 4000);  //turns around

      lift(MOGOHEIGHT, 200, 2000, 0);

      drive(200, 500, 1000);

      mogoPosition = 1; //extends mobile goal lift
      delay(1100);

      drive(-1000, 0000, 6000);  //drives away
    } else if (autonVariation == 3){  //10pt zone L
      drive(-2800, 000, 3500);

      turn(-195, 000, 4000);  //turns around

      lift(MOGOHEIGHT, 200, 2000, 1);

      drive(450, 500, 1000);

      mogoPosition = 1; //extends mobile goal lift
      delay(1100);

      drive(-1000, 2000, 6000);  //drives away
    } else if (autonVariation == 4){  //10pt zone R
      drive(-2800, 000, 3500);

      turn(195, 000, 4000);  //turns around

      lift(MOGOHEIGHT, 200, 2000, 1);

      drive(450, 500, 1000);

      mogoPosition = 1; //extends mobile goal lift
      delay(1100);

      drive(-1000, 2000, 6000);  //drives away
    }

    if(autonVariation == 5){  //20 pt zone L
      drive(-2950, 200, 3500);

      turn(38, 200, 1500); //Turns parallel to bar, facing midpoint

      drive(-1000, 0, 3000);  //Drives parallel to bar

      turn(135, 200, 1500);

      driveImprecise(500, 000, 3000);  //Drives to bar

      fullPower = 1;
      driveImprecise(400, 0, 1000); //Drives over bar

      mogoPosition = 1; //extends out mobile goal
      delay(1000);

      fullPower = -1;
      driveImprecise(-600, 100, 1500);
      mogoPosition = 0;// pulls mobile goal lift away
      fullPower = 0;

    }

    if(autonVariation == 6){  //20 pt zone R
      drive(-2950, 200, 3500);

      turn(-38, 200, 1500); //Turns parallel to bar, facing midpoint

      drive(-1100, 0, 3000);  //Drives parallel to bar

      turn(-135, 200, 1500);

      driveImprecise(500, 000, 3000);  //Drives to bar

      fullPower = 1;
      driveImprecise(400, 0, 1000); //Drives over bar

      mogoPosition = 1; //extends out mobile goal
      delay(1000);

      fullPower = -1;
      driveImprecise(-600, 100, 1500);
      mogoPosition = 0;// pulls mobile goal lift away
      fullPower = 0;

    }
    delay(15000);
    break;
    case 3 :  //Charge auton
    driveImprecise(1500, 6000, 10000);  //Drive forward

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
