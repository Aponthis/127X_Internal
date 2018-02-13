#include "main.h"
#include "sensors.h"
#include "motors.h"
#include "constants.h"
#include "deadband.h"
#include "delayuntil.h"
#include "mogolift.h"
#include "fourbar.h"
#include "dr4b.h"

extern uint8_t stackedCones;
extern bool shouldDrop;

void driveTask(void * parameter){ //Sets drive motors' power
while(1){
		motorSet(LEFTDRIVE, -deadband(joystickGetAnalog(1, 3)));
		motorSet(RIGHTDRIVE, -deadband(joystickGetAnalog(1, 2)));
		delay(20); //Save processing power
	}
}

bool manualMode = 1;
extern PID dr4b;
extern uint8_t stackedCones;
extern bool isOpControl;

void liftTask(void * parameter){  //Task for lift
	while(1){
		if(manualMode){
			manualLift();
		} else {
			dr4bLift();
		}
		fourBar();  //Calls fourBar() function
		delay(20); //Save processing power
	}
}

void operatorControl() {

	if(liftAutoTask != NULL){
		taskSuspend(liftAutoTask);
	}
	if(clockAutoTask != NULL){
		taskSuspend(liftAutoTask);
	}
	if(driveAutoTask != NULL){
		taskSuspend(driveAutoTask);
	}
	if(fourBarAutoTask != NULL){
		taskSuspend(fourBarAutoTask);
	}
	if(rollerAutoTask != NULL){
		taskSuspend(rollerAutoTask);
	}
	if(mogoAutonomousTask != NULL){
		taskSuspend(mogoAutonomousTask);
	}

	taskCreate(driveTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	taskCreate(liftTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	isOpControl = 1;
	while (1) {
		if(joystickGetDigital(1, 8, JOY_LEFT)){// Toggles manual mode
			manualMode = !manualMode;
			delay(500);
		}

		if(joystickGetDigital(1, 8, JOY_DOWN)){  //Mobile goal lift controls
		 mogoPosition = 1;  //puts out mogo intake

		} else if (joystickGetDigital(1, 8, JOY_UP)){
			mogoPosition = 0;
		}

		if(joystickGetDigital(1, 7, JOY_LEFT)){
			stackedCones -=1;
			delay(300);
		}

		if(joystickGetDigital(1, 7, JOY_RIGHT)){
			stackedCones += 1; //Not normally used, should automatically count; only if picking up pre-stacked mogos
			delay(300);
		}

		if(joystickGetDigital(1, 8, JOY_RIGHT)){
			stackedCones = 0;
			delay(300);
		}

		if (joystickGetDigital(1, 7, JOY_DOWN)){
			shouldDrop = 1;
		} else if(isOpControl) {
			shouldDrop = 0;
		}

		mogoLift();
		roller();
		delay(20);
	}
}