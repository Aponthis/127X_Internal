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
extern bool isOpControl;

void driveTask(void * parameter){ //Sets drive motors' power
	while(isOpControl){
		motorSet(LEFTDRIVE1, -deadband(joystickGetAnalog(1, 3)));
		motorSet(LEFTDRIVE2, deadband(joystickGetAnalog(1, 3)));
		motorSet(RIGHTDRIVE1, deadband(joystickGetAnalog(1, 2)));
		motorSet(RIGHTDRIVE2, -deadband(joystickGetAnalog(1, 2)));
		delay(20); //Save processing power
	}
}

bool button1Pressed = 0;
bool button2Pressed = 0;
bool button3Pressed = 0;
bool button4Pressed = 0;
bool manualMode = 0;
bool macroMode = 0; //is currently operating under releasing macro
bool usingLoader = 0;
extern PID dr4b;
extern uint8_t stackedCones;


void liftTask(void * parameter){  //Task for lift
	while(1){
		manualLift();
		delay(20); //Save processing power
	}
}

void fourBarTask(void * parameter){  //Task for lift
	while(1){
		fourBar();
		delay(20); //Save processing power
	}
}

void operatorControl() {

	if(liftAutoTask != NULL){
		taskSuspend(liftAutoTask);
	}
	if(clockAutoTask != NULL){
		taskSuspend(clockAutoTask);
	}
	if(driveAutoTask != NULL){
		taskSuspend(driveAutoTask);
	}
	if(fourBarAutoTask != NULL){
		taskSuspend(fourBarAutoTask);
	}
	if(mogoAutonomousTask != NULL){
		taskSuspend(mogoAutonomousTask);
	}
	if(rollerAutoTask != NULL){
		taskSuspend(rollerAutoTask);
	}

	isOpControl = 1;

	taskCreate(driveTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	taskCreate(liftTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	taskCreate(fourBarTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);


  //digitalWrite(LIGHTS, HIGH);
	while (1) {
		// if(joystickGetDigital(1, 8, JOY_LEFT) && !button1Pressed){
		// 	manualMode = !manualMode;
		// 	button1Pressed = 1;
		// } else if (!joystickGetDigital(1, 8, JOY_LEFT)){
		// 	button1Pressed = 0;
		// }
		//
		// if(joystickGetDigital(1, 8, JOY_DOWN)){  //Mobile goal lift controls
		//  mogoPosition = 1;  //puts out mogo intake
		//
		// } else if (joystickGetDigital(1, 8, JOY_UP)){
		// 	mogoPosition = 0;
		// }
		//
		// if(joystickGetDigital(1, 7, JOY_LEFT) && !button2Pressed){
		// 	stackedCones -= 1;
		// 	button2Pressed = 1;
		// } else if (!joystickGetDigital(1, 7, JOY_LEFT)){
		// 	button2Pressed = 0;
		// }
		//
		// if(joystickGetDigital(1, 7, JOY_RIGHT) && !button3Pressed){
		// 	stackedCones += 1;
		// 	button3Pressed = 1;
		// } else if (!joystickGetDigital(1, 7, JOY_RIGHT)){
		// 	button3Pressed = 0;
		// }
		//
		// if(joystickGetDigital(1, 8, JOY_RIGHT) && !button4Pressed){
		// 	stackedCones = 0;
		// 	button4Pressed = 1;
		// } else if (!joystickGetDigital(1, 8, JOY_RIGHT)){
		// 	button4Pressed = 0;
		// }
		//
		// if (joystickGetDigital(1, 7, JOY_DOWN)){
		// 	shouldDrop = 1;
		// } else if(isOpControl) {
		// 	shouldDrop = 0;
		// }

		if(joystickGetDigital(2, 8, JOY_LEFT)){
			usingLoader = 1;
		} else if (joystickGetDigital(2, 8, JOY_RIGHT)){
			usingLoader = 0;
		}

		if(joystickGetDigital(2, 8, JOY_DOWN)){
			macroMode = 1;
		}



		mogoLift();
		roller();
		delay(20);
	}
}
