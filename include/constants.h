#ifndef CONSTANTS_H_  //File for constant values of heights used  printf("\n%d %f %f %f %f", four_bar.actual, four_bar.error, four_bar.Kp*four_bar.error, four_bar.Ki*four_bar.integral, four_bar.Kd*four_bar.derivative);
#define CONSTANTS_H_

#define MOGOBOTTOM -105
#define MOGOTOP -5  //Top of mogo lift quad encoder reading

#define STATGOANGLE 2000 //Angle for 4-bar to place cone on stationary goal
#define STACKANGLE 1700 //Angle for 4-bar to stack cones
#define HOLDANGLE 1500  //Angle to hold out cone safely
#define SAFEANGLE 1500  //Angle that DR4B can start to safely move up
#define PICKUPANGLE 3700  //Angle for 4-bar to pick up cones

#define DRIVEKP 0.020
#define DRIVEKI 0.001
#define DRIVEKD 0.005

//Autonomous values

#define MOGOHEIGHT 33 //Height to remove lift from way of mobile goal mechanism
#define DROPHEIGHT 5
#define LOADERHEIGHT 26

#define STATGO1 44  //Height to stack 1st statgo cone (currently a guesstimate)
#define STATGO2 47  //Height to stack 2nd statgo cone
#define STATGO3 51  //Height to stack 3rd statgo cone
#define LIFTTOP 80  //Height to max out lift
#define LIFTBOTTOM 0  //

#endif
