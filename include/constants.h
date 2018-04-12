#ifndef CONSTANTS_H_  //File for constant values of heights used  printf("\n%d %f %f %f %f", four_bar.actual, four_bar.error, four_bar.Kp*four_bar.error, four_bar.Ki*four_bar.integral, four_bar.Kd*four_bar.derivative);
#define CONSTANTS_H_

#define MOGOBOTTOM -105
#define MOGOTOP -5  //Top of mogo lift quad encoder reading

#define STATGOANGLE 2650 //Angle for 4-bar to place cone on stationary goal
#define STACKANGLE 3220 //Angle for 4-bar to stack cones
#define HOLDANGLE 1300  //Angle to hold out cone safely
#define SAFEANGLE 1080  //Angle that DR4B can start to safely move up
#define PICKUPANGLE 1120  //Angle for 4-bar to pick up cones

#define DRIVEKP 0.020
#define DRIVEKI 0.001
#define DRIVEKD 0.005

//Autonomous values

#define MOGOHEIGHT 14  //Height to remove lift from way of mobile goal mechanism


#define STATGO1 42  //Height to stack 1st statgo cone (currently a guesstimate)
#define STATGO2 46  //Height to stack 2nd statgo cone
#define STATGO3 51  //Height to stack 3rd statgo cone
#define LIFTTOP 80  //Height to max out lift
#define LIFTBOTTOM 0  //

#endif
