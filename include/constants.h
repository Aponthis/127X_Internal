#ifndef CONSTANTS_H_  //File for constant values of heights used  printf("\n%d %f %f %f %f", four_bar.actual, four_bar.error, four_bar.Kp*four_bar.error, four_bar.Ki*four_bar.integral, four_bar.Kd*four_bar.derivative);
#define CONSTANTS_H_

#define MOGOBOTTOM -100
#define MOGOTOP -5  //Top of mogo lift potentiometer reading

#define STATGOANGLE 1800 //Angle for 4-bar to place cone on stationary goal
#define STACKANGLE 2770 //Angle for 4-bar to stack cones
#define HOLDANGLE 950  //Angle to hold out cone safely
#define SAFEANGLE 820  //Angle that DR4B can start to safely move up
#define PICKUPANGLE 780  //Angle for 4-bar to pick up cones

#define DRIVEKP 0.020
#define DRIVEKI 0.001
#define DRIVEKD 0.005

//Autonomous values

#define MOGOHEIGHT 15  //Height to remove lift from way


#define STATGODROP 600  //FAKE NEWS
#define STATGO1 900  //Height to stack 1st statgo cone
#define STATGO2 1000  //Height to stack 2nd statgo cone
#define STATGO3 1200  //Height to stack 3rd statgo cone
#define LIFTTOP 80  //Height to max out lift
#define LIFTBOTTOM 0  //lower than 0 to give a little passive power

#endif
