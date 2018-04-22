#include "main.h"
#include "sensors.h"
#include "string.h"

#define MAINPAGE1 1 //Front page of LCD screen
#define MAINPAGE2 1
#define MAXPAGES1 6 //Highest page number
uint8_t maxPages2 = 4;

char line1Text[17]; //Text on first line
char line2Text[17]; //Text on second line

uint8_t lcdPage1Number = MAINPAGE1;
uint8_t lcdPage2Number = MAINPAGE2;
double batteryVoltage;
//uint8_t autonMode was initialized in <main.h>
//uint8_t autonVariation was initialized in <main.h>

bool shouldDelay = 0;

void autonSelect(){
  while(!isEnabled()){
    batteryVoltage = powerLevelMain() / 10;

    while(lcdReadButtons(LCDSCREEN) == 5){  //If pressing left and right buttons simultaneously
      lcdPrint(LCDSCREEN, 1, "%f", batteryVoltage);  //Display battery voltage
      lcdPrint(LCDSCREEN, 2, "^ BATTERY ^");
      delay(20);
    }

    if(autonMode == 0){  //If auton is not selected
      if(lcdReadButtons(LCDSCREEN) == 1){ //If left button is pressed and auton is not selected
      lcdPage1Number -= 1; //Decrease page number
      delay(300);
      }
      if(lcdReadButtons(LCDSCREEN) == 4){ //If right button is pressed and auton is not selected
        lcdPage1Number += 1; //Increase page number
        delay(300);
      }


      if (lcdPage1Number > MAXPAGES1){ //Cycle to beginning from end
        lcdPage1Number = MAINPAGE1;
      }
      if(lcdPage1Number < MAINPAGE1){ //Cycle to end from beginning
        lcdPage1Number = MAXPAGES1;
      }

      if (lcdReadButtons(LCDSCREEN) == 2){ //If middle button is pressed
        autonMode = lcdPage1Number; //select auton
        delay(500);
        if(lcdReadButtons(LCDSCREEN) == 2){
          delay(500);
          if(lcdReadButtons(LCDSCREEN) == 2){
            autonMode = 0;
            autonVariation = 0;
            lcdPage1Number = MAINPAGE1;
          }
        }
      }
    }

    if(autonMode != 0){  //If autonMode is selected
      if((autonVariation == 0) && lcdReadButtons(LCDSCREEN) == 1){ //If left button is pressed and auton is not selected
        lcdPage2Number -= 1; //Decrease page number
        delay(300);
      }
      if((autonVariation == 0) && lcdReadButtons(LCDSCREEN) == 4){ //If right button is pressed and auton is not selected
        lcdPage2Number += 1; //Increase page number
        delay(300);
      }
      if (lcdPage2Number > maxPages2){ //Cycle to beginning from end
        lcdPage2Number = maxPages2;
      }
      if(lcdPage2Number < MAINPAGE2){ //Cycle to end from beginning
        lcdPage2Number = maxPages2;
      }
      if (lcdReadButtons(LCDSCREEN) == 2){ //If middle button is pressed
        autonVariation = lcdPage2Number; //select auton variant
        Gyro gyro = gyroInit(GYRO, 196);  /////////////////////////////////////////////Changed this value from 0
        delay(500);
        gyroReset(gyro);
        if(lcdReadButtons(LCDSCREEN) == 2){  //If holding button
          delay(500);
          if(lcdReadButtons(LCDSCREEN) == 2){  //If still holding button
            autonMode = 0;
            autonVariation = 0;
            lcdPage1Number = MAINPAGE2;
            shouldDelay = 1;
          }
        }
      }
    }

    switch(lcdPage1Number){  //Determines what to display on screen based on which page number
      case 1 :  //1st auton
      strcpy(line1Text, "StatGo");
      switch(lcdPage2Number){
        case 1 :  //1st variation
        strcpy(line2Text, "1 & Wait");
        break;
        case 2 :
        strcpy(line2Text, "Wait & 1");
        break;
        case 3 :
        strcpy(line2Text, "Cone & Charge L");
        break;
        case 4 :
        strcpy(line2Text, "Cone & Charge R");
        break;
        case 5 :
        strcpy(line2Text, "L Juke&Cone");
        break;
        case 6 :
        strcpy(line2Text, "R Juke&Cone");
        break;
      }
      maxPages2 = 6;
      break;
      case 2 :  //2nd auton
      strcpy(line1Text, "Mogo, 1c");
      switch(lcdPage2Number){
        case 1 :  //First variation
        strcpy(line2Text, "5pt Zone L");
        break;
        case 2 :
        strcpy(line2Text, "5pt Zone R");
        break;
        case 3 :
        strcpy(line2Text, "10pt Zone L");
        break;
        case 4 :
        strcpy(line2Text, "10pt Zone R");
        break;
        case 5 :
        strcpy(line2Text, "20pt Zone L");
        break;
        case 6 :
        strcpy(line2Text, "20pt Zone R");
        break;
      }
      maxPages2 = 6;
      break;
      case 3 :  //3rd auton
      strcpy(line1Text, "Charge!");
      switch(lcdPage2Number){
        case 1 :  //1st variation
        strcpy(line2Text, "Stop");
        break;
        case 2 :
        strcpy(line2Text, "Left");
        break;
        case 3 :
        strcpy(line2Text, "Right");
        break;
        case 4 :
        strcpy(line2Text, "Wait, Left");
        break;
        case 5 :
        strcpy(line2Text, "Wait, Right");
        break;
      }
      maxPages2 = 5;
      break;

      case 4 :  //Mogo with two cones
      strcpy(line1Text, "Mogo, 2c");
      switch(lcdPage2Number){
        case 1 :  //First variation
        strcpy(line2Text, "5pt Zone L");
        break;
        case 2 :
        strcpy(line2Text, "5pt Zone R");
        break;
        case 3 :
        strcpy(line2Text, "10pt Zone L");
        break;
        case 4 :
        strcpy(line2Text, "10pt Zone R");
        break;
        case 5 :
        strcpy(line2Text, "20pt Zone L");
        break;
        case 6 :
        strcpy(line2Text, "20pt Zone R");
        break;
      }
      maxPages2 = 6;
      break;

      case 5:  //Mogo with three cones
      strcpy(line1Text, "Mogo, 3c");
      switch(lcdPage2Number){
        case 1 :  //First variation
        strcpy(line2Text, "5pt Zone L");
        break;
        case 2 :
        strcpy(line2Text, "5pt Zone R");
        break;
        case 3 :
        strcpy(line2Text, "10pt Zone L");
        break;
        case 4 :
        strcpy(line2Text, "10pt Zone R");
        break;
        case 5 :
        strcpy(line2Text, "20pt Zone L");
        break;
        case 6 :
        strcpy(line2Text, "20pt Zone R");
        break;
      }
      break;

      case 6: //Autoloader mogo
      strcpy(line1Text, "Autoloader");
      switch(lcdPage2Number){
        case 1 :  //First variation
        strcpy(line2Text, "5pt Zone L");
        break;
        case 2 :
        strcpy(line2Text, "5pt Zone R");
        break;
        case 3 :
        strcpy(line2Text, "10pt Zone L");
        break;
        case 4 :
        strcpy(line2Text, "10pt Zone R");
        break;
      }
      break;
    }

    if(autonMode == 0){  //If autonMode is not selected
      lcdPrint(LCDSCREEN, 1, "<%s>", line1Text);  //If selecting
    } else {
      lcdPrint(LCDSCREEN, 1, "|%s|", line1Text);  //If selected
    }

    if(autonVariation  == 0){  //If auton variation is not selected
      if(autonMode != 0){  //If autonmode is selected
        lcdPrint(LCDSCREEN, 2, "<%s>",line2Text);  //If selecting
      } else {
        lcdPrint(LCDSCREEN, 2, " "); //Print blank if not selecting bottom auton
      }
    } else {
      lcdPrint(LCDSCREEN, 2, "|%s|", line2Text);  //If selected
    }
    if(shouldDelay){
      delay(2000);
      shouldDelay = 0;
    }
    delay(20);
  }
}
