
#include "main.h"

int deadband(int value){
 if(value > 0){
    if (abs(value) > 120){
      value = 127;
    } else if (abs(value) > 110){
      value = 100;
    } else if (abs(value) > 100){
      value = 85;
    } else if (abs(value) > 90){
      value = 70;
    } else if (abs(value) > 80){
      value = 60;
    } else if (abs(value) > 70){
      value = 50;
    } else if (abs(value) > 60){
      value = 43;
    } else if (abs(value) > 50){
      value = 37;
    } else if (abs(value) > 40){
      value = 31;
    } else if (abs(value) > 20){
      value = 25;
    } else {
      value = 0;
    }
  } else {
    if (abs(value) > 120){
      value = -127;
    } else if (abs(value) > 110){
      value = -100;
    } else if (abs(value) > 100){
      value = -85;
    } else if (abs(value) > 90){
      value = -70;
    } else if (abs(value) > 80){
      value = -60;
    } else if (abs(value) > 70){
      value = -50;
    } else if (abs(value) > 60){
      value = -43;
    } else if (abs(value) > 50){
      value = -37;
    } else if (abs(value) > 40){
      value = -31;
    } else if (abs(value) > 20){
      value = -25;
    } else {
      value = 0;
    }
  }
  return value;
}
