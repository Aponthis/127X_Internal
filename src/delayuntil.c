#include "main.h"

void delayUntil(bool input){
  while(!input){
    delay(20);
  }
}
