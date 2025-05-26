#ifndef PCB_H
#define PCB_H

#include <Arduino.h>

const int ETH_CS        =  5;
const int ETH_INT       = 39;

const int dirPin        = 12;
const int stepPin       = 14;
const int enablePin[]   = { 33, 25, 26, 27};
const int modePin[]     = { 17, 16,  4};

const int pullerPin     = 13;
const int sensorPin[]   = { 36, 34, 35, 32};

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

#endif //!PCB_H