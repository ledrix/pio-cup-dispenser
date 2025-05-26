#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define DEBUG
// #define RELEASE

//#define KUKA_ROBOT
#define NEUROMEKA_ROBOT

#define NO_LIM_SW

const char* name        = "Cup";

byte ETH_IP[]           = {192, 168, 15, 40};
byte ETH_MAC[]          = {0xD0, 0xD0, 0xD0, 0xD0, 0xD0, 0xD0};

const char* MQTT_IP     = "192.168.15.93";
const int   mqtt_rate   = 100; //Hz

#ifdef KUKA_ROBOT
const char* kuka_IP       = "192.168.15.147";
const char* kuka_var      = "CUP_DISPENSER";
const int   kuka_refresh  = 10; //Hz
#endif //KUKA

#ifdef NEUROMEKA_ROBOT
const char* neuromeka_IP      = "192.168.15.46";
const int   neuromeka_address = 901;
const int   neuromeka_refresh =  10; //Hz
#endif //NEUROMEKA

const bool NPN = LOW;
const bool PNP = HIGH;

const bool CLOCKWISE = true;
const bool COUNTERCLOCKWISE = false;

const uint16_t STEPS_REV = 200; //Nema 17

const float REVERSE_REVS = 1.0;

const char* STEP_STR[] = { "IDLE",
                           "DISPENSING",
                           "WAITING",
                           "RETRACTING" };

#endif //!CONFIG_H