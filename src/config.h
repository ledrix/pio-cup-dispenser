#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define DEBUG
// #define RELEASE

// #define KUKA_ROBOT
#define MODBUS_ROBOT

#define NO_LIM_SW

const char* name        = "Cup";

byte ETH_IP[]           = {192, 168, 15, 40};
byte ETH_MAC[]          = {0xD0, 0xD0, 0xD0, 0xD0, 0xD0, 0xD0};

const char* MQTT_IP     = "192.168.15.93";
const int   MQTT_RATE   = 100; //Hz

#ifdef KUKA_ROBOT
const char* KUKA_IP     = "192.168.15.147";
const char* KUKA_VAR    = "CUP_DISPENSER";
const int   KUKA_RATE   = 10; //Hz
#endif //KUKA

#ifdef MODBUS_ROBOT
const char* MODBUS_IP   = "192.168.15.46";
const int   MODBUS_ADDR = 901;
const int   MODBUS_RATE = 100; //Hz
#endif //MODBUS_ROBOT

const double R_SHUNT = 0.05; //Ohm

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