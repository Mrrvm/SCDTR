#ifndef DEFS_h
#define DEFS_h

#include "Arduino.h"

// Defining constants and pins
//#define A -0.61092
//#define B 4.69897
#define VDD 5
#define R1 10000
#define MAXLUX 75
#define HIGH 50
#define LOW 25
#define W 75
//PWM=M*LUX+OFFSET
#define M 2.582
#define MEASURES 1
#define OFFSET 28
#define ERROR_MAX 1
#define N 2
#define C 1

const double A[2]={-0.61,-0.46}; //{-0.61092,-0.57092} for equal gains
const double B[2]={4.69897,4.69897}; //{4.69897} for equal gains

const int analogInPin = A0;
const int analogOutPin = 9;

double lux_converter(double volt);
double voltage_converter(double lux);
double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max);



#endif