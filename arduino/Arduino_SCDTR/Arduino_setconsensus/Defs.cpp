#include "Arduino.h"
#include "Defs.h"

extern int my_address;

// Convert voltage to lux
double lux_converter(double volt){
  double l=pow(((VDD/volt)-1)*R1/pow(10,B[my_address-1]),1/A[my_address-1]);
  return l;
}

//Convert lux to voltage
double voltage_converter(double lux){
  double v=(VDD)/(1+pow(10,B[my_address-1])*pow(lux,A[my_address-1])/R1);
  return v;
}

//Maps variables with double type
double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max){
  return (double)(vi-vi_min)*(vo_max-vo_min)/(double)(vi_max-vi_min)+vo_min;
}