#include "defs.h"

// Convert voltage to lux
double lux_converter(double volt){
  double l=pow(((VDD/volt)-1)*R1/pow(10,B),1/A);
  return l;
}

//Convert lux to voltage
double voltage_converter(double lux){
  double v=(VDD)/(1+pow(10,B)*pow(lux,A)/R1);
  return v;
}

//Maps variables with double type
double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max){
  return (double)(vi-vi_min)*(vo_max-vo_min)/(double)(vi_max-vi_min)+vo_min;
}