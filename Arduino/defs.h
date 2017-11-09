// Defining constants and pins
#define A -0.61092
#define B 4.69897
#define VDD 5
#define R1 10000
#define MAXLUX 75
#define HIGH 50
#define LOW 25
#define W 75
//PWM=M*LUX+OFFSET
#define M 2.582
#define OFFSET 28
#define ERROR_MAX 1

double lux_converter(double volt);
double voltage_converter(double lux);
double map_double(double vi, int vi_min, int vi_max, int vo_min, int vo_max);