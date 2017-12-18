#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>

#include "pigpio.h"

/*
This software reads pigpio notification reports monitoring the I2C signals.

Notifications are pipe based so this software must be run on the Pi
being monitored.

It should be able to handle a 100kHz bus.  You are unlikely to get any
usable results if the bus is running at 400kHz.

gcc -o pig2i2c pig2i2c.c

Do something like

sudo pigpiod -s 2

# get a notification handle, assume handle 0 was returned

pigs no

# start notifications for SCL/SDA

e.g. pigs nb 0 0x3   # Rev. 1 select gpios 0/1
e.g. pigs nb 0 0xC   # Rev. 2 select gpios 2/3
e.g. pigs nb 0 0xA00 # select gpios 9/11 (1<<9|1<<11)

# run the program, specifying SCL/SDA and notification pipe

./pig2i2c SCL SDA </dev/pigpioN # specify gpios for SCL/SDA and pipe N

e.g. ./pig2i2c 1  0 </dev/pigpio0 # Rev.1 I2C gpios
e.g. ./pig2i2c 3  2 </dev/pigpio0 # Rev.2 I2C gpios
e.g. ./pig2i2c 9 11 </dev/pigpio0 # monitor external bus 
*/

#define RS (sizeof(gpioReport_t))

#define SCL_FALLING 0
#define SCL_RISING  1
#define SCL_STEADY  2

#define SDA_FALLING 0
#define SDA_RISING  4
#define SDA_STEADY  8

int argv1, argv2;

int parse_I2C(int SCL, int SDA)
{
   static int in_data=0, byte=0, bit=0, ret_byte=0;
   static int oldSCL=1, oldSDA=1;

   int xSCL, xSDA;


   if (SCL != oldSCL)
   {
      oldSCL = SCL;
      if (SCL) xSCL = SCL_RISING;
      else     xSCL = SCL_FALLING;
   }
   else        xSCL = SCL_STEADY;

   if (SDA != oldSDA)
   {
      oldSDA = SDA;
      if (SDA) xSDA = SDA_RISING;
      else     xSDA = SDA_FALLING;
   }
   else        xSDA = SDA_STEADY;

   switch (xSCL+xSDA)
   {
      case SCL_RISING + SDA_RISING:
      case SCL_RISING + SDA_FALLING:
      case SCL_RISING + SDA_STEADY:
         if (in_data)
         {
            if (bit++ < 8)
            {
               byte <<= 1;
               byte |= SDA;
            }
            else
            {
               ret_byte = byte;
               bit = 0;
               byte = 0;
            }
         }
         break;

      case SCL_FALLING + SDA_RISING:
         break;

      case SCL_FALLING + SDA_FALLING:
         break;

      case SCL_FALLING + SDA_STEADY:
         break;

      case SCL_STEADY + SDA_RISING:
         if (SCL)
         {
            in_data = 0;
            byte = 0;
            bit = 0;
         }
         break;

      case SCL_STEADY + SDA_FALLING:
         if (SCL)
         {
            in_data = 1;
            byte = 0;
            bit = 0;
         }
         break;

      case SCL_STEADY + SDA_STEADY:
         break;

   }
   return ret_byte;
}

void run_sniffer() {

   int gSCL, gSDA, SCL, SDA, xSCL;
   int r;
   uint32_t level, changed, bI2C, bSCL, bSDA;
   clock_t t1;
   int byte = 0, prev_byte = 0;
   int fd;
   char * myfifo = "myfifo";
   char buff[64];
   char val[8];
   int counter = 0;
   mkfifo(myfifo, 0666);

   fd = open(myfifo, O_WRONLY);
   gpioReport_t report;

   gSCL = argv1;
   gSDA = argv2;

   bSCL = 1<<gSCL;
   bSDA = 1<<gSDA;

   bI2C = bSCL | bSDA;

   /* default to SCL/SDA high */

   SCL = 1;
   SDA = 1;
   level = bI2C;


   while ((r=read(STDIN_FILENO, &report, RS)) == RS)
   {
      report.level &= bI2C;

      if (report.level != level)
      {
         changed = report.level ^ level;

         level = report.level;

         if (level & bSCL) SCL = 1; else SCL = 0;
         if (level & bSDA) SDA = 1; else SDA = 0;

         byte = parse_I2C(SCL, SDA);
         if(byte != prev_byte) {
            
            if(prev_byte == 97) {
               snprintf(val, 8, "a%d", byte);
               strcat(buff, val);
            }
            else if(prev_byte == 100) {
               snprintf(val, 8, "d%d", byte);
               strcat(buff, val);
            }
            else if(prev_byte == 108) {
               snprintf (val, 8, "l%d", byte);
               strcat(buff, val);    
            }
            else if(prev_byte == 111) {
               snprintf (val, 8, "o%d\n", byte);
               strcat(buff, val);
               if(write(fd, buff, strlen(buff)+1) < 0) {
                  printf("Error\n");
                  exit(0);
               }
               printf("%s", buff);
               memset(&buff[0], 0, sizeof(buff));
            }
	    else if(prev_byte == 105){
	      snprintf(val,8, "i",byte);
	      strcat(buff,val);
	    }
	    else if(prev_byte == 116){
	      snprintf(val, 8, "l",byte);
	      strcat(buff,val);
	    }
	     else if(prev_byte == 120) {
               snprintf (val, 8, "x%d\n", byte);
               strcat(buff, val);
               if(write(fd, buff, strlen(buff)+1) < 0) {
                  printf("Error\n");
                  exit(0);
               }
         }
         prev_byte = byte;     
      }
   }
   close(fd);
}

void handler(int s) {run_sniffer();}

int main(int argc, char *argv[])
{
   if (argc > 2) {
      argv1 = atoi(argv[1]);
      argv2 = atoi(argv[2]);
   }
   else {
      exit(-1);
   }
   signal(SIGPIPE, handler);
   printf("going to run sniffer\n");
   run_sniffer();
   
   return 0;
}

