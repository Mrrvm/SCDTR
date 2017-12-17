#!/bin/sh

sudo pigpiod -s 2
OUTPUT="$(pigs no)"
pigs nb ${OUTPUT} 0xC
make
gcc -o sniff I2Csniff.c
./pig2i2c 3 2 </dev/pigpio${OUTPUT} > log 2>&1 &
./server