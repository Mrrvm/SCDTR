`main/` has all the files you should upload to arduino.

To make it work you should [write to the arduino's EEPROM](https://www.arduino.cc/en/Tutorial/EEPROMWrite) its address. 
For example, for 2 arduinos, you could write to one 1 and to the other 2.

You should have a similar setup:

![I2C connection](https://github.com/Mrrvm/SCDTR/blob/master/papers/level_shifters.png "I2C connection")

where 5V devices are the arduinos, SDA_2 is A4 pin, SCL_2 is A5 pin (for arduino UNO), and R_p are pull up resistor.

The resistor values can range from 1k8 (1800 ohms) to 47k (47000 ohms).
