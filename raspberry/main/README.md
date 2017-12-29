To run the program manually check how its done in `startup.sh`. Else just run it.

<hr>

`I2Csniff.c` sniffs the communications between the arduinos and using a FIFO sends the information to the server side.

To have it working, your setup must be similar to this:

![I2C connection](https://github.com/Mrrvm/SCDTR/blob/master/papers/level_shifters.png "I2C connection")

where 5V devices are the arduinos, 3.3V devices are the raspberry Pis, SDA_1 is rapberry pin 03 and SCL_1 is raspberry pin 05.

You must also **connect the ground** of the arduinos to the ground of the raspberry Pis, else you won't have a circuit.

<hr>

`server.cpp` is the main server file. It generates 2 threads: the main thread receives requests from the client and sends the respective responses;
the other thread (sniff thread) receives the sniffed information from the FIFO and saves it.

<hr>

`data.cpp` and `data.h` are responsible for saving and making calculations with the retrieved information.

<hr>

`resolve_get.cpp` returns the client responses according to the request.
