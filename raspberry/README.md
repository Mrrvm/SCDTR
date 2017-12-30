# Raspberry Pi hosted C++ server

You should read [main](https://github.com/Mrrvm/SCDTR/tree/master/raspberry/main) README before anything.

To run the server, simply upload the `main/` content to a raspberry pi and run the command

```
sh startup.sh
```

If you want it to start autonomously, you can add it to `rc.local`. Check this [guide](https://www.raspberrypi.org/documentation/linux/usage/rc-local.md).

To run the client, run the following command to compile

```
g++ -o [executable name] -std=c++11 -lboost_system -pthread client.cpp
```
and the following to execute wherever you want:

```
./[executable name] [IP] [PORT]
```
As a client you can send the requests presented in page 15-17 of the [project_goal](https://github.com/Mrrvm/SCDTR/blob/master/project_goal.pdf) + "s c 0/1" to turn off/on the consensus algorithm + "s w 0/90/180" to spin the box window if you have one.
