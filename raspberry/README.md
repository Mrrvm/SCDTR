# Raspberry Pi hosted C++ server

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
