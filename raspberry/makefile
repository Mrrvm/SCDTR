server: server.o  resolve_get.o data.o
		g++ -o server server.o data.o resolve_get.o -std=c++11 -lboost_system -pthread
server.o: server.cpp
		g++ -c -pthread  server.cpp -std=c++11
resolve_get.o: resolve_get.cpp
		g++ -c -pthread  resolve_get.cpp -std=c++11
data.o: data.cpp
		g++ -c -pthread  data.cpp -std=c++11