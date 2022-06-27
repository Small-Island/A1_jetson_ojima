all: udp_client udp_server

udp_client: udp_client.o
	g++ udp_client.o -o udp_client -pthread

udp_client.o: udp_client.cpp
	g++ -c udp_client.cpp

udp_server: udp_server.o
	g++ -o udp_server udp_server.o -pthread

udp_server.o: udp_server.cpp
	g++ -c udp_server.cpp -o udp_server.o
