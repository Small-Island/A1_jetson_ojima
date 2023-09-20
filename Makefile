all: udp_client udp_server tst exrec

udp_client: udp_client.o
	g++ udp_client.o -o udp_client -pthread

udp_client.o: udp_client.cpp
	g++ -c udp_client.cpp

udp_server: udp_server.o
	g++ -o udp_server udp_server.o -pthread

udp_server.o: udp_server.cpp
	g++ -c udp_server.cpp -o udp_server.o

tst: tst.cpp
	g++ tst.cpp -o tst

exrec: exrec.o
	g++ exrec.o -o exrec -pthread

exrec.o: exrec.cpp
	g++ -c exrec.cpp