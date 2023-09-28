all: udp_client udp_server tst gst_rec expe_main recv_from_x86_send_to_unity expe_main_rec

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

expe_main: expe_main.o
	g++ expe_main.o -o expe_main -pthread

expe_main.o: expe_main.cpp
	g++ -c expe_main.cpp

expe_main_rec: expe_main_rec.o
	g++ expe_main_rec.o -o expe_main_rec -pthread

expe_main_rec.o: expe_main_rec.cpp
	g++ -c expe_main_rec.cpp

recv_from_x86_send_to_unity: recv_from_x86_send_to_unity.o
	g++ recv_from_x86_send_to_unity.o -o recv_from_x86_send_to_unity -pthread

recv_from_x86_send_to_unity.o: recv_from_x86_send_to_unity.cpp
	g++ -c recv_from_x86_send_to_unity.cpp

gst_rec: gst_rec.o
	gcc gst_rec.o -o gst_rec -pthread

gst_rec.o: gst_rec.c
	gcc -c gst_rec.c