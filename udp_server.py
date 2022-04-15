#!/usr/bin/python3

from socket import *
import numpy as np

IP = "0.0.0.0"
Port = 4002
Addr = (IP, Port)
BUFSIZE = 1024
udpServSock = socket(AF_INET, SOCK_DGRAM)
udpServSock.bind(Addr)

while 1:
    data, addr = udpServSock.recvfrom(BUFSIZE)
    print(np.array(data[0], dtype='int8'), addr)
