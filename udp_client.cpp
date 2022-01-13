#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sys/fcntl.h>

int main(int argc, char* argv[]) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.123.161");
    addr.sin_port = htons(4001);

    int fd_read = open("./serial_out", O_RDONLY);
    while (1) {
        int req_size = 100;
        char buf_ptr[50];
        int read_size = 0;
        read_size = read(fd_read, buf_ptr, req_size);

        if (read_size < 1) {
            continue;
        }
        buf_ptr[read_size] = '\0';
        std::string str = std::string(buf_ptr);
        sendto(sockfd, str.c_str(), str.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
    }
    close(sockfd);
    return 0;
}
