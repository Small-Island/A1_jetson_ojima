// #include <sys/types.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
// #include <string>
#include <cstring>
#include <iostream>
// #include <iomanip>
// #include <sys/fcntl.h>

int main(int argc, char* argv[]) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4001);

    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));
    // int fd_read = open("./serial_out", O_RDONLY);

    while (1) {
        int32_t buf_ptr[1] = {0};
        int recv_size = recv(sockfd, buf_ptr, sizeof(buf_ptr), 0);
        printf("read %d byte: %08x\n", recv_size, buf_ptr[0]);
    }
    close(sockfd);
    return 0;
}
