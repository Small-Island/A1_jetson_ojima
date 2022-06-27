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

#include <thread>

void from_nx() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4001);

    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));
    // int fd_read = open("./serial_out", O_RDONLY);

    while (1) {
        uint8_t buf_ptr[100] = {0};
        int recv_size = recv(sockfd, buf_ptr, sizeof(buf_ptr), 0);
        printf("read %d byte: %02x %02x %02x %02x\n", recv_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);
    }
    close(sockfd);
}

void to_nx() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.1.205");
    addr.sin_port = htons(4002);

    while (1) {
        uint8_t buf_ptr[100] = {0xab, 0xcd, 0xef, 0x00};
        sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    close(sockfd);
}

int main(int argc, char* argv[]) {

    std::thread th_to_nx(to_nx);
    std::thread th_from_nx(from_nx);

    while(1){
        sleep(10);
    };

    return 0;
}
