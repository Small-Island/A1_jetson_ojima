#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
// #include <string>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sys/fcntl.h>
#include <thread>

#include <chrono>

struct ThreeDOF {
    float lift;
    float roll;
    float pitch;
};

int main(int argc, char* argv[]) {
    int sockfd_recv = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_recv;
    addr_recv.sin_family = AF_INET;
    addr_recv.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr_recv.sin_port = htons(4123);
    bind(sockfd_recv, (const struct sockaddr *)&addr_recv, sizeof(addr_recv));

    int sockfd_send = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_send;
    addr_send.sin_family = AF_INET;
    addr_send.sin_addr.s_addr = inet_addr("192.168.10.205");
    addr_send.sin_port = htons(4123);

    struct ThreeDOF threeDOF;
    printf("time,lift,roll,pitch\n");
    uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double init_time = time_since_epoch/1000.0;
    while (1) {
        int recv_size = recv(sockfd_recv, &threeDOF, sizeof(struct ThreeDOF), 0);
        uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        double time = time_since_epoch/1000.0 - init_time;
        printf("%d,%lf,%lf,%lf,%lf\n", recv_size, time, threeDOF.lift, threeDOF.roll, threeDOF.pitch);

        sendto(sockfd_send, &threeDOF, sizeof(struct ThreeDOF), 0, (struct sockaddr *)&addr_send, sizeof(addr_send));

        sleep(0.1);
    };
    return 0;
}
