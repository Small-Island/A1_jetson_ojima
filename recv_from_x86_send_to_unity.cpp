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

struct A1Status {
    float lift;
    float roll;
    float pitch;
    int16_t foot1;
    int16_t foot2;
    int16_t foot3;
    int16_t foot4;
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

    struct A1Status a1Status;
    printf("time,lift,roll,pitch,foot1,foot2,foot3,foot4\n");
    uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double init_time = time_since_epoch/1000.0;
    while (1) {
        int recv_size = recv(sockfd_recv, &a1Status, sizeof(struct A1Status), 0);
        uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        double time = time_since_epoch/1000.0 - init_time;
        printf(
            "%lf,%lf,%lf,%lf,%d,%d,%d,%d\n",
            time, a1Status.lift, a1Status.roll, a1Status.pitch,
            a1Status.foot1, a1Status.foot2, a1Status.foot3, a1Status.foot4
        );

        sendto(sockfd_send, &a1Status, sizeof(struct A1Status), 0, (struct sockaddr *)&addr_send, sizeof(addr_send));

        sleep(0.1);
    };
    return 0;
}
