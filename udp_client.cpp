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

int ltime = 0;

void momo_to_x86(void) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.123.161");
    addr.sin_port = htons(4001);

    int sockfd_chair = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_chair;
    addr_chair.sin_family = AF_INET;
    addr_chair.sin_addr.s_addr = inet_addr("192.168.11.240");
    addr_chair.sin_port = htons(4009);

    int fd_read = open("./serial_out", O_RDONLY);
    while (1) {
        int req_size = 100*sizeof(uint8_t);
        uint8_t buf_ptr[100] = {0};

        int read_size = read(fd_read, buf_ptr, req_size);
        printf("from momo %d byte: %02x %02x %02x %02x\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);

        if (read_size == 4) {
            if (buf_ptr[0] == 0xa1) {
                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
                if (abs((int8_t)buf_ptr[2]) > 10 || abs((int8_t)buf_ptr[3] > 10)) {
                    uint8_t val[3] = {0xc0};
                    sendto(sockfd_chair, val, sizeof(uint8_t), 0, (struct sockaddr *)&addr_chair, sizeof(addr_chair));
                }
            }
            else if (buf_ptr[0] == 0xa4) {
                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            }
            else if (buf_ptr[0] == 0xaa) {
                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            }
            else if (buf_ptr[0] == 0x11 && buf_ptr[1] == 0x11 && buf_ptr[2] == 0x11 && buf_ptr[3] == 0x11) {
                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            }
            else if (buf_ptr[0] == 0x99 && buf_ptr[1] == 0x99 && buf_ptr[2] == 0x99 && buf_ptr[3] == 0x99) {
                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            }
            // else if (buf_ptr[0] == 0x01 && buf_ptr[1] == 0x01 && buf_ptr[2] == 0x01 && buf_ptr[3] == 0x01) {
            //     sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            // }
            // else if (buf_ptr[0] == 0x09 && buf_ptr[1] == 0x09 && buf_ptr[2] == 0x09 && buf_ptr[3] == 0x09) {
            //     sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            // }
        }
        else if (read_size  == 5) {
            if (buf_ptr[0] == 0x96) {
                ltime = (int)((uint32_t)buf_ptr[1] << 24) + (int)((uint32_t)buf_ptr[2] << 16) + (int)((uint32_t)buf_ptr[3] << 8) + (int)((uint32_t)buf_ptr[4]);
            }
        }

    }
    close(sockfd);
}

void x86_to_momo() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4002);
    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));

    int fd_write = open("./serial_out", O_WRONLY);
    while (1) {
        uint8_t buf_ptr[100] = {0};
        int recv_size = recv(sockfd, buf_ptr, sizeof(buf_ptr), 0);
        if (recv_size == 16) {
            uint32_t time_since_epoch;
            if (ltime == 0) {
                time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - 1661234080000;
            }
            else {
                time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - 1660000000*1000 - ltime*1000;
            }
            buf_ptr[16] = (uint8_t)((uint32_t)(time_since_epoch & 0xff000000) >> 24);
            buf_ptr[17] = (uint8_t)((uint32_t)(time_since_epoch & 0x00ff0000) >> 16);
            buf_ptr[18] = (uint8_t)((uint32_t)(time_since_epoch & 0x0000ff00) >> 8);
            buf_ptr[19] = (uint8_t)(time_since_epoch & 0x000000ff);
            buf_ptr[20] = 10;

            write(fd_write, buf_ptr, 21);
        }
    }
}

int main(int argc, char* argv[]) {

    std::thread th_momo_to_x86(momo_to_x86);
    std::thread th_x86_to_momo(x86_to_momo);

    while(1){
        sleep(10);
    };

    return 0;
}
