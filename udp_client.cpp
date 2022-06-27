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


void momo_to_x86(void) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.123.161");
    addr.sin_port = htons(4001);

    int fd_read = open("./serial_out", O_RDONLY);
    while (1) {
        int req_size = 100*sizeof(uint8_t);
        uint8_t buf_ptr[100] = {0};

        int read_size = read(fd_read, buf_ptr, req_size);
        printf("from momo %d byte: %02x %02x %02x %02x\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);

        if (read_size == 4) {
            if (buf_ptr[0] == 0xa1) {
                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            }
            else if (buf_ptr[0] == 0xa2) {
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
        if (recv_size == 5) {
            // buf_ptr[5] = 10;
            write(fd_write, buf_ptr, 5);
            printf("to momo %d byte: %02x %02x %02x %02x %02x\n", recv_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3], buf_ptr[4]);
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
