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

int main(int argc, char* argv[]) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.123.161");
    addr.sin_port = htons(4001);

    int fd_read = open("/home/tristar/A1_jetson_ojima/serial_out", O_RDONLY);

    while (1) {
        // int req_size = 30;
        // char buf_ptr[50];
        // int read_size = 0;
        // // memset(buf_ptr, 0, sizeof(buf_ptr));
        // // read_size = read(fd_read, buf_ptr, req_size);
        //
        // if (read_size < 1) {
        //     continue;
        // }
        // buf_ptr[read_size] = '\0';
        // uint8_t buf_ptr[6] = "ojima";
        // // std::string str = std::string(buf_ptr);
        // // printf("%d   %s\n", read_size, buf_ptr);

        int req_size = 100*sizeof(uint8_t);
        uint8_t buf_ptr[100] = {
            0
        };

        int read_size = read(fd_read, buf_ptr, req_size);
        printf("read %d byte: %02x %02x %02x %02x\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);
        // printf("read %d byte: %d, %d\n", read_size, (int8_t)((buf_ptr[0] & 0xff00) >> 8), (int8_t)(buf_ptr[0] & 0x00ff));


        if (read_size == 4) {
            if ((buf_ptr[0] & 0xff) == 0x43) {
                // segway_rmp::jyja msg;
                // printf("%lf %lf\n", 50*(int8_t)buf_ptr[2] /127.0, 1.0*(int8_t)buf_ptr[3] /127.0);
                // jyja_pub.publish(msg);

                sendto(sockfd, buf_ptr, 4*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            }
            else if (buf_ptr[0] == 0x11111111) {
                sendto(sockfd, buf_ptr, 1*sizeof(int32_t), 0, (struct sockaddr *)&addr, sizeof(addr));
                // segway_rmp::AccelCmd msg;
                // msg.T2 = 20*(int8_t)((buf_ptr[0] & 0x00ff0000) >> 12)/127.0;
                // msg.a = 0.5*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                // msg.vel_limit = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                // msg.reverse = 1;
                // accel_pub.publish(msg);
            }
            else if (buf_ptr[0] == 0x99999999) {
                sendto(sockfd, buf_ptr, 1*sizeof(int32_t), 0, (struct sockaddr *)&addr, sizeof(addr));
                // segway_rmp::AccelCmd msg;
                // msg.T2 = 20*(int8_t)((buf_ptr[0] & 0x00ff0000) >> 12)/127.0;
                // msg.a = 0.5*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                // msg.vel_limit = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                // msg.reverse = 1;
                // accel_pub.publish(msg);
            }
        }

    }
    close(sockfd);
    return 0;
}
