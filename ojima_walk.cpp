/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cerrno>

#include <chrono>
#include <thread>
#include <mutex>

#include <cmath>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom():
    control(LeggedType::A1, HIGHLEVEL), udp()
    {
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    void momoUDPRecv();
    void HighStateRecv();

    void realsenseUDPRecv();

    Control control;
    UDP udp;
    HighCmd cmd = {0};
    HighState highstate = {0};
    int motiontime = 0;
    float dt = 0.005;     // 0.001~0.01

    int sockfd;
    struct sockaddr_in addr;


    std::chrono::system_clock::time_point jyja_arrival_time;

    int mode;

    std::mutex mutex;

    int show_count = 0;

    int auto_moving_state = 0; //0: not auto_moving, 1: auto_moving
    double z = 0, x = 0;

    bool robot_control = false;

    char obstacle_detected_in_0_5m, obstacle_detected_in_1_0m;

    double momo_rotateSpeed = 0;
    double momo_sideSpeed = 0;
    double momo_forwardSpeed = 0;
    double momo_mode = 0;
    double momo_roll = 0;
    double momo_pitch = 0;
    double momo_yaw = 0;
};


void Custom::UDPRecv() {
    if (robot_control) {
        udp.Recv();
    }
}

void Custom::momoUDPRecv() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4001);
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "Error bind:" << std::strerror(errno);
        exit(1);
    }

    while (1) {
        const int N = 4;
        uint8_t buf_ptr[N] = {0};
        int read_size = recv(sockfd, buf_ptr, sizeof(buf_ptr), 0);

        // printf("read %d byte: %02x %02x %02x %02x\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);

        // printf("read %d byte: %08x\n", read_size, buf_ptr[0]);

        if (read_size == 4) {
            if (buf_ptr[0] == 0xa1) {
                (*this).jyja_arrival_time = std::chrono::system_clock::now();
                // cmd.sideSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;

                // if (abs((int8_t)buf_ptr[1]) < 20 && abs((int8_t)buf_ptr[2]) < 20 && abs((int8_t)buf_ptr[3]) < 20 ) {
                //     (*this).momo_sideSpeed = 0;
                //     (*this).momo_rotateSpeed = 0;
                //     (*this).momo_forwardSpeed = 0;
                //     (*this).momo_roll  = 0;
                //     (*this).momo_pitch = 0;
                //     (*this).momo_yaw = 0;
                //     (*this).momo_mode = 1;
                //     if (auto_moving_state == 0) {
                //         robot_control = false;
                //     }
                // }
                if (abs((int8_t)buf_ptr[2]) < 10) {
                    (*this).momo_sideSpeed = 0;
                    (*this).momo_rotateSpeed = 0;
                    (*this).momo_forwardSpeed = 0;
                    (*this).momo_roll  = 0;
                    (*this).momo_pitch = 0;
                    (*this).momo_yaw = 0;
                    (*this).momo_mode = 1;
                    if (auto_moving_state == 0) {
                        robot_control = false;
                    }
                }
                else {
                    // (*this).cmd.sideSpeed = 0.5*(int8_t)buf_ptr[1]/127.0;
                    (*this).momo_rotateSpeed = 45.0/180.0*M_PI*(int8_t)buf_ptr[2] /127.0;
                    // (*this).momo_rotateSpeed = 0;
                    // (*this).momo_forwardSpeed = 0.5*(int8_t)buf_ptr[3] /127.0;
                    (*this).momo_roll  = 0;
                    (*this).momo_pitch = 0;
                    (*this).momo_yaw = 0;
                    // (*this).momo_rotateSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                    // (*this).momo_forwardSpeed = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                    (*this).momo_mode = 2;
                    robot_control = true;
                }
            }
            else if (buf_ptr[0] == 0xa4) {
                (*this).x = 1.0f*(int8_t)buf_ptr[2]/10.0;
                (*this).z = 1.0f*(int8_t)buf_ptr[3]/10.0;
                (*this).auto_moving_state = 1;
                (*this).robot_control = true;
            }
            else if (buf_ptr[0] == 0xaa) {
                (*this).jyja_arrival_time = std::chrono::system_clock::now();
                // cmd.sideSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                if (buf_ptr[1] == 0x00 && buf_ptr[2] == 0x00 && buf_ptr[3] == 0x00 ) {
                    (*this).momo_sideSpeed = 0;
                    (*this).momo_rotateSpeed = 0;
                    (*this).momo_forwardSpeed = 0;
                    (*this).momo_roll  = 0;
                    (*this).momo_pitch = 0;
                    (*this).momo_yaw = 0;
                    (*this).momo_mode = 1;
                }
                else {
                    (*this).momo_sideSpeed = 0;
                    (*this).momo_rotateSpeed = 0;
                    (*this).momo_forwardSpeed = 0;
                    (*this).momo_roll = -(int8_t)buf_ptr[1] /127.0;
                    (*this).momo_pitch = (int8_t)buf_ptr[3] /127.0;
                    // (*this).momo_rotateSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                    // (*this).momo_forwardSpeed = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                    (*this).momo_mode = 1;
                }
            }
            else if (buf_ptr[0] == 0x99 && buf_ptr[1] == 0x99 && buf_ptr[2] == 0x99 & buf_ptr[3] == 0x99) {
                (*this).momo_sideSpeed = 0;
                (*this).momo_rotateSpeed = 0;
                (*this).momo_forwardSpeed = 0;
                (*this).momo_mode = 1;
                (*this).auto_moving_state = 0;
            }
        }
    }

    return;

}

void Custom::realsenseUDPRecv() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4003);
    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));
    struct My_udp_data {
        char obstacle_detected_in_0_5m = 0;
        char obstacle_detected_in_1_0m = 0;
    };

    while (1) {
        struct My_udp_data my_udp_data;
        my_udp_data.obstacle_detected_in_0_5m = 0;
        my_udp_data.obstacle_detected_in_1_0m = 0;
        int recv_size = recv(sockfd, &my_udp_data, sizeof(struct My_udp_data), 0);
        (*this).obstacle_detected_in_0_5m = my_udp_data.obstacle_detected_in_0_5m;
        (*this).obstacle_detected_in_1_0m = my_udp_data.obstacle_detected_in_1_0m;
    }
}

void Custom::UDPSend()
{
    if (robot_control) {
        udp.Send();
    }
}


void Custom::RobotControl()
{
    // motiontime += 2;

    // cmd.bodyHeight = 1.0f;

    // if (motiontime < 1000) {
    //     cmd.forwardSpeed = 0.0f;
    //     cmd.sideSpeed = 0.0f;
    //     cmd.rotateSpeed = 0.0f;
    //     cmd.mode = 1;
    // }
    //
    // if (motiontime > 1000) {
    //     cmd.mode = 2;
    // }

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.123.12");
    addr.sin_port = htons(4002);

    while (1) {
        udp.GetRecv(highstate);
        // printf("forwardSpeed %lf\n", highstate.forwardSpeed);

        if (show_count > 10) {
            int p_x = highstate.sidePosition* 100.0;
            uint8_t hx = (uint8_t)((uint16_t)(p_x & 0xff00) >> 8);
            uint8_t lx = (uint8_t)(p_x & 0x00ff);

            int p_z = highstate.forwardPosition * 100.0;
            uint8_t hz = (uint8_t)((uint16_t)(p_z & 0xff00) >> 8);
            uint8_t lz = (uint8_t)(p_z & 0x00ff);

            uint8_t buf_ptr[6] = {0xa5, (uint8_t)auto_moving_state, hx, lx, hz, lz};
            sendto(sockfd, buf_ptr, 6*sizeof(uint8_t), 0, (struct sockaddr *)&addr, sizeof(addr));
            printf("%d, auto_moving_state %d forwardPosition %lf sidePosition %lf\n", robot_control, auto_moving_state, highstate.forwardPosition, highstate.sidePosition);
            show_count = 0;
        }
        show_count++;

        mutex.lock();
        if ((*this).robot_control) {
            if ((*this).auto_moving_state == 0) {
                if (std::chrono::system_clock::now() - this->jyja_arrival_time < std::chrono::milliseconds(250)) {
                    (*this).cmd.rotateSpeed = (*this).momo_rotateSpeed;
                    (*this).cmd.forwardSpeed = -0.05f*highstate.forwardPosition/fabs(highstate.forwardPosition);
                    (*this).cmd.sideSpeed = -0.3f*highstate.sidePosition/fabs(highstate.sidePosition);
                    (*this).udp.SetSend((*this).cmd);
                }
                else {
                    (*this).cmd.forwardSpeed = 0.0f;
                    (*this).cmd.sideSpeed = 0.0f;
                    (*this).cmd.rotateSpeed = 0.0f;
                    (*this).cmd.roll  = 0;
                    (*this).cmd.pitch = 0;
                    (*this).cmd.yaw = 0;
                    (*this).cmd.mode = 1;
                    (*this).udp.SetSend((*this).cmd);
                    (*this).robot_control = false;
                }
            }
            else if ((*this).auto_moving_state == 1) {
                (*this).cmd.forwardSpeed = 0;
                (*this).cmd.rotateSpeed = 0;
                (*this).cmd.sideSpeed = 0;
                (*this).cmd.mode = 1;
                if ((*this).obstacle_detected_in_0_5m == 1 && (*this).z > 0) {
                    (*this).auto_moving_state = 0;
                    (*this).udp.SetSend((*this).cmd);
                }
                else {
                    if (fabs((*this).z - highstate.forwardPosition) > 0.05) {
                        (*this).cmd.forwardSpeed = 0.1f*((*this).z - highstate.forwardPosition)/fabs((*this).z - highstate.forwardPosition);
                        if ((*this).cmd.forwardSpeed < 0) {
                            (*this).cmd.forwardSpeed = 3.0*(*this).cmd.forwardSpeed;
                        }
                        (*this).cmd.mode = 2;
                    }
                    if (fabs((*this).x + highstate.sidePosition) > 0.05) {
                        (*this).cmd.sideSpeed = -0.5f*((*this).x + highstate.sidePosition)/fabs((*this).x + highstate.sidePosition);
                        (*this).cmd.mode = 2;
                    }

                    if ((*this).cmd.mode == 2) {
                        (*this).udp.SetSend((*this).cmd);
                    }
                    else {
                        // cmd.forwardSpeed = 0.0f;
                        // cmd.sideSpeed = 0.0f;
                        // cmd.rotateSpeed = 0.0f;
                        // cmd.mode = 1;
                        printf("\n\n==============================================\n=============complete auto_moving=============\n==============================================\n\n");
                        (*this).auto_moving_state = 0;
                    }
                }
            }
        }
        mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(void)
{
    std::cout << "Control level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    Custom custom;

    // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));


    loop_udpSend.start();
    loop_udpRecv.start();
    // loop_control.start();

    // std::thread th_HighStateRecv(&Custom::HighStateRecv, &custom);
    std::thread th_momoUDPRecv(&Custom::momoUDPRecv, &custom);
    std::thread th_realsenseUDPRecv(&Custom::realsenseUDPRecv, &custom);
    // std::thread th_user_input(&Custom::user_input, &custom);

    std::thread th_RobotControl(&Custom::RobotControl, &custom);

    while(1){
        sleep(10);
    };

    return 0;
}
