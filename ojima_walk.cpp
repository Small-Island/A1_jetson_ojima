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

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom():
    control(LeggedType::A1, HIGHLEVEL), udp(),
    sockfd(socket(AF_INET, SOCK_DGRAM, 0))
    {
        (*this).control.InitCmdData(cmd);

        (*this).addr.sin_family = AF_INET;
        (*this).addr.sin_addr.s_addr = inet_addr("0.0.0.0");
        (*this).addr.sin_port = htons(4001);
        if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            std::cout << "Error bind:" << std::strerror(errno);
            exit(1);
        }
        (*this).start = false;
        (*this).sleep = false;
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    void momoUDPRecv();
    void HighStateRecv();


    Control control;
    UDP udp;
    HighCmd cmd = {0};
    HighState highstate = {0};
    int motiontime = 0;
    float dt = 0.005;     // 0.001~0.01

    int sockfd;
    struct sockaddr_in addr;

    bool start;

    std::chrono::system_clock::time_point jyja_arrival_time;

    bool sleep;

    double forwardPosition = 0, init_fp = 0;
    double sidePosition = 0, init_sp = 0;
    int mode;

    std::mutex mutex;

    int show_count = 0;

    int auto_moving_state = 0; //0: not auto_moving, 1: auto_moving
    double z = 0, x = 0;
};


void Custom::UDPRecv() {
    udp.Recv();
}

void Custom::momoUDPRecv()
{
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

                if (abs((int8_t)buf_ptr[1]) < 20 && abs((int8_t)buf_ptr[2]) < 20 && abs((int8_t)buf_ptr[3]) < 20 ) {
                    (*this).cmd.sideSpeed = 0;
                    (*this).cmd.rotateSpeed = 0;
                    (*this).cmd.forwardSpeed = 0;
                    cmd.roll  = 0;
                    cmd.pitch = 0;
                    cmd.yaw = 0;
                    (*this).cmd.mode = 1;
                }
                else {
                    (*this).cmd.sideSpeed = 0.5*(int8_t)buf_ptr[1]/127.0;
                    (*this).cmd.rotateSpeed = 0.45*(int8_t)buf_ptr[2] /127.0;
                    // (*this).cmd.rotateSpeed = 0;
                    (*this).cmd.forwardSpeed = 0.5*(int8_t)buf_ptr[3] /127.0;
                    cmd.roll  = 0;
                    cmd.pitch = 0;
                    cmd.yaw = 0;
                    // (*this).cmd.rotateSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                    // (*this).cmd.forwardSpeed = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                    (*this).cmd.mode = 2;
                }
            }
            else if (buf_ptr[0] == 0xa2) {
                z = 1.0f*(int8_t)buf_ptr[2]/100.0;
                x = 1.0f*(int8_t)buf_ptr[3]/100.0;
                auto_moving_state = 1;
            }
            else if (buf_ptr[0] == 0xaa) {
                (*this).jyja_arrival_time = std::chrono::system_clock::now();
                // cmd.sideSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                if (buf_ptr[1] == 0x00 && buf_ptr[2] == 0x00 && buf_ptr[3] == 0x00 ) {
                    (*this).cmd.sideSpeed = 0;
                    (*this).cmd.rotateSpeed = 0;
                    (*this).cmd.forwardSpeed = 0;
                    cmd.roll  = 0;
                    cmd.pitch = 0;
                    cmd.yaw = 0;
                    (*this).cmd.mode = 1;
                }
                else {
                    (*this).cmd.sideSpeed = 0;
                    (*this).cmd.rotateSpeed = 0;
                    (*this).cmd.forwardSpeed = 0;
                    (*this).cmd.roll = -(int8_t)buf_ptr[1] /127.0;
                    (*this).cmd.pitch = (int8_t)buf_ptr[3] /127.0;
                    // (*this).cmd.rotateSpeed =  50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                    // (*this).cmd.forwardSpeed = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                    (*this).cmd.mode = 1;
                }
            }
            else if ((buf_ptr[0] & 0xff000000) == 0xab000000) {

            }
            else if ((buf_ptr[0] & 0xff000000) == 0xaf000000) {

            }
            // else if (buf_ptr[0] == 0x99999999) {
            //     (*this).start = false;
            // }
            // else if (buf_ptr[0] == 0x11111111) {
            //     (*this).start = true;
            // }
        }
    }

    return;

}

void Custom::UDPSend()
{
    udp.Send();
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

    udp.GetRecv(highstate);
    // printf("forwardSpeed %lf\n", highstate.forwardSpeed);

    if (show_count > 50) {
        printf("auto_moving_state %d forwardPosition %lf sidePosition %lf\n", auto_moving_state, highstate.forwardPosition, highstate.sidePosition);
        show_count = 0;
    }
    show_count++;
    // if (highstate.forwardPosition < 0.1 && forwardPosition > 0.1) {
    //     init_fp = forwardPosition;
    //     printf("init_fp %lf\n", init_fp);
    // }
    forwardPosition = highstate.forwardPosition;
    sidePosition = highstate.sidePosition;

    mutex.lock();
    if (auto_moving_state == 0) {
        if (std::chrono::system_clock::now() - this->jyja_arrival_time < std::chrono::milliseconds(500)) {
            udp.SetSend(cmd);
        }
        else {
            cmd.forwardSpeed = 0.0f;
            cmd.sideSpeed = 0.0f;
            cmd.rotateSpeed = 0.0f;
            cmd.roll  = 0;
            cmd.pitch = 0;
            cmd.yaw = 0;
            cmd.mode = 1;
            udp.SetSend(cmd);
        }
    }
    else if (auto_moving_state == 1) {
        if ((fabs(z - highstate.forwardPosition) > 0.01 || fabs(x + highstate.sidePosition) > 0.01)) {
            cmd.forwardSpeed = 0.1f*(z - highstate.forwardPosition)/fabs(z - highstate.forwardPosition);
            cmd.sideSpeed = 0.5f*(x + highstate.sidePosition)/fabs(x + highstate.sidePosition);
            cmd.rotateSpeed = 0;
            cmd.mode = 2;
            udp.SetSend(cmd);
        }
        else {
            // cmd.forwardSpeed = 0.0f;
            // cmd.sideSpeed = 0.0f;
            // cmd.rotateSpeed = 0.0f;
            // cmd.mode = 1;
            printf("heello\n");
            auto_moving_state = 0;
        }
    }
    mutex.unlock();
}

void Custom::HighStateRecv() {
    while (1) {
        udp.GetRecv(highstate);
        // printf("forwardSpeed %lf\n", highstate.forwardSpeed);

        // printf("forwardPosition %lf sidePosition %lf\n", highstate.forwardPosition + init_fp, highstate.sidePosition + init_sp);
        forwardPosition = highstate.forwardPosition;
        sidePosition = highstate.sidePosition;
    }
    return;
}

int main(void)
{
    std::cout << "Control level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    Custom custom;

    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));


    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    // std::thread th_HighStateRecv(&Custom::HighStateRecv, &custom);
    std::thread th_momoUDPRecv(&Custom::momoUDPRecv, &custom);
    // std::thread th_user_input(&Custom::user_input, &custom);

    while(1){
        sleep(10);
    };

    return 0;
}
