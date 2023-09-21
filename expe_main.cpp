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

#include <string>
#include <sstream>

struct A1Status {
    float lift;
    float roll;
    float pitch;
    int16_t foot1;
    int16_t foot2;
    int16_t foot3;
    int16_t foot4;
};

void udp_recv_from_x86(void) {
    int sockfd_recv = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_recv;
    addr_recv.sin_family = AF_INET;
    addr_recv.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr_recv.sin_port = htons(4123);
    bind(sockfd_recv, (const struct sockaddr *)&addr_recv, sizeof(addr_recv));

    struct A1Status a1Status;
    printf("time,lift,roll,pitch,foot1,foot2,foot3,foot4\n");
    uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double init_time = time_since_epoch/1000.0;
    while (1) {
        int recv_size = recv(sockfd_recv, &a1Status, sizeof(struct A1Status), 0);
        uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        double time = time_since_epoch/1000.0 - init_time;
        // printf(
        //     "%lf,%lf,%lf,%lf,%d,%d,%d,%d\n",
        //     time, a1Status.lift, a1Status.roll, a1Status.pitch,
        //     a1Status.foot1, a1Status.foot2, a1Status.foot3, a1Status.foot4
        // );
    };
}


int end = 0;

void gst_loopback() {
    FILE* fp = popen("lsusb", "r");
    char buf[256];
    std::string devID;
    std::string camStatus;
    memset(buf, 0, sizeof(buf));
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        std::string str = std::string(buf);
        if (str.find("Ricoh") != std::string::npos) {
            printf("%s", str.c_str());
            devID = str.substr(15, 3);
            camStatus = str.substr(28, 4);
            printf("devID: %s\\n\n", devID.c_str());
            printf("camStatus: %s\\n\n", camStatus.c_str());
        }
        memset(buf, 0, sizeof(buf));
    }
    std::string cid = devID;
    printf("----------%s: status-------\n", cid.c_str());
    std::string cmd = "ptpcam --dev=" + cid + " --show-property=0x5013";
    printf("%s\n", cmd.c_str());
    fp = popen(cmd.c_str(), "r");
    memset(buf, 0, sizeof(buf));
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        std::string str = std::string(buf);
        if (str.find("Ricoh") != std::string::npos) {
            printf("%s", str.c_str());
            devID = str.substr(15, 3);
            camStatus = str.substr(28, 4);
            printf("devID: %s\\n\n", devID.c_str());
            printf("camStatus: %s\\n\n", camStatus.c_str());
        }
        memset(buf, 0, sizeof(buf));
    }
    // system("/home/tristar/MyWork-NX4_6/AutoRun_gst_loopback.py");
}

void gst_record() {
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    char pipe_proc[300];
    sprintf(
        pipe_proc,
        "gst-launch-1.0  "
        "v4l2src num-buffers=600 device=/dev/video0 ! "
        "video/x-raw ! "
        "nvvidconv ! "
        "nvv4l2h265enc ! "
        "h265parse ! "
        "qtmux ! "
        "filesink location=/home/tristar/A1_jetson_ojima/Video/%04d-%02d%02d-%02d%02d%02d.mp4 -e",
        tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec
    );
    printf("%s\n", pipe_proc);
    system(pipe_proc);
    end = 1;
}


int main(int argc, char* argv[]) {
    // std::thread th_udp_recv_from_x86(udp_recv_from_x86);
    std::thread th_gst_loopback(gst_loopback);
    // sleep(15);
    // std::thread th_gst_record(gst_record);
    while (1) {
        sleep(10);
    }
    return 0;
}
