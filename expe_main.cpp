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

int end = 0;

void cameraSetting() {
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
    bool captureMode = false;
    FILE* fpw = NULL;
    FILE* fpl = NULL;
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        std::string str = std::string(buf);
        printf("%s", str.c_str());
        if (str.find("ERROR") != std::string::npos) {
            printf("Wake up\n");
            std::string cmd2 = "ptpcam --dev=" + cid + " --set-property=0xD80E --val=0x00";
            printf("%s\n", cmd2.c_str());
            fpw = popen(cmd2.c_str(), "r");
            char bufw[256];
            memset(bufw, 0, sizeof(bufw));
            while (fgets(bufw, sizeof(bufw), fpw) != NULL) {
                printf("%s", bufw);
            }
            printf("set live mode");
			std::string cmd3 = "ptpcam --dev=" + cid + " --set-property=0x5013 --val=0x8005";
            printf("%s\n", cmd3.c_str());
            fpl = popen(cmd3.c_str(), "r");
            char bufl[256];
            memset(bufl, 0, sizeof(bufl));
            while (fgets(bufl, sizeof(bufl), fpl) != NULL) {
                printf("%s", bufl);
            }
        }
        if (str.find("Capture") != std::string::npos) {
            captureMode = true;
        }
        memset(buf, 0, sizeof(buf));
    }
    if (!captureMode) {
        printf("set live mode");
        std::string cmd3 = "ptpcam --dev=" + cid + " --set-property=0x5013 --val=0x8005";
        printf("%s\n", cmd3.c_str());
        fpl = popen(cmd3.c_str(), "r");
        char bufl[256];
        memset(bufl, 0, sizeof(bufl));
        while (fgets(bufl, sizeof(bufl), fpl) != NULL) {
            printf("%s", bufl);
        }
    }

    if (fp != NULL) {
        pclose(fp);
    }
    if (fpw != NULL) {
        pclose(fpw);
    }
    if (fpl != NULL) {
        pclose(fpl);
    }

    // system("/home/tristar/MyWork-NX4_6/AutoRun_gst_loopback.py");
}

bool runTheta0 = false;

void gst_loopback() {
    std::string cam0 = "/home/tristar/MyWork-NX4_6/THETA_Cameras/camera0/gst_loopback";
    bool runTheta0 = true;
    printf("Open THETA0");
	FILE* fp0 = popen(cam0.c_str(), "r");
    char buf[256];
    memset(buf, 0, sizeof(buf));
    while (runTheta0) {
        while (fgets(buf, sizeof(buf), fp0) != NULL) {
            printf("%s", buf);
        }
        sleep(5);
    }
    if (fp0 != NULL) {
        pclose(fp0);
    }
}

bool is_gst_recording = false;
uint32_t gst_record_start_time = 0;

time_t t;
struct tm tm;

void get_today() {
    t = time(NULL);
    localtime_r(&t, &tm);
}

void gst_record() {
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
    is_gst_recording = true;
    gst_record_start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    system(pipe_proc);
    is_gst_recording = false;
}

uint32_t a1_record_start_time = 0;
bool is_a1_recording = false;

void observe_gst_record() {
    while (1) {
        if (is_gst_recording) {
            uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if (time_since_epoch - gst_record_start_time > 260) {
                a1_record_start_time = time_since_epoch;
                is_a1_recording = true;
                printf("A1 Record Started\n");
                return;
            }
        }
    }
}

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
    char filename[256];
    sprintf(
        filename,
        "%04d-%02d%02d-%02d%02d%02d.csv",
        tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec
    );
    FILE* fp = fopen(filename, "w");
    fprintf(fp, "time,lift,roll,pitch,foot1,foot2,foot3,foot4\n");
    while (1) {
        int recv_size = recv(sockfd_recv, &a1Status, sizeof(struct A1Status), 0);
        if (is_a1_recording && is_gst_recording) {
            uint32_t time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            double time = time_since_epoch/1000.0 - a1_record_start_time/1000.0;
            fprintf(
                fp,
                "%lf,%lf,%lf,%lf,%d,%d,%d,%d\n",
                time, a1Status.lift, a1Status.roll, a1Status.pitch,
                a1Status.foot1, a1Status.foot2, a1Status.foot3, a1Status.foot4
            );
        }
        if (!is_gst_recording && is_a1_recording) {
            is_a1_recording = false;
            fclose(fp);
        }
    };
}


int main(int argc, char* argv[]) {
    get_today();
    std::thread th_observe_gst_record(observe_gst_record);
    std::thread th_udp_recv_from_x86(udp_recv_from_x86);
    cameraSetting();
    sleep(3);
    std::thread th_gst_loopback(gst_loopback);
    sleep(5);
    std::thread th_gst_record(gst_record);
    while (1) {
        sleep(10);
    }
    return 0;
}
