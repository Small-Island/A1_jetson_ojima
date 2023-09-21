#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "pthread.h"

int end = 0;

void* gst_loopback(void *p) {
    system("/home/tristar/MyWork-NX4_6/AutoRun_gst_loopback.py");
}

void* gst_record(void *p) {
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
        "filesink location=/home/tristar/MyWork-NX4_6/Video/%04d-%02d%02d-%02d%02d%02d.mp4 -e",
        tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec
    );
    printf("%s\n", pipe_proc);
    system(pipe_proc);
    end = 1;
}

int main (int argc, char *argv[]) {
    pthread_t th_gstlb;
    pthread_create(&th_gstlb, NULL, &gst_loopback, NULL);
    sleep(15);
    pthread_t th;
    pthread_create(&th, NULL, &gst_record, NULL);
    usleep(0.31*1000*1000);
    struct timeval tv;
    unsigned int sec;
    int nsec;
    double d_sec;
    
    struct timespec start_time, end_time;

    /* 処理開始前の時間を取得 */
    clock_gettime(CLOCK_REALTIME, &start_time);


    while (1) {
        /* 処理開始後の時間とクロックを取得 */
        clock_gettime(CLOCK_REALTIME, &end_time);

        /* 処理中の経過時間を計算 */
        sec = end_time.tv_sec - start_time.tv_sec;
        nsec = end_time.tv_nsec - start_time.tv_nsec;

        d_sec = (double)sec
            + (double)nsec / (1000 * 1000 * 1000);

        /* 計測時間の表示 */
        printf(
            "time:%f\n", d_sec
        );
        // gettimeofday(&tv, NULL);
        // printf("現在時刻: %ld秒 %ldマイクロ秒\n", tv.tv_sec, tv.tv_usec);
        usleep(1000*100);
        if (end) {
            break;
        }
    }
    return 0;
}