#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

void StartDefaultTask_run(void *arg);
void bnoTask_run(void *arg);
void motionTask_run(void *arg);
void controlTask_run(void *arg);
void algorithmTask_run(void *arg);
void HMIConfigTask_run(void *arg);
void loggerTask_run(void* arg);
void app_main(); // This is your C++ entry function

#ifdef __cplusplus
}
#endif

#endif