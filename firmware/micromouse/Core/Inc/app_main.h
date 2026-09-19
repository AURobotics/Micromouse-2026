#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_LEFT_TIM  htim2
#define ENCODER_RIGHT_TIM htim3

#define ENCODER_CPR 10 //// counts per revolution

#define WHEEL_DIAMETER 1             //// in meters
#define WHEEL_BASE 1                 //// in meters
#define ENCODER_TASK_DT_S 1          //// in seconds
#define EXPECTED_SIDE_DIST_TO_WALL 9 // TODO:(half cell width - half width of robot)

#define MOTOR_LEFT_TIM htim1          
#define MOTOR_LEFT_CHANNEL 1        // TODO
#define MOTOR_RIGHT_TIM htim1        // TODO
#define MOTOR_RIGHT_CHANNEL 1       // TODO
#define MOTOR_DIR_LEFT_GPIO_Port 1  // TODO
#define MOTOR_DIR_LEFT_Pin 1        // TODO
#define MOTOR_DIR_RIGHT_GPIO_Port 1 // TODO
#define MOTOR_DIR_RIGHT_Pin 1       // TODO
#define MOTOR_PWM_MAX_CCR 1         // TODO

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t AlgorithimTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t MotionTaskHandle;
extern osThreadId_t BnoTaskHandle;
extern osThreadId_t HMITaskHandle;
extern osThreadId_t LoggerTaskHandle;
extern osMessageQueueId_t LoggingQueueHandle;

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