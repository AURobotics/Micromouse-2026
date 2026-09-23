#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "cmsis_os2.h" 
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_LEFT_TIM  htim2
#define ENCODER_RIGHT_TIM htim3

//TODO:
#define ENCODER_CPR 10 //// counts per revolution

#define WHEEL_DIAMETER 1             //// in meters
#define WHEEL_BASE 1                 //// in meters
#define ENCODER_TASK_DT_S 0.005          //// in seconds
#define EXPECTED_SIDE_DIST_TO_WALL 9 // TODO:(half cell width - half width of robot)

//TODO: check pins
#define MOTOR_LEFT_TIM htim1          
#define MOTOR_LEFT_FORWARD_CHANNEL TIM_CHANNEL_1       
#define MOTOR_LEFT_BACKWARD_CHANNEL TIM_CHANNEL_2
#define MOTOR_RIGHT_TIM htim1        
#define MOTOR_RIGHT_FORWARD_CHANNEL TIM_CHANNEL_3      
#define MOTOR_RIGHT_BACKWARD_CHANNEL TIM_CHANNEL_4

#define MOTOR_PWM_MAX_CCR 999         // TODO

#define BTN_IRCAL_GPIO_PORT GPIOB
#define BTN_IRCAL_PIN GPIO_PIN_13
#define BTN_BNOCAL_GPIO_PORT GPIOB
#define BTN_BNOCAL_PIN GPIO_PIN_14
#define BTN_STOP_START_PIN GPIO_PIN_12

#define EEPROM_MAGIC 0x42
#define EEPROM_ADDR_BNO_VALID     0    // 1 byte
#define EEPROM_ADDR_BNO_OFFSETS   5    // 22 bytes -> 5-26

#define EEPROM_ADDR_IR_VALID      1    // 1 byte
#define EEPROM_ADDR_IR_CAL        30   // 48 bytes -> 30-77 (left a gap after BNO)

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t AlgorithimTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t MotionTaskHandle;
extern osThreadId_t BnoTaskHandle;
extern osThreadId_t HMITaskHandle;
extern osThreadId_t LoggerTaskHandle;
extern osMessageQueueId_t LoggingQueueHandle;
extern const osThreadAttr_t ControlTask_attributes;
extern void controlTask(void *argument);

void StartDefaultTask_run(void *arg);
void bnoTask_run(void *arg);
void motionTask_run(void *arg);
void controlTask_run(void *arg);
void algorithmTask_run(void *arg);
void HMIConfigTask_run(void *arg);
void loggerTask_run(void* arg);
void app_main(); // This is your C++ entry function
uint32_t millis(void);


#ifdef __cplusplus
}
#endif

#endif