/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "feedforward_pi.h"
#include "pure_pursuit.h"
#include "queue.h"
#include "semphr.h"

#include "iirFilter.h"
#include<math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 #define ENCODER_LEFT_TIM   /////nned to define timer
 #define ENCODER_RIGHT_TIM   /////

 typedef uint16_t encoder_count_t;    //adjust 3la 16 bit or 32 bit based on the encoder timer
 // tim2 and timer 5 -->32 bit 
 // tim3 and timer 4 -->16 bit
 
 #define ENCODER_CPR  //// counts per revolution 

 #define WHEEL_DIAMETER  //// in meters
 #define WHEEL_BASE  //// in meters
#define ENCODER_TASK_DT_S    //// in seconds


 #define MOTOR_LEFT_TIM            // TODO
#define MOTOR_LEFT_CHANNEL   // TODO
#define MOTOR_RIGHT_TIM          // TODO
#define MOTOR_RIGHT_CHANNEL     // TODO
#define MOTOR_DIR_LEFT_GPIO_Port     // TODO
#define MOTOR_DIR_LEFT_Pin         // TODO
#define MOTOR_DIR_RIGHT_GPIO_Port  // TODO
#define MOTOR_DIR_RIGHT_Pin        // TODO
#define MOTOR_PWM_MAX_CCR   // TODO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum MotionType{
  FORWARD,
  STOP,
  TURN
};

typedef struct {
  enum MotionType type;
} MotionCommand_t;

typedef struct {
  bool status; // 0 = done, 1 = running
} MotionStatus_t;

struct velocity{
  double v; // m/s
  double omega; // rad/s
  double vL; // m/s    left wheel
  double vR; // m/s     right wheel
};

struct vec_3 {
  float vec[3];
  float& x() { return vec[0]; }
  float& y() { return vec[1]; }
  float& z() { return vec[2]; }
  const float& x() const { return vec[0]; }
  const float& y() const { return vec[1]; }
  const float& z() const { return vec[2]; }
};



bool walls[3] = {false};//front right left
vec_3 euler;
vec_3 gyro;
double ir_readings[6] = {0};
struct Pose position = {0,0,0};
SemaphoreHandle_t dataMutex;
QueueHandle_t motionCmdQueue;
QueueHandle_t motionStatusQueue;

//TODO: tune these // km_ff tau_ff kp ki
FFPIConfig left_config = {0.05f,  0.12f, 0, 0};
FFPIConfig right_config = {0.05f,  0.12f, 0, 0};
static VelocityController leftCtrl(left_config);
static VelocityController rightCtrl(right_config);
//lookahead, wheel_base, kp_omega, kd_omega
static PurePursuitPD purePursuit(0, 0, 0, 0);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void imuTask(void *arg);
void encoderTask(void *arg);
void irTask(void *arg);
void motionControlTask(void *arg);
void algorithmTask(void *arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */

  dataMutex = xSemaphoreCreateMutex();
  motionCmdQueue    = xQueueCreate(8, sizeof(MotionCommand_t));
  motionStatusQueue = xQueueCreate(8, sizeof(MotionStatus_t));

  xTaskCreate(algorithmTask,"ALGO", 512, NULL, 1, NULL);
  xTaskCreate(motionControlTask,"MOTC", 512, NULL, 4, NULL);
  xTaskCreate(imuTask,"IMU",256, NULL, 2, NULL);
  xTaskCreate(encoderTask, "ENC",256, NULL, 3, NULL);
  xTaskCreate(irTask, "IR",256, NULL,2, NULL);
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ########### IMPORTANT: dont forget fy kol task betekteb fy global variable semaphoretake/give ba3d kol read/write##################
void imuTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        // read i2c
        // write euler, gyro
    }
}

void encoderTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
  
        // iir filter
        // write position
      const float mm_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick
      ButterworthIIR velL;
      ButterworthIIR velR;  
      velL.init(, ); // cutoff freq, sample rate
      velR.init(, ); 
      velL.reset();
      velR.reset();

      EncoderCount_t left_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
      EncoderCount_t right_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);

       for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
         // raw counts
        EncoderCount_t countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
        EncoderCount_t countR = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);

        int32_t deltaL = (int32_t)(EncoderCount_t)(countL - lastCountL);
        int32_t deltaR = (int32_t)(EncoderCount_t)(countR - lastCountR);


        
         if (sizeof(EncoderCount_t) == sizeof(uint16_t)) {
            deltaL = (int16_t)deltaL;
            deltaR = (int16_t)deltaR;
        }
        lastCountL = countL;
        lastCountR = countR;

        // --- counts -> distance (m) -> raw velocity (m/s) ---
        float rawVelL = (deltaL * mm_per_tick) / ENCODER_TASK_DT_S;
        float rawVelR = (deltaR * mm_per_tick) / ENCODER_TASK_DT_S;
         //////?????
        float velL = velL.filter(rawVelL);
        float velR = velR.filter(rawVelR);
        float v = (velL + velR) * 0.5f;          // m/s, forward speed
        float w = (velR - velL) / WHEEL_BASE;  // rad/s, positive = turning left
        float dTheta = w * ENCODER_TASK_DT_S;


      }
     }


void irTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        // read sensors
        // fix position beta3 encoders
        // write walls[3]
        // update readings[6]
    }
}

void motionControlTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
    double dt = 0.005; //TODO: is it better to calculate dt every loop?
    std::vector<Point> path;
    double target_v;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
        // TODO: decide how to actually do this do we use mutex wala is it safe
        Pose current_pose = position;
        double v_measured = 0.0f; 
        double omega_measured = 0.0f;
        double left_wheel_meas = 0.0f;
        double right_wheel_meas = 0.0f;

        struct wheelVelocity wheel_ref = purePursuit.computeControl(current_pose, v_measured, omega_measured,target_v, path, dt);

        
        double left_cmd  = leftCtrl.compute(wheel_ref.left,  left_wheel_meas,  dt);
        double right_cmd = rightCtrl.compute(wheel_ref.right, right_wheel_meas, dt);

        //drive motors dont know pins and all that yet
        
    }
}

void algorithmTask(void *arg) {
    MotionStatus_t status;
    for (;;) {
        if (xQueueReceive(motionStatusQueue, &status, portMAX_DELAY) == pdTRUE){
          
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
