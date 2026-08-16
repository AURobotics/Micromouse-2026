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
#include "PDcontroller.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_LEFT_TIM  htim2/////TODO:nned to define timer
#define ENCODER_RIGHT_TIM htim3/////7ateet ay etneen 3ashan ye compile STILL NEED TO DEFINE TIMER

typedef uint16_t EncoderCount_t; // adjust 3la 16 bit or 32 bit based on the encoder timer
// tim2 and timer 5 -->32 bit
// tim3 and timer 4 -->16 bit

#define ENCODER_CPR 10 //// counts per revolution

#define WHEEL_DIAMETER 1             //// in meters
#define WHEEL_BASE 1                 //// in meters
#define ENCODER_TASK_DT_S 1          //// in seconds
#define EXPECTED_SIDE_DIST_TO_WALL 9 // TODO:(half cell width - half width of robot)

#define MOTOR_LEFT_TIM 1            // TODO
#define MOTOR_LEFT_CHANNEL 1        // TODO
#define MOTOR_RIGHT_TIM 1           // TODO
#define MOTOR_RIGHT_CHANNEL 1       // TODO
#define MOTOR_DIR_LEFT_GPIO_Port 1  // TODO
#define MOTOR_DIR_LEFT_Pin 1        // TODO
#define MOTOR_DIR_RIGHT_GPIO_Port 1 // TODO
#define MOTOR_DIR_RIGHT_Pin 1       // TODO
#define MOTOR_PWM_MAX_CCR 1         // TODO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// use this to know the type of motion 3ashan ne center the robot only in STRAIGHT segments algorithm task controls it
enum MotionType
{
  STRAIGHT,
  STOP,
  TURN,
  DIAGONAL // maybe???
};

typedef struct {
  enum MotionType type;
} MotionCommand_t;

/*
  not sure abt this?? algorithm needs to know if the robot finished turning to read new walls and decide the new tile to move to
  fa 3ashan keda nestakhdem motion status
*/
typedef struct
{
  bool status; // 0 = done, 1 = running
} MotionStatus_t;

struct velocity
{
  double v;     // m/s
  double omega; // rad/s
  double vL;    // m/s    left wheel
  double vR;    // m/s     right wheel
};

struct vec_3
{
  float vec[3];
  float &x() { return vec[0]; }
  float &y() { return vec[1]; }
  float &z() { return vec[2]; }
  const float &x() const { return vec[0]; }
  const float &y() const { return vec[1]; }
  const float &z() const { return vec[2]; }
};

/* wrap reading/writing structs aw variables related le ba3d b taskENTER_CRITICAL() and taskEXIT_CRITICAL()
  bas keep them short and fast with no blocking functions inside
  3ashan mayektebsh half the data and then ye7sal interrupt fa yeb2a nos el data new w nos old*/

bool walls[3] = {false}; // front left right
//TODO:IMPORTANT CHECK THE UNITS OF EULER 
vec_3 euler;
vec_3 gyro;
double ir_readings[6] = {0}; // left_front, right_front, left,right,left_diag, right_diag
double ir_distance[6] = {0};
struct Pose position = {0, 0, 0};
struct velocity robot_velocity = {0, 0, 0, 0};

QueueHandle_t motionCmdQueue;
QueueHandle_t motionStatusQueue; // queue 3ashan ye trigger algorithm when status changes
MotionType motionType = STOP;
double target_v;
// TODO: tune these // km_ff tau_ff kp ki
FFPIConfig left_config = {0.05f, 0.12f, 0, 0};
FFPIConfig right_config = {0.05f, 0.12f, 0, 0};
static VelocityController leftCtrl(left_config);
static VelocityController rightCtrl(right_config);
// lookahead, wheel_base, kp_omega, kd_omega
static PurePursuitPD purePursuit(0, 0, 0, 0);
struct wheelVelocity wheel_ref = {0, 0};//purepursuit writes this

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern "C" void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void imuTask(void *arg);
void motionTask(void *arg);
void irTask(void *arg);
void controllerTask(void *arg);
void algorithmTask(void *arg);
float wrapAngle(float angle);
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

  
  motionCmdQueue = xQueueCreate(8, sizeof(MotionCommand_t));
  motionStatusQueue = xQueueCreate(8, sizeof(MotionStatus_t));

  xTaskCreate(algorithmTask, "ALGO", 512, NULL, 1, NULL);
  xTaskCreate(motionTask, "MOTC", 512, NULL, 4, NULL);
  xTaskCreate(imuTask, "IMU", 256, NULL, 2, NULL);
  xTaskCreate(controllerTask, "CONTROL", 256, NULL, 3, NULL);
  xTaskCreate(irTask, "IR", 256, NULL, 2, NULL);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
// wont use mutexes 3ashan if one task writes and another reads mesh moshkela awy isa
// the problem is if 2 tasks write on the same variable
void imuTask(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    // read i2c
    // write euler, gyro
  }
}

void motionTask(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  // iir filter
  // write position
  const float mm_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick
  ButterworthIIR velL;
  ButterworthIIR velR;
  double dt = 0.005; // TODO: is it better to calculate dt every loop?
  velL.init(1,1 );     // cutoff freq, sample rate
  velR.init(1, 1);
  velL.reset();
  velR.reset();

  EncoderCount_t left_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
  EncoderCount_t right_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
  EncoderCount_t lastCountL = 0;
  EncoderCount_t lastCountR = 0;
  MotionType lastMotionTypeMotion = STOP;

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    // raw counts
    EncoderCount_t countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    EncoderCount_t countR = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);

    int32_t deltaL = (int32_t)(EncoderCount_t)(countL - lastCountL);
    int32_t deltaR = (int32_t)(EncoderCount_t)(countR - lastCountR);
    if (sizeof(EncoderCount_t) == sizeof(uint16_t))
    {
      deltaL = (int16_t)deltaL;
      deltaR = (int16_t)deltaR;
    }
    lastCountL = countL;
    lastCountR = countR;

    // --- counts -> distance (m) -> raw velocity (m/s) ---
    float rawVelL = (deltaL * mm_per_tick) / ENCODER_TASK_DT_S;
    float rawVelR = (deltaR * mm_per_tick) / ENCODER_TASK_DT_S;
    //////?????
    float velLfiltered = velL.filter(rawVelL);
    float velRfiltered = velR.filter(rawVelR);
    float v = (velLfiltered + velRfiltered) * 0.5f;       // m/s, forward speed
    float w = (velRfiltered - velLfiltered) / WHEEL_BASE; // rad/s, positive = turning left
    float dTheta = w * ENCODER_TASK_DT_S;
    // TODO: et2akedy men dool
    float distance_center = ((deltaL * mm_per_tick) + (deltaR * mm_per_tick)) / 2.0f;

    /*straight line: PD Controller + IR centering + longitudinal correction with diagonal IRs
      turns: Pure pursuit only, corrected using gyro. there is no IR correction in turns

      error generated from purepursuit is corrected by the IRs later
      the error should be negligible for one turn
    */

    if(motionType != lastMotionTypeMotion){
      //do i reset these? ana mayla le both reset or not reset so idk
      leftCtrl.reset();
      rightCtrl.reset();
      lastMotionTypeMotion = motionType;
    }
    taskENTER_CRITICAL();
    wheelVelocity wheel_speed = wheel_ref;
    taskEXIT_CRITICAL();

    double left_cmd = leftCtrl.compute(wheel_speed.left,velLfiltered,dt);
    double right_cmd = rightCtrl.compute(wheel_speed.right,velRfiltered,dt);
    
    // update global
    // 2 critical blocks 3ashan mesh taba3 ba3d w law 3ayez ye3mel interrupt mabenhom no problem
    taskENTER_CRITICAL();
    position.theta = euler.y(); // wont calculate angle from encoders
    position.x += distance_center * cos(position.theta);
    position.y += distance_center * sin(position.theta);
    taskEXIT_CRITICAL();

    taskENTER_CRITICAL();
    robot_velocity.omega = w;
    robot_velocity.v = v;
    robot_velocity.vL = velLfiltered;
    robot_velocity.vR = velRfiltered;
    taskEXIT_CRITICAL();
    
    // TODO:drive motors dont know pins and stuff yet
  }
}

void irTask(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    // read sensors
    // fix position beta3 encoders
    // write walls[3]
    // update readings[6]
  }
}

void controllerTask(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  double dt = 0.005; // TODO: is it better to calculate dt every loop?
  std::vector<Point> path;
  PDController headingHoldPD(0,0,-100,100);//TODO:tune kp,kd
  PDController lateralPD(0,0,-100,100); //TODO:tune kp,kd
  float target_heading; // read only the moment we lose wall reference
  float lateral_correction = 0.0f;
  bool have_wall_ref_last_tick = true;
  MotionType lastMotionTypeCtrl = STOP; 

  purePursuit.reset();

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));

    taskENTER_CRITICAL();
    Pose current_pose = position;
    double v_measured = robot_velocity.v;
    double omega_measured = robot_velocity.omega;
    double left_distance = ir_distance[2], right_distance = ir_distance[3];
    taskEXIT_CRITICAL();

    if(motionType != lastMotionTypeCtrl){
      headingHoldPD.reset();
      lateralPD.reset();
      lastMotionTypeCtrl = motionType;
    }
    if (motionType == STRAIGHT)
    {
      //TODO: wrap reading the walls in CRITICAL section
      double base_v = target_v;
      float lateral_error = 0.0f;
      if (walls[1] && walls[2]){ // 2 side walls
        lateral_error = left_distance - right_distance;
        have_wall_ref_last_tick = true;
        lateral_correction = lateralPD.compute(0.0f,lateral_error,dt);
      }
      else if (walls[1]) {// only left wall
        lateral_error = left_distance - EXPECTED_SIDE_DIST_TO_WALL;
        have_wall_ref_last_tick = true;
        lateral_correction = lateralPD.compute(0.0f,lateral_error,dt);
      }
      else if (walls[2]){ // only right wall
        lateral_error = EXPECTED_SIDE_DIST_TO_WALL - right_distance;
        have_wall_ref_last_tick = true;
        lateral_correction = lateralPD.compute(0.0f,lateral_error,dt);
      }

      else{ // no side walls then hold current heading //dont know law dah momken ye7sal aslan bas better safe
        if(have_wall_ref_last_tick){
          target_heading = euler.y();
          have_wall_ref_last_tick = false;
        }
        float heading_error = wrapAngle(target_heading - euler.y());
        lateral_correction = headingHoldPD.compute(0.0f,-heading_error,dt);
      }
      taskENTER_CRITICAL();
      wheel_ref.left = base_v - lateral_correction;
      wheel_ref.right = base_v + lateral_correction;
      taskEXIT_CRITICAL();
    }
    else // if (motiontype == TURN) //not TURN only ay haga tanya ba2a will fall back to pure pursuit and we'll have to trust it or smth else idk
    {    // TODO: need to make a case for STOP 
      wheelVelocity wheel_speed = purePursuit.computeControl(current_pose, v_measured, omega_measured, target_v, path, dt);
      taskENTER_CRITICAL();
      wheel_ref = wheel_speed;
      taskEXIT_CRITICAL();
    }
  }
}

void algorithmTask(void *arg)
{
  MotionStatus_t status;
  for (;;)
  {
    if (xQueueReceive(motionStatusQueue, &status, portMAX_DELAY) == pdTRUE)
    {
    }
  }
}

float wrapAngle(float angle){
  while(angle > 180) angle -= 360;
  while(angle < -180) angle += 360;
  return angle;
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
