#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "queue.h"
#include "app_main.h"
#include "pure_pursuit.h"
#include "feedforward_pi.h"
#include "iirFilter.h"
#include "PDcontroller.h"
#include "semphr.h"
#include "BNO055.h"
#include "eeprom.h"
#include <cmath>
#include <vector>
// TODO: wrap ir stuff in CRITICAL sections cuz they're shared across more than 1 task
/* TODO: Calibrate adc, check adc calibration modes...
 * useful links: https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples/#introducing-stm32-adc
 *
 * taskname_run 3ashan freertos owns the tasks fa we'll call these functions gwa freertos.c
 * mesh katbeen el tasks henak fy freertos.c 3ashan el global variables kolaha teb2a hena
 * w el tasks and stuff cpp
 */

bool pausedAndReset = true; // started it as true 3ashan yebda2 el motors mesh sha8aleen and then start on pressing the button
bool calibrateIR = false;
bool calibrateBNO = false;
uint32_t lastIrCalTick = 0;
typedef uint32_t EncoderCount_t; // adjust 3la 16 bit or 32 bit based on the encoder timer
// tim2 and timer 5 -->32 bit
// tim3 and timer 4 -->16 bit

// use this to know the type of motion 3ashan ne center the robot only in STRAIGHT segments algorithm task controls it
enum MotionType
{
  STRAIGHT,
  STOP,
  TURN,
  TESTING_WHEEL_SPEEDS,
};

typedef struct
{
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

/* wrap reading/writing structs aw variables related le ba3d b taskENTER_CRITICAL() and taskEXIT_CRITICAL()
  bas keep them short and fast with no blocking functions inside
  3ashan mayektebsh half the data and then ye7sal interrupt fa yeb2a nos el data new w nos old*/

volatile uint32_t adc_dma_buffer[3];
uint16_t ir_sequence[9] = {
    // TODO:check this
    0, 1260, 0, // Pulse 1: Only Channel 2 is ON
    0, 0, 1260, // Pulse 2: Only Channel 3 is ON
    1260, 0, 0  // Pulse 3: Only Channel 1 is ON
};
bool walls[3] = {false}; // front left right
vec_3 euler;             // TODO:IMPORTANT CHECK THE UNITS OF EULER
vec_3 gyro;
uint16_t ir_readings[6] = {0}; // left_front, right_front, left,right,left_diag, right_diag
// double ir_distance[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // l7d ma el ir task ytktb  // in meters
uint16_t ir_thresh[6];
// uint32_t calReadings[4][6];
// uint16_t calDistances[6] = {2,4,6,8,10,14};
struct Pose position = {0, 0, 0};
struct velocity robot_velocity = {0, 0, 0, 0};

QueueHandle_t motionCmdQueue;
QueueHandle_t motionStatusQueue; // queue 3ashan ye trigger algorithm when status changes
MotionType motionType = STOP;
double target_v = 0.0; // m/s, set by algorithm task, read by control task
// TODO: tune these // km_ff tau_ff kp ki
FFPIConfig left_config = {0.05f, 0.12f, 0, 0};

FFPIConfig right_config = {0.05f, 0.12f, 0, 0};
static VelocityController leftCtrl(left_config);
static VelocityController rightCtrl(right_config);
// lookahead, wheel_base, kp_omega, kd_omega
static PurePursuitPD purePursuit(0, 0, 0, 0);
struct wheelVelocity wheel_ref = {0, 0}; // purepursuit writes this
std::vector<Point> current_path;         // pure pursuit reads this & algorithm writes this
SemaphoreHandle_t pathMutex;             // mutex to protect access to current_path
volatile uint32_t path_version = 0;      // incremented by algorithm task when it writes a new path, read by control task to know if it needs to copy the new path
volatile uint32_t cmd_id = 0;            // incremented by algorithm task when it writes a new command

/////////////////////////////////////////////////HELPER FUNCTIONS//////////////////////////////////////////////////
uint32_t millis(void)
{
  return __HAL_TIM_GET_COUNTER(&htim5);
}
int _write(int file, char *ptr, int len)
{
  for (int i = 0; i < len; i++)
  {
    ITM_SendChar((uint32_t)ptr[i]);
  }
  return len;
}
void SWO_Init(void) // in order to use ITM_SendChar
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // TPIU/ITM config — assumes core clock known, SWO baud rate e.g. 2000000
  *((volatile unsigned int *)0xE0040010) = HAL_RCC_GetHCLKFreq() / 2000000 - 1; // TPIU prescaler for SWO baud
  *((volatile unsigned int *)0xE00400F0) = 2;                                   // Selected PIN Protocol Register: 2 = NRZ
  // Enable ITM, port 0
  ITM->LAR = 0xC5ACCE55; // Unlock
  ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk;
  ITM->TER = 1; // Enable stimulus port 0
}
float wrapAngle(float angle)
{
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;
  return angle;
}
double angleDiff(double start, double goal)
{
  // goal  = (goal + 360) % 360.0;
  double diff = fmod(goal - start, 360.0);
  if (diff > 180)
    diff -= 360;
  if (diff < -180)
    diff += 360;
  return diff;
}
extern "C" void app_rtos_init(void)
{
  if (!pathMutex)
    pathMutex = xSemaphoreCreateMutex();
  if (!motionStatusQueue)
    motionStatusQueue = xQueueCreate(4, sizeof(MotionStatus_t));
  if (!motionCmdQueue)
    motionCmdQueue = xQueueCreate(4, sizeof(MotionCommand_t));
}

void motors(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *port,
            uint16_t pin, double speed)
{
  if (speed > 100.f)
    speed = 100.f;
  if (speed < -100.f)
    speed = -100.f;

  double pwm = (fabs(speed) / 100.f) * MOTOR_PWM_MAX_CCR; // convert speed percentage to pwm duty cycle
  if (pwm > MOTOR_PWM_MAX_CCR)
    pwm = MOTOR_PWM_MAX_CCR;

  if (speed >= 0)
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)pwm);
  }
  else
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)(-pwm));
  }
}
void motor_speeds(float left_speed, float right_speed)
{
  motors(&MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL, MOTOR_DIR_LEFT_GPIO_Port, MOTOR_DIR_LEFT_Pin, left_speed);
  motors(&MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL, MOTOR_DIR_RIGHT_GPIO_Port, MOTOR_DIR_RIGHT_Pin, right_speed);
}
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // stop
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SetCompare(&htim2, 0, 1260);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  }
}
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_Pin == BTN_STARTSTOP_Pin)
  {
    vTaskNotifyGiveFromISR((TaskHandle_t)defaultTaskHandle, &xHigherPriorityTaskWoken);
  }
  else if (GPIO_Pin == BTN_IRCAL_PIN)
  {
    uint32_t now = millis();
    if (now - lastIrCalTick > 200)
    { // 200ms debounce
      lastIrCalTick = now;
      calibrateIR = true;
      vTaskNotifyGiveFromISR((TaskHandle_t)HMITaskHandle, &xHigherPriorityTaskWoken);
    }
  }
  else if (GPIO_Pin == BTN_BNOCAL_PIN)
  {
    calibrateBNO = true;
    vTaskNotifyGiveFromISR((TaskHandle_t)BnoTaskHandle, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
bool calibrateBnoAndSave(imu &bno)
{
  Calibration_t s{};
  unsigned long start = millis();
  const unsigned long TIMEOUT_MS = 120000;

  printf("BNO Calibration...\n");
  while (true)
  {
    bno.calibration_status(s);
    printf("sys = %d", s.sys);
    printf("  gyro = %d", s.gyro);
    printf("  accel = %d", s.accel);
    printf("  mag = %d\n", s.mag);
    if (s.sys == 3 && s.gyro == 3 && s.accel == 3 /*&& s.mag == 3*/)
      break;
    if (millis() - start > TIMEOUT_MS)
    {
      printf("BNO Calibration timed out :(\n");
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }

  CalibProfile_t p;
  bno.getOffsets(p);
  uint8_t magic_value = EEPROM_MAGIC;
  writeCalibration(&hi2c1, EEPROM_ADDR_BNO_VALID, &magic_value, 1);
  if (writeCalibration(&hi2c1, EEPROM_ADDR_BNO_OFFSETS, p.data, 22))
    printf("BNO calibration saved to EEPROM :)\n");
  else
    printf("saving to eeprom failed :(");
  return true;
}
bool loadBnoCalibration(imu &bno)
{
  uint8_t magic_value;
  readCalibration(&hi2c1, 0, &magic_value, 1);
  if (magic_value == EEPROM_MAGIC)
  {
    CalibProfile_t p;
    readCalibration(&hi2c1, 5, p.data, 22);
    bno.setOffsets(p);
    return true;
  }
  return false;
}
void IRCalibration(/*uint8_t sensor*/)
{
  printf("IR calibration: position robot and then press button2\r\n");
  // for (int i = 0; i < 6; i++) {
  // printf("Move %d to %d cm from wall, then press IR-CAL button\r\n", sensor, calDistances[i]);

  // Block here until the EXTI ISR gives a notification for this button
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  long sum[6] = {0};
  for (int s = 0; s < 20; s++)
  {
    for (int i = 0; i < 6; i++)
      sum[i] += ir_readings[i];
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  // calReadings[sensor][i] = sum / 20;
  for (int i = 0; i < 6; i++)
    ir_thresh[i] = sum[i] / 20;
  // }

  // printf(" calReadings[%d] = ", sensor);
  // for (int i = 0; i < 6; i++) printf("%ld, ", calReadings[sensor][i]);
  printf("\r\ncalibration done\r\n");
  calibrateIR = false;
}
bool saveIRCalToEEPROM(I2C_HandleTypeDef *i2c, int16_t thresholds[6])
{
  // write data block first
  if (!writeCalibration(i2c, EEPROM_ADDR_IR_CAL, (uint8_t *)thresholds, 6 * sizeof(int16_t)))
  {
    printf("IR calibration EEPROM write FAILED\r\n");
    return false;
  }
  // write magic byte LAST, only if data write succeeded
  uint8_t magic = EEPROM_MAGIC;
  if (!writeCalibration(i2c, EEPROM_ADDR_IR_VALID, &magic, 1))
  {
    printf("IR calibration magic byte write FAILED\r\n");
    return false;
  }
  printf("IR thresholds saved to EEPROM: ");
  for (int i = 0; i < 6; i++)
    printf("%d ", thresholds[i]);
  printf("\r\n");

  return true;
}
bool loadIRCalFromEEPROM(I2C_HandleTypeDef *i2c)
{
  uint8_t magic = 0;
  if (!readCalibration(i2c, EEPROM_ADDR_IR_VALID, &magic, 1))
    return false;

  if (magic != EEPROM_MAGIC)
  {
    printf("No valid IR calibration in EEPROM\r\n");
    return false;
  }
  if (!readCalibration(i2c, EEPROM_ADDR_IR_CAL, (uint8_t *)ir_thresh, 6 * sizeof(int16_t)))
  {
    printf("IR calibration EEPROM read FAILED\r\n");
    return false;
  }
  printf("IR thresholds loaded from EEPROM: ");
  for (int i = 0; i < 6; i++)
    printf("%d ", ir_thresh[i]);
  printf("\r\n");
  return true;
}
////////////////////////////////////////////////TASKS////////////////////////////////////////////////////
void StartDefaultTask_run(void *arg)
{
  // im going to use this task for START/STOP button
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(30)); // debounce: wait, then confirm still pressed
    if (HAL_GPIO_ReadPin(BTN_STARTSTOP_GPIO_Port, BTN_STARTSTOP_Pin) != GPIO_PIN_RESET)
      continue; // bounce, ignore

    if (pausedAndReset)
    {
      pausedAndReset = false;
      // TODO:start the run: signal algorithm task, or set motionType or whatever
    }
    else if (!pausedAndReset)
    {
      pausedAndReset = true;
      // STOP motors
      taskENTER_CRITICAL();
      motionType = STOP;
      taskEXIT_CRITICAL();
      // TODO:reset position
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void bnoTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  imu bno(&hi2c2, 0x28); // TODO: check the address
  bno.init();
  double prevRawYaw = bno.euler().x();
  double yawOffset = 0;
  double yawJumpThresh=60; // TODO: tune
  if (loadBnoCalibration(bno))
    printf("loaded bnoCalibration successfully :)\n");
  else
    printf("no bnoCalibration data to load\n");

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    if (calibrateBNO)
    {
      calibrateBnoAndSave(bno);
      calibrateBNO = false;
    }
    else
    {
      vec_3 v = bno.euler();
      vec_3 g = bno.gyro();

      if (fabs(angleDiff(prevRawYaw, v.x())) > yawJumpThresh)
      {
        yawOffset += angleDiff(v.x(), prevRawYaw);
        printf("-------------------------------------BNO jump detected, discrepancy=%f, newOffset=%f", angleDiff(prevRawYaw, v.x()), yawOffset);
      }
      prevRawYaw = v.x();
      taskENTER_CRITICAL();
      euler = v;
      euler.x() += yawOffset;
      gyro = g;
      taskEXIT_CRITICAL();
    }
  }
}

void motionTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  // iir filter
  // write position
  const float m_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick

  double dt = ENCODER_TASK_DT_S;
  ButterworthIIR velL;
  ButterworthIIR velR;

  velL.init(1, 200.0f); // cutoff freq, sample rate 5ms
  velR.init(1, 200.0f);
  velL.reset();
  velR.reset();

  EncoderCount_t countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
  EncoderCount_t countR = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
  EncoderCount_t lastCountL = countL;
  EncoderCount_t lastCountR = countR;
  MotionType lastMotionTypeMotion = STOP;
  uint32_t last_cmd_id = 0; // b3rad el command id 3ashan ne know law command gdeda w nreset el controllers
  uint32_t ir_margin = 100; // TODO 3ashan benakhod el reading w hwa odam el 7eeta belzabt bas 3ayzeen nedeeh headroom

  if (loadIRCalFromEEPROM(&hi2c1))
    printf("loaded IR calibration\n");
  else
    printf("no IR calibration in EEPROM\n");

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    for (int i = 0; i < 6; i += 2)
    {
      ir_readings[i] = adc_dma_buffer[i / 2] & 0xFFFF;
      ir_readings[i + 1] = (adc_dma_buffer[i / 2] >> 16) & 0xFFFF;
    }
    walls[0] = (ir_readings[0] > ir_thresh[0] - ir_margin) || (ir_readings[1] > ir_thresh[1] - ir_margin);
    walls[1] = ir_readings[2] > ir_thresh[2] - ir_margin;
    walls[2] = ir_readings[3] > ir_thresh[3] - ir_margin;
    // raw counts
    // overflow logic for tim3 16bit
    uint16_t raw_R = __HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
    int32_t deltaR = (int16_t)(raw_R - lastCountR);
    countR += deltaR;

    countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    int32_t deltaL = (int32_t)(countL - lastCountL);

    lastCountL = countL;
    lastCountR = raw_R;

    // --- counts -> distance (m) -> raw velocity (m/s) ---
    float rawVelL = (deltaL * m_per_tick) / ENCODER_TASK_DT_S;
    float rawVelR = (deltaR * m_per_tick) / ENCODER_TASK_DT_S;
    //////?????
    float velLfiltered = velL.filter(rawVelL);
    float velRfiltered = velR.filter(rawVelR);
    float v = (velLfiltered + velRfiltered) * 0.5f;       // m/s, forward speed
    float w = (velRfiltered - velLfiltered) / WHEEL_BASE; // rad/s, positive = turning left
    // TODO: et2akedy men dool
    float distance_center = ((deltaL * m_per_tick) + (deltaR * m_per_tick)) * 0.5f;

    /*straight line: PD Controller + IR centering + longitudinal correction with diagonal IRs
      turns: Pure pursuit only, corrected using gyro. there is no IR correction in turns

      error generated from purepursuit is corrected by the IRs later
      the error should be negligible for one turn
    */
    taskENTER_CRITICAL();
    MotionType current_motion = motionType;
    uint32_t cmd = cmd_id;
    wheelVelocity wheel_speed = wheel_ref;
    float heading = euler.x(); // degree
    taskEXIT_CRITICAL();

    if (motionType != lastMotionTypeMotion || cmd != last_cmd_id)
    {
      // do i reset these? ana mayla le both reset or not reset so idk
      leftCtrl.reset();
      rightCtrl.reset();
      lastMotionTypeMotion = current_motion;
      last_cmd_id = cmd;
    }
    float left_cmd = 0.0f;
    float right_cmd = 0.0f;

    if (current_motion != STOP)
    {
      left_cmd = leftCtrl.compute(wheel_speed.left, velLfiltered, dt);
      right_cmd = rightCtrl.compute(wheel_speed.right, velRfiltered, dt);
    }
    motor_speeds(left_cmd, right_cmd);

    // update global
    // 2 critical blocks 3ashan mesh taba3 ba3d w law 3ayez ye3mel interrupt mabenhom no problem
    taskENTER_CRITICAL();
    position.theta = heading * (M_PI / 180.0); // wont calculate angle from encoders
    position.x += distance_center * cos(position.theta);
    position.y += distance_center * sin(position.theta);
    taskEXIT_CRITICAL();

    taskENTER_CRITICAL();
    robot_velocity.omega = w;
    robot_velocity.v = v;
    robot_velocity.vL = velLfiltered;
    robot_velocity.vR = velRfiltered;
    taskEXIT_CRITICAL();
  }
}

void controlTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  double dt = 0.005;
  // std::vector<Point> path;
  PDController headingHoldPD(0, 0, -100, 100); // TODO:tune kp,kd
  PDController lateralPD(0, 0, -100, 100);     // TODO:tune kp,kd
  float target_heading = 0.0f;                 // read only the moment we lose wall reference
  float lateral_correction = 0.0f;
  int last_mode = -1; // to detect change in motion type
  MotionType lastMotionTypeCtrl = STOP;
  uint32_t last_cmd_id = 0; // b3rad el command id 3ashan ne know law command gdeda w nreset el controllers
  bool status_sent = false; // 3shan ne send status only once when motion is done

  Pose straight_start_pose;
  float straight_target_dist = 0.18f;//cell in meters

  std::vector<Point> path_copy;
  uint32_t local_ver = 0;

  purePursuit.reset();

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));

    if (path_version != local_ver)
    {
      xSemaphoreTake(pathMutex, portMAX_DELAY);
      path_copy = current_path;
      local_ver = path_version;
      xSemaphoreGive(pathMutex);
    }
    taskENTER_CRITICAL();
    Pose current_pose = position;
    double v_measured = robot_velocity.v;
    double omega_measured = robot_velocity.omega;
    // double left_distance = ir_distance[2];
    // double right_distance = ir_distance[3];
    // double front_distance = (ir_distance[0] + ir_distance[1]) * 0.5;
    MotionType current_motion = motionType;
    double current_target_v = target_v;
    uint32_t cmd = cmd_id;
    // float heading = euler.x();  //heading_sign *euler.x(); //degree ccw positive
    bool wall_front = walls[0];
    bool wall_left = walls[1];
    bool wall_right = walls[2];
    taskEXIT_CRITICAL();

    if (motionType != lastMotionTypeCtrl || cmd != last_cmd_id)
    {
      headingHoldPD.reset();
      lateralPD.reset();
      purePursuit.reset();
      lastMotionTypeCtrl = current_motion;
      last_cmd_id = cmd;
      status_sent = false; // b nreeset el flag
      last_mode = -1;      // reset last mode to force re-evaluation of motion type

      straight_start_pose = current_pose; // 3ashan a3raf el distance el mashaha fy STRAIGHT segment
    }
    // emergency stop if front wall detected & 🛺 lsa mkml staright
    if (current_motion == STRAIGHT && (wall_front))
    { // TODO: i think en hena wall_front is not correct 3ashan momken yeb2a shayef front wall bas mashy sa7
      // lesa mesh hayekhbat ya3ny fa IMPORTANT

      taskENTER_CRITICAL();
      wheel_ref.left = 0.0;
      wheel_ref.right = 0.0;
      motionType = STOP;
      taskEXIT_CRITICAL();

      if (!status_sent)
      {
        MotionStatus_t status = {0};
        xQueueSend(motionStatusQueue, &status, 0);
        status_sent = true;
      }
      continue;
    }
    if (current_motion == STRAIGHT)
    {
      // TODO: wrap reading the walls in CRITICAL section
      double base_v = current_target_v;
      float lateral_error = 0.0f;
      if (walls[1] && walls[2])
      { // 2 side walls
        // ir_thresh[2]-ir_thresh[3] offset between 2 sensors readings
        // wont be entirely accurate law bo3ad 3an ba3d awy bas insha2allah accurate law el robot roughly fy nos el cell
        lateral_error = ir_readings[2] - (ir_thresh[2] - ir_thresh[3]) - ir_readings[3];
        last_mode = 0; // STRAIGHT
        lateral_correction = lateralPD.compute(0.0f, lateral_error, dt);
      }
      else if (walls[1])
      { // only left wall
        lateral_error = ir_readings[2] - ir_thresh[2];
        last_mode = 1; // LEFT_WALL
        lateral_correction = lateralPD.compute(0.0f, lateral_error, dt);
      }
      else if (walls[2])
      { // only right wall
        lateral_error = ir_thresh[3] - ir_readings[3];
        last_mode = 2; // RIGHT_WALL
        lateral_correction = lateralPD.compute(0.0f, lateral_error, dt);
      }

      else
      { // no side walls then hold current heading //dont know law dah momken ye7sal aslan bas better safe
        if (last_mode == -1)
        {
          taskENTER_CRITICAL();
          target_heading = euler.x();
          last_mode = 3; // NO_WALLS
          taskEXIT_CRITICAL();
        }
        taskENTER_CRITICAL();
        float heading_error = wrapAngle(target_heading - euler.x());
        taskEXIT_CRITICAL();
        lateral_correction = headingHoldPD.compute(0.0f, -heading_error, dt);
      }
      // TODO :
      taskENTER_CRITICAL();
      wheel_ref.left = base_v - lateral_correction;
      wheel_ref.right = base_v + lateral_correction;
      taskEXIT_CRITICAL();

      double dist_traveled = std::hypot(current_pose.x - straight_start_pose.x,
                                        current_pose.y - straight_start_pose.y);
      if (dist_traveled >= straight_target_dist && !status_sent && motionStatusQueue != NULL)
      {
        taskENTER_CRITICAL();
        wheel_ref.left = 0.0;
        wheel_ref.right = 0.0;
        motionType = STOP;
        taskEXIT_CRITICAL();

        MotionStatus_t status = {false};
        xQueueSend(motionStatusQueue, &status, 0);
        status_sent = true;
      }
    }
    else if (current_motion == TURN)
    {
      wheelVelocity wheel_speed = purePursuit.computeControl(current_pose, v_measured, omega_measured, current_target_v, path_copy, dt);
      taskENTER_CRITICAL();
      wheel_ref = wheel_speed;
      taskEXIT_CRITICAL();

      if (!path_copy.empty() && !status_sent && motionStatusQueue != NULL)
      {

        const Point &goal = path_copy.back();
        double dist_to_goal = std::hypot(goal.x - current_pose.x, goal.y - current_pose.y);

        if (dist_to_goal < 0.01)
        {                                  // threshold to consider the turn complete
          MotionStatus_t status = {false}; // done
          xQueueSend(motionStatusQueue, &status, 0);
          status_sent = true;
        }
      }
    }
    else if (current_motion == TESTING_WHEEL_SPEEDS)
    {
      taskENTER_CRITICAL();
      wheel_ref.left = current_target_v;
      wheel_ref.right = current_target_v;
      taskEXIT_CRITICAL();
    }
    else if (current_motion == STOP)
    {
      // TODO: need to make a case for STOP
      taskENTER_CRITICAL();
      wheel_ref.left = 0;
      wheel_ref.right = 0;
      taskEXIT_CRITICAL();
      ////kda mafrood el el algo ykteen el current_path f el pathMutex w b3den path ver++
      /// w yzwed el cmd_id f col 2mr gded 7ta lw nafs el motion type 3shan a3rf a3ml reset w el status_sent flag
    }
  }
}

void algorithmTask_run(void *arg)
{ // have to check pausedAndReset bool before pushing any new command
  MotionStatus_t status;
  for (;;)
  {
    if (xQueueReceive(motionStatusQueue, &status, portMAX_DELAY) == pdTRUE)
    {
    }
  }
}

void HMIConfigTask_run(void *arg)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (calibrateIR)
    {
      IRCalibration();
      if (saveIRCalToEEPROM(&hi2c1, ir_thresh))
        printf("IRcalibration done and saved to eeprom\n");
      else
        printf("failed to save IRcalibration to eeprom\n");
      calibrateIR = false;
    }
  }
}

void loggerTask_run(void *arg)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}
//////////////////////////////////////////////////END TASKS//////////////////////////////////////////////
void app_main()
{
  // Write your C++ application code here
  // This acts as your new int main()
  while (1)
  {
  }
}
