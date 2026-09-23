#include "main.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "app_main.h"
#include "pure_pursuit.h"
#include "feedforward_pi.h"
#include "iirFilter.h"
#include "PDcontroller.h"
#include "BNO055.h"
#include "eeprom.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
struct queue
{
  char items[300]; // MAX QUEUE SIZE 16*16=256
  short head;
  short tail;
  short size = 300;
  short counter;
};

////////////////////////////////////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////
// sorry na2altohom kolohom hena 3ashan some c/c++ sh!t kan nefsy a7otohom fy app_main.h bas mashakel w mesh adra
typedef uint32_t EncoderCount_t; // adjust 3la 16 bit or 32 bit based on the encoder timer
// tim2 and timer 5 -->32 bit
// tim3 and timer 4 -->16 bit

/* wrap reading/writing structs aw variables related le ba3d b taskENTER_CRITICAL() and taskEXIT_CRITICAL()
  bas keep them short and fast with no blocking functions inside
  3ashan mayektebsh half the data and then ye7sal interrupt fa yeb2a nos el data new w nos old*/

vec_3 euler; // TODO:IMPORTANT CHECK THE UNITS OF EULER
vec_3 gyro;

volatile uint32_t adc_dma_buffer[3];
uint16_t ir_sequence[9] = {
    // TODO:check this
    0, 1260, 0, // Pulse 1: Only Channel 2 is ON
    0, 0, 1260, // Pulse 2: Only Channel 3 is ON
    1260, 0, 0  // Pulse 3: Only Channel 1 is ON
};

double ir_readings[6] = {0};              // left_front, right_front, left,right,left_diag, right_diag
double ir_thresh[6] = {1, 1, 1, 1, 1, 1}; // TODO
// double ir_distance[6] = {0};
struct Pose position = {0, 0, 0};
double yawOffset;
double theoreticalHeading = 0;
imu bno(&hi2c2, 0x29); // TODO: check address with physical connection
bool menu = false;
/* TODO: Calibrate adc, check adc calibration modes...
 * useful links: https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples/#introducing-stm32-adc
 *
 * taskname_run 3ashan freertos owns the tasks fa we'll call these functions gwa freertos.c
 * mesh katbeen el tasks henak fy freertos.c 3ashan el global variables kolaha teb2a hena
 * w el tasks and stuff cpp
 */
#define MAX_H 18 // TODO:mesh heya 16x16???
#define MAX_W 18 // 18
#define QUEUE_MAX (MAX_H * MAX_W)
char curr_dir = 0; // 0--> North, 1 --> East, 2 --> South, 3 --> West
char curr_r = 16, curr_c = 1;

int current_run;
int previous_run;

bool maze[MAX_H][MAX_W][5] = {0}; // represents the maze, first 4 bits represent the walls N E S W, the last bit represents the visiting status
// leh mn3melsh byte/char maze[MAX_H][MAX_W] ?

short dis[MAX_H][MAX_W] = {
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16},
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13},
    {12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12},
    {11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    {9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8},
    {8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8},
    {9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    {11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12},
    {13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
queue r_q;
queue c_q;
// change r, c to move to: N, E, S, W
signed char r_mov[4] = {-1, 0, 1, 0};
signed char c_mov[4] = {0, 1, 0, -1};

bool calibrateIR = false;
bool calibrateBNO = false;
uint32_t lastIrCalTick = 0;
typedef enum
{
  ROBOT_RUNNING,
  ROBOT_STOPPED
} RobotState_t;
volatile RobotState_t robotState = ROBOT_RUNNING;
volatile bool toggleRequested = false;
uint32_t lastResetTick = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t millis(void)
{
  return __HAL_TIM_GET_COUNTER(&htim5);
}
inline float getLin()
{
  vec_3 lin = bno.linear_acceleration();
  return lin.vec[2]; // Z axis
}
double map(double value, double fromLow, double fromHigh, double toLow, double toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
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
double fixSpeed(double speed)
{
  int maxx = 100;
  speed = constrain(speed, -maxx, maxx);
  if (fabs(speed) < 2)
    return 0;
  if (speed > 0)
    return map(speed, 0, maxx, 45, maxx); // TODO:shoofy motors start moving at which speed
  if (speed < 0)
    return map(speed, -maxx, 0, -maxx, -45);
  else
    return 0;
}
void set_motor_speeds(int16_t left_duty, int16_t right_duty)
{
  left_duty *= 9.99;
  right_duty *= 9.99;

  if (left_duty > 999)
    left_duty = 999;
  if (left_duty < -999)
    left_duty = -999;
  if (right_duty > 999)
    right_duty = 999;
  if (right_duty < -999)
    right_duty = -999;

  // TODO: define channels
  if (right_duty >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_FORWARD_CHANNEL, right_duty);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_BACKWARD_CHANNEL, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_FORWARD_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_BACKWARD_CHANNEL, -right_duty);
  }

  if (left_duty >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_FORWARD_CHANNEL, left_duty);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_BACKWARD_CHANNEL, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_FORWARD_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_BACKWARD_CHANNEL, -left_duty);
  }
}
inline double calculateDistance(double x, double y)
{
  return sqrt(pow(x - position.x, 2) + pow(y - position.y, 2));
}
bool frontEmergency()
{
  if (ir_readings[0] > 1250) // TODO
    return 1;
  return 0;
}

bool wallFront()
{
  for (int i = 0; i < 10; i++)
  {
    if (ir_readings[0] > ir_thresh[0] or ir_readings[1] > ir_thresh[1])
      return 1;
  }
  return 0;
}
bool wallRight()
{
  for (int i = 0; i < 10; i++)
  {
    if (ir_readings[3] > ir_thresh[3])
      return 1;
  }
  return 0;
}
bool wallLeft()
{
  for (int i = 0; i < 10; i++)
  {
    if (ir_readings[2] > ir_thresh[2])
      return 1;
  }
  return 0;
}
void turn(double angle)
{
  float currentAngle = position.theta;

  double desiredAngle = currentAngle + angle;

  double error = angleDiff(currentAngle, desiredAngle);
  bool direction = (error > 0 ? true : false); // true -> turn right | false -> turn left
  double errorPrev = error;
  double totalerror = 0;
  uint32_t lastPrint = millis();
  uint32_t lastLoopTime = millis();
  double minSpeed = 15; //------------------------------------------------------------------------TODO:need to tune this

  double kp = 1.2; // TODO:Kp and Kd will be set with testing
  double ki = 0.05;
  double kd = -0.09;

  const double integralMax = 30.0;
  double speed = 100;

  int counter = 0;

  while (fabs(error) > 1 || fabs(gyro.x()) > 0.5)
  {
    vTaskDelay(1);
    currentAngle = position.theta;

    error = angleDiff(currentAngle, desiredAngle);

    uint32_t now = millis();
    double dt = (now - lastLoopTime) / 1000.0;
    lastLoopTime = now;

    double pTerm = kp * error;
    double dTerm = kd * error / dt;

    // anti-windUp
    double iTermTentative = ki * error * dt;
    bool saturating = (pTerm + iTermTentative + dTerm > 100) || (pTerm + iTermTentative + dTerm < -100);
    if (!saturating)
    {
      totalerror += error * dt;
    }
    double iTerm = constrain(ki * totalerror, -integralMax, integralMax);

    speed = pTerm + iTerm + dTerm;
    speed = fixSpeed(speed);

    // if (fabs(speed) > 1 && fabs(speed) < minSpeed) {
    //   speed = (speed > 0 ? minSpeed : -minSpeed);
    // }

    direction = (speed > 0 ? true : false);
    set_motor_speeds(speed, -speed);

    errorPrev = error;
    totalerror += error * dt;

    if (fabs(gyro.x()) < 0.1)
      counter++;
    if (counter >= 40)
      break;

    if (millis() - lastPrint >= 100)
    {
      printf("turning %f, yaw=%f, error=%f, speed=%f, rate=%f,yaw offset=%f\n",
             desiredAngle, position.theta, error, speed, gyro.x(), yawOffset);

      lastPrint = millis();
    }
  }
  printf("done turning\n");

  set_motor_speeds(0, 0);
  theoreticalHeading = currentAngle;
  theoreticalHeading -= (theoreticalHeading > 360) ? 360 : 0;
  theoreticalHeading += (theoreticalHeading < 0) ? 360 : 0;
}
bool moveF(double tiles = 16)            // if you want to move tile by tile use moveF(1), if you want continuous use moveF();
{                                        // just need to add to make it stop using the irs
  double desiredDistance = tiles * 19.7; // el tile el mafrood 18cm, bas we found it would move slightly less than what we wanted, fa we increased it

  double startX = position.x, startY = position.y;
  double startYaw = theoreticalHeading;

  uint32_t startTime = millis();

  double errorL = desiredDistance - calculateDistance(startX, startY);
  double errorLPrev = errorL;

  bool direction = (errorL >= 0 ? true : false); // true -> forward, false -> backward

  // to keep moving staight

  double errorA = angleDiff(position.theta, startYaw);
  double errorAPrev = errorA;

  // double errorTicks = 0;
  double errorTicksPrev = 0;

  uint32_t t = millis();

  double Kpl = 3; // KD AND KP are changed with testing
  double Kdl = -1;

  double Kpa = -2.95; // changed
  double Kda = 1.2;   // decreased

  double KpTicks = 0.0;
  double KdTicks = 0.0;
  double kiTicks = 0.0;

  double speedl;
  double speeda;
  double speedTicks = 0;
  double speed;

  char timeout_ctr = 0;

  while ((fabs(errorL) > 0.2) && timeout_ctr < 50) // this 1 might change
  {
    vTaskDelay(1);
    errorL = desiredDistance - calculateDistance(startX, startY);
    errorA = angleDiff(position.theta, startYaw);

    speedl = Kpl * errorL + Kdl * (errorL - errorLPrev) / (millis() - t);
    speeda = Kpa * errorA + Kda * gyro.x();

    direction = (speedl >= 0 ? true : false);

    set_motor_speeds(fixSpeed(speedl - speeda), -fixSpeed(speedl - speeda));

    errorLPrev = errorL;
    errorAPrev = errorA;

    t = millis();
    if (fabs(getLin()) < 0.1)
    {
      timeout_ctr++;
    }
    if (frontEmergency())
      break;
  }

  printf("Done moveF\n");
  set_motor_speeds(0, 0);
  if (timeout_ctr >= 50)
    return 0;
  if (errorL > 10)
    return 0;
  return 1;
}
// ADC DMA Callback function
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

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)MotionTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
// printf->SWO
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
  // Enable trace subsystem
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // TPIU/ITM config — assumes core clock known, SWO baud rate e.g. 2000000
  *((volatile unsigned int *)0xE0040010) = HAL_RCC_GetHCLKFreq() / 2000000 - 1; // TPIU prescaler for SWO baud

  *((volatile unsigned int *)0xE00400F0) = 2; // Selected PIN Protocol Register: 2 = NRZ

  // Enable ITM, port 0
  ITM->LAR = 0xC5ACCE55; // Unlock
  ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk;
  ITM->TER = 1; // Enable stimulus port 0
}
// queue implementation
void initialise(queue &q, short size)
{
  q.head = 0;
  q.tail = 0;
  q.size = size;
  q.counter = 0;
}
bool isfull(queue &q)
{
  if (q.counter == q.size)
    return (1);
  else
    return (0);
}
bool isempty(queue &q)
{
  if (q.counter == 0)
    return (1);
  else
    return (0);
}

void enqueue(queue &q, char value)
{
  if (!isfull(q))
  {
    q.items[q.tail] = value;
    q.tail = (q.tail + 1) % q.size;
    q.counter++;
  }
}
char dequeue(queue &q)
{
  if (!isempty(q))
  {
    char result;
    result = q.items[q.head];
    q.head = (q.head + 1) % q.size;
    q.counter--;
    return result;
  }
  else
    return 0; // not sure law dah momken yebawaz logic el code bas it throws an error without it (reaches end of non-void function)
}

bool isValid(char r, char c)
{
  return ((r >= 0) && (r < MAX_H)) && ((c >= 0) && (c < MAX_W));
}

bool isAccessible(char r, char c, int dir)
{
  return !(maze[r][c][dir] || maze[r + r_mov[dir]][c + c_mov[dir]][(dir + 2) % 4]);
}

void flood(bool goal = 1)
{ // make goal = 0 to change the goal to the start
  printf("starting flood\n");
  for (char i = 0; i < MAX_W; i++)
  { // initialize all cells with -1
    for (char j = 0; j < MAX_H; j++)
    {
      dis[i][j] = -1;
    }
  }

  if (goal)
  {
    for (char x = MAX_W / 2 - 1; x < MAX_W / 2 + 1; x++)
    { // change middle cells with 0
      for (char w = MAX_H / 2 - 1; w < MAX_H / 2 + 1; w++)
      {
        dis[x][w] = 0;
        enqueue(r_q, x);
        enqueue(c_q, w);
      }
    }
  }
  else
  {
    dis[16][1] = 0;
    enqueue(r_q, 16);
    enqueue(c_q, 1);
  }

  while (!isempty(c_q) && !isempty(r_q))
  {
    char r = dequeue(r_q);
    char col = dequeue(c_q);
    printf("flooding from %d %d", (int)r, (int)col);
    for (int i = 0; i < 4; i++)
    {
      printf("%d\n ", maze[r][col][i]);
    }

    for (int i = 0; i < 4; i++)
    {
      printf("%d %d %d\n", isValid(r + r_mov[i], col + c_mov[i]), isAccessible(r, col, i), dis[r + r_mov[i]][col + c_mov[i]]);

      if (isValid(r + r_mov[i], col + c_mov[i]) && isAccessible(r, col, i) && dis[r + r_mov[i]][col + c_mov[i]] == -1)
      {
        printf("enqueuing %d %d\n", (int)(r + r_mov[i]), (int)(col + c_mov[i]));
        dis[r + r_mov[i]][col + c_mov[i]] = dis[r][col] + 1;
        enqueue(r_q, r + r_mov[i]);
        enqueue(c_q, col + c_mov[i]);
      }
    }
  }
}

bool moveTo(char r, char c)
{
  // get where I want to move relative to abolute direction (y3ny lw el robot bases north) ana lesa m2alef el term dah
  short dir;
  if (r < curr_r)
    dir = 0;
  if (r > curr_r)
    dir = 2;
  if (c < curr_c)
    dir = 3;
  if (c > curr_c)
    dir = 1;
  // compare the movement direction to the current directoin to know how should I turn
  if (dir - curr_dir == -1 || dir - curr_dir == 3) // turn left
  {
    printf("turning left\n");
    turn(-90); // turnLeft();
    vTaskDelay(pdMS_TO_TICKS(100));
    // moveF(1);       //moveForward();
    curr_dir += 3; // b3mel +3 msh -1 because el negative numbers don't work/work differently fel mod//
    curr_dir %= 4;
    printf("moving forward\n");
    if (!moveF(1))
      return 0;
  }

  else if (dir - curr_dir == 1 || dir - curr_dir == -3) // turn right
  {
    printf("turning right\n");
    turn(90); // turnRight();
    vTaskDelay(pdMS_TO_TICKS(100));
    curr_dir++;
    curr_dir %= 4;
    printf("moving forward\n");
    if (!moveF(1))
      return 0; // moveForward(); //this function returns 0 if it was unable to move,so we leave the func, 3shan man8ayarsh el curr c wel curr r
  }
  else if (dir == curr_dir) // move forward
  {
    printf("moving forward\n");
    if (!moveF(1))
      return 0; // moveForward();
  }
  else // turn 180
  {
    // turnRight();
    printf("turning 180\n");
    turn(180); // turnRight();
    vTaskDelay(pdMS_TO_TICKS(100));
    curr_dir += 2;
    curr_dir %= 4;
    printf("moving forward\n");
    if (!moveF(1))
      return 0; // moveForward();
  }

  curr_dir %= 4;
  curr_c = c;
  curr_r = r;
  return 1;
}
bool motionSuccessful = 0;
int flooded = 0;
void exploreToCenter()
{
  motionSuccessful = 1;
  flooded = 0;
  while (!(((curr_r == MAX_H / 2 - 1) || (curr_r == MAX_H / 2)) && ((curr_c == MAX_W / 2 - 1) || (curr_c == MAX_W / 2))))
  {
    vTaskDelay(1);
    // while (!(((curr_r == 12) || (curr_r == 13)) && ((curr_c == 4) || (curr_c == 5))) && !menu ) {
    // while(!(curr_r == 14 && curr_c == 3) && !menu){
    printf("Exploring to center %c %c %c\n", curr_c, curr_r, curr_dir);

    if (!maze[curr_r][curr_c][4] || !motionSuccessful)
    {
      // motionSuccessful = 1;
      bool walls[4];
      walls[0] = wallFront();
      walls[1] = wallRight();
      walls[3] = wallLeft();
      if (curr_r == 16 && curr_c == 1 && curr_dir == 0)
        walls[2] = 1;
      else
        walls[2] = 0;
      printf("done reading the walls\n");
      char d = curr_dir, w = 0;
      do
      {
        if (!(w == 2 && walls[w] == 0))
        {
          maze[curr_r][curr_c][d] = walls[w];
          maze[curr_r + r_mov[d]][curr_c + c_mov[d]][(d + 2) % 4] = walls[w]; // set the wall for the neighbouring cell too
        }
        d = (d + 1) % 4;
        w++;
      } while (d != curr_dir);
    }
    maze[curr_r][curr_c][4] = 1;
    char next_r = curr_r, next_c = curr_c;

    for (char i = 0; i < 4; i++)
    {
      if (isValid(curr_r + r_mov[i], curr_c + c_mov[i]) && isAccessible(curr_r, curr_c, i) && dis[curr_r + r_mov[i]][curr_c + c_mov[i]] < dis[next_r][next_c])
      {
        next_r = curr_r + r_mov[i];
        next_c = curr_c + c_mov[i];
      }
    }

    if ((next_r == curr_r) && (next_c == curr_c))
    {                                 // you re-flood when you can't find a place to go
      printf("next == curr flood\n"); // fa if you re-flood more than once, then the ir readings are most probablly wrong, fa sent mostionSuccessful to 0 to retake them
      flood();
      printf("done the next == curr flood\n");
      flooded++;
      if (flooded > 1)
      {
        motionSuccessful = 0;
        flooded = 0;
      }
    }
    else
    {
      flooded = 0;
      printf("Start moving\n");
      motionSuccessful = moveTo(next_r, next_c); // if failed, i want it to retake the ir readings
      printf("done moving\n");
    }
  }

  return;
}
void exploreToStart()
{
  motionSuccessful = 1;
  flooded = 0;
  while (!(curr_c == 1 && curr_r == 16) && !menu)
  {
    vTaskDelay(1);
    printf("Exploring to start %c %c %c\n", curr_c, curr_r, curr_dir);

    if (!maze[curr_r][curr_c][4] || !motionSuccessful)
    {
      bool walls[4];
      walls[0] = wallFront();
      walls[1] = wallRight();
      walls[3] = wallLeft();
      if (curr_r == 16 && curr_c == 1 && curr_dir == 0)
        walls[2] = 1;
      else
        walls[2] = 0;
      printf("done reading the walls");

      char d = curr_dir, w = 0;
      do
      {
        if (!(w == 2 && walls[w] == 0))
        {
          maze[curr_r][curr_c][d] = walls[w];
          maze[curr_r + r_mov[d]][curr_c + c_mov[d]][(d + 2) % 4] = walls[w];
        }
        d = (d + 1) % 4;
        w++;
      } while (d != curr_dir);
    }
    maze[curr_r][curr_c][4] = 1;

    char next_r = curr_r, next_c = curr_c;

    for (char i = 0; i < 4; i++)
    {
      if (isValid(curr_r + r_mov[i], curr_c + c_mov[i]) && isAccessible(curr_r, curr_c, i) && dis[curr_r + r_mov[i]][curr_c + c_mov[i]] < dis[next_r][next_c])
      {
        next_r = curr_r + r_mov[i];
        next_c = curr_c + c_mov[i];
      }
    }

    if ((next_r == curr_r) && (next_c == curr_c))
    {
      printf("next == curr flood");
      flood(0);
      printf("done the next == curr flood");
      flooded++;
      if (flooded > 1)
      {
        motionSuccessful = 0;
        flooded = 0;
      }
    }
    else
    {
      flooded = 0;
      printf("starting motion");
      motionSuccessful = moveTo(next_r, next_c);
      printf("done motion");
    }
  }
  return;
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_Pin == BTN_IRCAL_PIN)
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
  else if (GPIO_Pin == BTN_STOP_START_PIN)
  {
    uint32_t now = millis();
    if (now - lastResetTick > 200)
    {
      lastResetTick = now;
      if (robotState == ROBOT_RUNNING)
        set_motor_speeds(0, 0); 
      toggleRequested = true;
      vTaskNotifyGiveFromISR((TaskHandle_t)HMITaskHandle, &xHigherPriorityTaskWoken);
    }
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

  if (writeCalibration(&hi2c1, EEPROM_ADDR_BNO_OFFSETS, p.data, 22))
  {
    printf("BNO calibration saved to EEPROM :)\n");
    uint8_t magic_value = EEPROM_MAGIC;
    writeCalibration(&hi2c1, EEPROM_ADDR_BNO_VALID, &magic_value, 1);
  }
  else
    printf("saving to eeprom failed :(");
  return true;
}
bool loadBnoCalibration(imu &bno)
{
  uint8_t magic;
  readCalibration(&hi2c1, EEPROM_ADDR_BNO_VALID, &magic,1);
  if (magic == EEPROM_MAGIC)
  {
    CalibProfile_t p;
    readCalibration(&hi2c1, EEPROM_ADDR_BNO_OFFSETS, p.data, 22);
    bno.setOffsets(p);
    for (int i = 0; i < 22; i++)
      printf("%d ", p.data[i]);
    printf("\n");
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
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

void bnoTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  bno.init();
  double prevRawYaw = 0;
  double yawJumpThresh; // TODO
  if (loadBnoCalibration(bno))
    printf("BNO offsets loaded :)\n");
  else
    printf("no BNO offsets to load\n");

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
      vec_3 u = bno.gyro();
      double rawYaw = v.x();

      if (fabs(angleDiff(prevRawYaw, rawYaw)) > yawJumpThresh)
      {
        yawOffset += angleDiff(rawYaw, prevRawYaw);
        printf("-------------------------------------BNO jump detected, discrepancy=%f, new offset=%f", angleDiff(prevRawYaw, rawYaw), yawOffset);
      }
      prevRawYaw = rawYaw;
      euler = v;
      euler.vec[0] += yawOffset;
      gyro = u;
    }
  }
}

void motionTask_run(void *arg)
{
  // TODO: this will only update ir_readings and the encoder position
  //  so we wont need the getPosition() function from pharos
  //  momken yeb2a feeh wa2t negarab iir?
  TickType_t last = xTaskGetTickCount();

  // iir filter
  // ButterworthIIR ir_iir[6];
  // for (int i = 0; i < 6; i++)
  //   ir_iir[i].init(1, 200.0f); // TODO:tune
  // write position
  const float m_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick
  // ButterworthIIR velL;
  // ButterworthIIR velR;
  double dt = 0.005;
  // velL.init(1, 200.0f); // TODO: cutoff freq, sample rate 5ms
  // velR.init(1, 200.0f);
  // velL.reset();
  // velR.reset();

  EncoderCount_t countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
  EncoderCount_t countR = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
  EncoderCount_t lastCountL = countL;
  EncoderCount_t lastCountR = countR;

  loadIRCalFromEEPROM(&hi2c1);
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ////////////////////////////IRS//////////////////////
    for (int i = 0; i < 6; i += 2)
    {
      ir_readings[i] = adc_dma_buffer[i / 2] & 0xFFFF;
      ir_readings[i + 1] = (adc_dma_buffer[i / 2] >> 16) & 0xFFFF;

      // iir filter
      // ir_readings[i] = ir_iir[i].filter(ir_readings[i]);
      // ir_readings[i + 1] = ir_iir[i + 1].filter(ir_readings[i + 1]);
    }

    ///////////////////////ENCODERS/////////////////////
    // overflow logic for 16bit timer tim3
    uint16_t raw_R = __HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
    int32_t deltaR = (int16_t)(raw_R - lastCountR);
    countR += deltaR;

    countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    int32_t deltaL = (int32_t)(EncoderCount_t)(countL - lastCountL);

    lastCountL = countL;
    lastCountR = raw_R;

    // TODO: et2akedy men dool
    float distance_center = ((deltaL * m_per_tick) + (deltaR * m_per_tick)) / 2.0f;

    // update global
    // 2 critical blocks 3ashan mesh taba3 ba3d w law 3ayez ye3mel interrupt mabenhom no problem
    position.theta = euler.x();
    position.x += distance_center * cos(position.theta * M_PI / 180.0f);
    position.y += distance_center * sin(position.theta * M_PI / 180.0f);
  }
}

void controlTask_run(void *arg)
{
  // TODO: add algorithm and control stuff from pharos here. maybe also driving the motors? badal ma yeb2a global ya3ny w task tany yedrive them
  TickType_t last = xTaskGetTickCount();
  double dt = 0.005;

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));

    flood();
    ;
    // ir_readings[i + 1] = ir_iir[i + 1].filter(ir_readings[i + 1]);
    printf("done flood \n");
    previous_run = current_run;
    exploreToCenter();
    printf("done exploretocenter\n");
    current_run = dis[16][1];
    // if (current_run != 0 && current_run == previous_run) break;
    flood(0);
    printf("done flood to begin\n");
    exploreToStart();
    printf("done exploretostart\n");
  }
}

void algorithmTask_run(void *arg)
{
  // dont need this
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

void HMIConfigTask_run(void *arg)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (calibrateIR)
    {
      set_motor_speeds(0, 0);
      vTaskSuspend((TaskHandle_t)ControlTaskHandle);

      //waits for second press
      IRCalibration();
      if (saveIRCalToEEPROM(&hi2c1, (int16_t *)ir_thresh))
        printf("IRcalibration done and saved to eeprom\n");
      else
        printf("failed to save IRcalibration to eeprom\n");
      calibrateIR = false;

      vTaskResume((TaskHandle_t)ControlTaskHandle);
    }

    if (toggleRequested)
    {
      toggleRequested = false;

      if (robotState == ROBOT_RUNNING)
      {
        // first press: STOP+RESET  
        vTaskSuspend((TaskHandle_t)MotionTaskHandle);
        osThreadTerminate(ControlTaskHandle); 

        taskENTER_CRITICAL();
        position = {0, 0, 0};
        yawOffset = 0;
        curr_r = 16; curr_c = 1; curr_dir = 0;
        motionSuccessful = 1;
        flooded = 0;
        initialise(r_q, 300);
        initialise(c_q, 300);
        taskEXIT_CRITICAL();

        robotState = ROBOT_STOPPED;
        printf("Robot stopped and reset\n");
      }
      else
      {
        // second press: START 
        ControlTaskHandle = osThreadNew(controlTask, NULL, &ControlTask_attributes);
        vTaskResume((TaskHandle_t)MotionTaskHandle);
        robotState = ROBOT_RUNNING;
        printf("Robot started\n");
      }
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

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 3);

  // starts IR pwm
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

  // starts OC channel
  HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_4);

  // starts pwm write sequence
  HAL_TIM_DMABurst_WriteStart(&htim8, TIM_DMABASE_CCR1, TIM_DMA_CC4,
                              (uint32_t *)ir_sequence,
                              TIM_DMABURSTLENGTH_3TRANSFERS);

  // starts the master 1000 Hz timer
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim4);

  /* Init scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
  /* Start scheduler */
  osKernelStart();
  while (1)
  {
  }
}