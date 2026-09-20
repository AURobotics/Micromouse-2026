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
#include <cmath>
#include <vector>

/* TODO: Calibrate adc, check adc calibration modes...
* useful links: https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples/#introducing-stm32-adc
* 
* taskname_run 3ashan freertos owns the tasks fa we'll call these functions gwa freertos.c 
* mesh katbeen el tasks henak fy freertos.c 3ashan el global variables kolaha teb2a hena 
* w el tasks and stuff cpp 
*/

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
#define MOTOR_RIGHT_TIM 1       // TODO
#define MOTOR_RIGHT_CHANNEL 1       // TODO
#define MOTOR_DIR_LEFT_GPIO_Port 1  // TODO
#define MOTOR_DIR_LEFT_Pin 1        // TODO
#define MOTOR_DIR_RIGHT_GPIO_Port 1 // TODO
#define MOTOR_DIR_RIGHT_Pin 1       // TODO
#define MOTOR_PWM_MAX_CCR 1         // TODO

// use this to know the type of motion 3ashan ne center the robot only in STRAIGHT segments algorithm task controls it
enum MotionType
{
  STRAIGHT,
  STOP,
  TURN,
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

// struct vec_3
// {
//   float vec[3];
//   float &x() { return vec[0]; }
//   float &y() { return vec[1]; }
//   float &z() { return vec[2]; }
//   const float &x() const { return vec[0]; }
//   const float &y() const { return vec[1]; }
//   const float &z() const { return vec[2]; }
// };

/* wrap reading/writing structs aw variables related le ba3d b taskENTER_CRITICAL() and taskEXIT_CRITICAL()
  bas keep them short and fast with no blocking functions inside
  3ashan mayektebsh half the data and then ye7sal interrupt fa yeb2a nos el data new w nos old*/

bool walls[3] = {false}; // front left right
vec_3 euler;//TODO:IMPORTANT CHECK THE UNITS OF EULER 
vec_3 gyro;
double ir_readings[6] = {0}; // left_front, right_front, left,right,left_diag, right_diag
double ir_distance[6] = {1.0,1.0,1.0,1.0,1.0,1.0};  //l7d ma el ir task ytktb  // in meters 
struct Pose position = {0, 0, 0};
struct velocity robot_velocity = {0, 0, 0, 0};

QueueHandle_t motionCmdQueue;
QueueHandle_t motionStatusQueue; // queue 3ashan ye trigger algorithm when status changes
MotionType motionType = STOP;
double target_v= 0.0; // m/s, set by algorithm task, read by control task
// TODO: tune these // km_ff tau_ff kp ki
FFPIConfig left_config = {0.05f, 0.12f, 0, 0};

FFPIConfig right_config = {0.05f, 0.12f, 0, 0};
static VelocityController leftCtrl(left_config);
static VelocityController rightCtrl(right_config);
// lookahead, wheel_base, kp_omega, kd_omega
static PurePursuitPD purePursuit(0, 0, 0, 0);
struct wheelVelocity wheel_ref = {0, 0};//purepursuit writes this
std::vector<Point> current_path; // pure pursuit reads this & algorithm writes this
SemaphoreHandle_t pathMutex; // mutex to protect access to current_path
volatile uint32_t path_version = 0; // incremented by algorithm task when it writes a new path, read by control task to know if it needs to copy the new path
volatile uint32_t cmd_id = 0; // incremented by algorithm task when it writes a new command
float wrapAngle(float angle){
  while(angle > 180) angle -= 360;
  while(angle < -180) angle += 360;
  return angle;
}
extern "C" void app_rtos_init(void)
{
  if (!pathMutex)          pathMutex = xSemaphoreCreateMutex();
  if (!motionStatusQueue)  motionStatusQueue = xQueueCreate(4, sizeof(MotionStatus_t));
  if (!motionCmdQueue)     motionCmdQueue = xQueueCreate(4, sizeof(MotionCommand_t));
}

void motors(TIM_HandleTypeDef* htim, uint32_t channel ,GPIO_TypeDef* port,
                uint16_t pin, double speed)
{
  if(speed > 100.f) 
    speed = 100.f;
  if(speed < -100.f) 
    speed = -100.f;

    double pwm = (fabs(speed) / 100.f) * MOTOR_PWM_MAX_CCR; // convert speed percentage to pwm duty cycle
    if (pwm >MOTOR_PWM_MAX_CCR)
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
  void motor_speeds (float left_speed, float right_speed) {
   // motors(&MOTOR_LEFT_TIM, MOTOR_LEFT_CHANNEL, MOTOR_DIR_LEFT_GPIO_Port, MOTOR_DIR_LEFT_Pin, left_speed);
   // motors(&MOTOR_RIGHT_TIM, MOTOR_RIGHT_CHANNEL, MOTOR_DIR_RIGHT_GPIO_Port, MOTOR_DIR_RIGHT_Pin, right_speed);
  }
 

////////////////////////////////////////////////TASKS////////////////////////////////////////////////////
void StartDefaultTask_run(void *arg)
{
  for(;;)
  {

  }

}

void bnoTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();
   
  imu bno(&hi2c2 , 0x28); // TODO: check the address 
  bno.init();
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    // read i2c
    // write euler, gyro
      vec_3 v =bno.euler();
      vec_3 g = bno.gyro();

      // printf("yaw: %f, pitch: %f, roll: %f\n", v.y(), v.x(), v.z());
      taskENTER_CRITICAL();
      euler = v;
      gyro = g;
      taskEXIT_CRITICAL();
   
  }
}

void motionTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  // iir filter
  // write position
  const float mm_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick

double dt = ENCODER_TASK_DT_S; // TODO: is it better to calculate dt every loop?
  ButterworthIIR velL;
  ButterworthIIR velR;
  
  velL.init(1,200.0f );     // cutoff freq, sample rate 5ms
  velR.init(1,200.0f );
  velL.reset();
  velR.reset();

  EncoderCount_t left_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
  EncoderCount_t right_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
  EncoderCount_t lastCountL = 0;
  EncoderCount_t lastCountR = 0;
  MotionType lastMotionTypeMotion = STOP;

  uint32_t last_cmd_id = 0; // b3rad el command id 3ashan ne know law command gdeda w nreset el controllers

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
    // TODO: et2akedy men dool
    float distance_center = ((deltaL * mm_per_tick) + (deltaR * mm_per_tick)) *0.5f;




    /*straight line: PD Controller + IR centering + longitudinal correction with diagonal IRs
      turns: Pure pursuit only, corrected using gyro. there is no IR correction in turns

      error generated from purepursuit is corrected by the IRs later
      the error should be negligible for one turn
    */
    taskENTER_CRITICAL();
    MotionType current_motion = motionType;
    uint32_t cmd = cmd_id;
    wheelVelocity wheel_speed = wheel_ref;
    float heading = euler.y();     //degree
    taskEXIT_CRITICAL();
    
    if(motionType != lastMotionTypeMotion || cmd != last_cmd_id){
      //do i reset these? ana mayla le both reset or not reset so idk
      leftCtrl.reset();
      rightCtrl.reset();
      lastMotionTypeMotion = current_motion;
      last_cmd_id = cmd;
    }
   float left_cmd = 0.0f;
   float right_cmd = 0.0f;
if(current_motion != STOP){
    left_cmd = leftCtrl.compute(wheel_speed.left,velLfiltered,dt);
    right_cmd = rightCtrl.compute(wheel_speed.right,velRfiltered,dt);
}
  motor_speeds(left_cmd,right_cmd);



    // update global
    // 2 critical blocks 3ashan mesh taba3 ba3d w law 3ayez ye3mel interrupt mabenhom no problem
    taskENTER_CRITICAL();
    position.theta = heading*(M_PI / 180.0); // wont calculate angle from encoders
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


void controlTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  double dt = 0.005; // TODO: is it better to calculate dt every loop?
  // std::vector<Point> path;
  PDController headingHoldPD(0,0,-100,100);//TODO:tune kp,kd
  PDController lateralPD(0,0,-100,100); //TODO:tune kp,kd
  float target_heading = 0.0f; // read only the moment we lose wall reference
  float lateral_correction = 0.0f;
  int last_mode = -1; // to detect change in motion type
  MotionType lastMotionTypeCtrl = STOP; 
  uint32_t last_cmd_id = 0; // b3rad el command id 3ashan ne know law command gdeda w nreset el controllers
  bool status_sent = false; //3shan ne send status only once when motion is done
 
  std::vector<Point> path_copy; 
  uint32_t local_ver = 0;

  purePursuit.reset();

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
     
     if (path_version != local_ver) {
      xSemaphoreTake(pathMutex, portMAX_DELAY);
      path_copy = current_path;
      local_ver = path_version;
      xSemaphoreGive(pathMutex);
    
     }
    taskENTER_CRITICAL();
    Pose current_pose = position;
    double v_measured = robot_velocity.v;
    double omega_measured = robot_velocity.omega;
    double left_distance = ir_distance[2];
    double right_distance = ir_distance[3];
    double front_distance = (ir_distance[0] + ir_distance[1]) * 0.5; 
    MotionType current_motion = motionType;
    double current_target_v = target_v;
    uint32_t cmd = cmd_id;
    // float heading = euler.y();  //heading_sign *euler.y(); //degree ccw positive
    bool wall_front = walls[0];
    // bool wall_left = walls[1];
    // bool wall_right = walls[2];
    taskEXIT_CRITICAL();

    if(motionType != lastMotionTypeCtrl || cmd != last_cmd_id){
      headingHoldPD.reset();
      lateralPD.reset();
      purePursuit.reset();    
      lastMotionTypeCtrl = current_motion;
      last_cmd_id = cmd;
      status_sent = false; //b nreeset el flag 
     last_mode = -1; // reset last mode to force re-evaluation of motion type
    }
   // emergency stop if front wall detected & 🛺 lsa mkml staright
    if (current_motion == STRAIGHT && (wall_front || front_distance < 0.02f ))  {
      // TODO: tune threshold
      
      taskENTER_CRITICAL();
      wheel_ref.left = 0.0;
      wheel_ref.right = 0.0;
      taskEXIT_CRITICAL();
      
    if (!status_sent) {
          MotionStatus_t status = {0};
          xQueueSend(motionStatusQueue, &status, 0);
          status_sent = true;
    }
      continue;
    }
    if (current_motion == STRAIGHT)
    {
      //TODO: wrap reading the walls in CRITICAL section
      double base_v = target_v;
      float lateral_error = 0.0f;
      if (walls[1] && walls[2]){ // 2 side walls
        lateral_error = left_distance - right_distance;
        last_mode = 0; // STRAIGHT
        lateral_correction = lateralPD.compute(0.0f,lateral_error,dt);
      }
      else if (walls[1]) {// only left wall
        lateral_error = left_distance - EXPECTED_SIDE_DIST_TO_WALL;
        last_mode = 1; // LEFT_WALL
        lateral_correction = lateralPD.compute(0.0f,lateral_error,dt);
      }
      else if (walls[2]){ // only right wall
        lateral_error = EXPECTED_SIDE_DIST_TO_WALL - right_distance;
        last_mode = 2; // RIGHT_WALL
        lateral_correction = lateralPD.compute(0.0f,lateral_error,dt);
      }

      else{ // no side walls then hold current heading //dont know law dah momken ye7sal aslan bas better safe
        if(last_mode == -1){
          target_heading = euler.y();
          last_mode = 3; // NO_WALLS
        }
        float heading_error = wrapAngle(target_heading - euler.y());
        lateral_correction = headingHoldPD.compute(0.0f,-heading_error,dt);
      }
      //TODO : 
      taskENTER_CRITICAL();
      wheel_ref.left = base_v - lateral_correction;
      wheel_ref.right = base_v + lateral_correction;
      taskEXIT_CRITICAL();
    }
    else if (current_motion == TURN)
    {
       wheelVelocity wheel_speed = purePursuit.computeControl(current_pose, v_measured, omega_measured, target_v, path_copy, dt);
          taskENTER_CRITICAL();
           wheel_ref = wheel_speed;
          taskEXIT_CRITICAL();

         if (!path_copy.empty() && !status_sent && motionStatusQueue != NULL) {
          
        const Point& goal = path_copy.back();
        double dist_to_goal = std::hypot(goal.x - current_pose.x, goal.y - current_pose.y);
        
      if (dist_to_goal < 0.01) { // threshold to consider the turn complete
        MotionStatus_t status = {false}; // done
        xQueueSend(motionStatusQueue, &status,0 );
        status_sent = true;
      }
    }}
    else  if (current_motion == STOP){ 
        // TODO: need to make a case for STOP 
      taskENTER_CRITICAL();
      wheel_ref.left = 0;
      wheel_ref.right = 0;
      taskEXIT_CRITICAL();
       ////kda mafrood el el algo ykteen el current_path f el pathMutex w b3den path ver++ 
       ///w yzwed el cmd_id f col 2mr gded 7ta lw nafs el motion type 3shan a3rf a3ml reset w el status_sent flag
  
  
}}}

void algorithmTask_run(void *arg)
{
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
  for(;;)
  {

  }
}

void loggerTask_run(void * arg)
{
  for(;;)
  {
    
  }
}
//////////////////////////////////////////////////END TASKS//////////////////////////////////////////////
void app_main() {
  // Write your C++ application code here
  // This acts as your new int main()
  while(1) {

  }
}
