#include "main.h"
#include "FreeRTOS.h"
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
#define MOTOR_RIGHT_TIM 1           // TODO
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
vec_3 euler;//TODO:IMPORTANT CHECK THE UNITS OF EULER 
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

float wrapAngle(float angle){
  while(angle > 180) angle -= 360;
  while(angle < -180) angle += 360;
  return angle;
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
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    // read i2c
    // write euler, gyro
  }
}

void motionTask_run(void *arg)
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


void controlTask_run(void *arg)
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