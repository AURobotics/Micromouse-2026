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
#include "BNO055.h"

////////////////////////////////////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////
//sorry na2altohom kolohom hena 3ashan some c/c++ sh!t kan nefsy a7otohom fy app_main.h bas mashakel w mesh adra
typedef uint32_t EncoderCount_t; // adjust 3la 16 bit or 32 bit based on the encoder timer
// tim2 and timer 5 -->32 bit
// tim3 and timer 4 -->16 bit
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
  int status; // 0 = done, 1 = running
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

vec_3 euler;//TODO:IMPORTANT CHECK THE UNITS OF EULER 
vec_3 gyro;

volatile uint32_t adc_dma_buffer[3];
uint16_t ir_sequence[9] = {//TODO:check this
    0, 1260,    0,   // Pulse 1: Only Channel 2 is ON
    0,    0, 1260,   // Pulse 2: Only Channel 3 is ON
    1260,    0,    0 // Pulse 3: Only Channel 1 is ON
};

bool walls[3] = {0};
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
VelocityController leftCtrl(left_config);
VelocityController rightCtrl(right_config);
// lookahead, wheel_base, kp_omega, kd_omega
static PurePursuitPD purePursuit(0, 0, 0, 0);
struct wheelVelocity wheel_ref = {0, 0};//purepursuit writes this
std::vector<Point> current_path; // pure pursuit reads this & algorithm writes this


/* TODO: Calibrate adc, check adc calibration modes...
* useful links: https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples/#introducing-stm32-adc
* 
* taskname_run 3ashan freertos owns the tasks fa we'll call these functions gwa freertos.c 
* mesh katbeen el tasks henak fy freertos.c 3ashan el global variables kolaha teb2a hena 
* w el tasks and stuff cpp 
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float wrapAngle(float angle){
  while(angle > 180) angle -= 360;
  while(angle < -180) angle += 360;
  return angle;
}

// ADC DMA Callback function
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) {
    // stop
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SetCompare(&htim2, 0, 1260);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t) MotionTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
  }
}
//printf->SWO
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar((uint32_t)ptr[i]);
    }
    return len;
}

void SWO_Init(void)//in order to use ITM_SendChar
{
    // Enable trace subsystem
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // TPIU/ITM config — assumes core clock known, SWO baud rate e.g. 2000000
    *((volatile unsigned int*)0xE0040010) = HAL_RCC_GetHCLKFreq() / 2000000 - 1; // TPIU prescaler for SWO baud

    *((volatile unsigned int*)0xE00400F0) = 2; // Selected PIN Protocol Register: 2 = NRZ

    // Enable ITM, port 0
    ITM->LAR = 0xC5ACCE55;       // Unlock
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk;
    ITM->TER = 1;                 // Enable stimulus port 0
}

void motor_speeds(int16_t left_duty,int16_t right_duty){
  left_duty *= 9.99;
  right_duty *= 9.99;

  if(left_duty>999) left_duty = 999;
  if(left_duty<-999) left_duty = -999;
  if(right_duty>999) right_duty= 999;
  if(right_duty<-999) right_duty = -999;

  //TODO: define channels
  //   if (right_duty >= 0)
  // {
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_FORWARD_CHANNEL, right_duty);
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_BACKWARD_CHANNEL, 0);
  // }
  // else
  // {
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_FORWARD_CHANNEL, 0);
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_BACKWARD_CHANNEL, -right_duty);
  // }

  //   if (left_duty >= 0)
  // {
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_FORWARD_CHANNEL, left_duty);
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_BACKWARD_CHANNEL, 0);
  // }
  // else
  // {
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_FORWARD_CHANNEL, 0);
  //     __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_BACKWARD_CHANNEL, -left_duty);
  // }
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
  imu bno(&hi2c2,0x29);//TODO: check address with physical connection
  bno.init();
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    vec_3 v = bno.euler();
    vec_3 u = bno.gyro();
    taskENTER_CRITICAL();
    euler = v;
    gyro = u;
    taskEXIT_CRITICAL();
  }
}

void motionTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  // iir filter
  ButterworthIIR ir_iir[6];
  for(int i=0;i<6;i++) ir_iir[i].init(1,200.0f); //TODO:tune
  // write position
  const float mm_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick
  ButterworthIIR velL;
  ButterworthIIR velR;
  double dt = 0.005; 
  velL.init(1,200.0f );     //TODO: cutoff freq, sample rate 5ms
  velR.init(1,200.0f );
  velL.reset();
  velR.reset();

  EncoderCount_t countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
  EncoderCount_t countR = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
  EncoderCount_t lastCountL = 0;
  EncoderCount_t lastCountR = 0;
  MotionType lastMotionTypeMotion = STOP;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ////////////////////////////IRS//////////////////////
    taskENTER_CRITICAL();
    for(int i=0;i<6;i+=2){
      ir_readings[i] = adc_dma_buffer[i/2] & 0xFFFF;
      ir_readings[i+1] = (adc_dma_buffer[i/2] >> 16) & 0xFFFF;

      //iir filter
      ir_readings[i] = ir_iir[i].filter(ir_readings[i]);
      ir_readings[i+1] = ir_iir[i+1].filter(ir_readings[i+1]);
    }
    taskEXIT_CRITICAL();


    ///////////////////////ENCODERS/////////////////////
    // overflow logic for 16bit timer tim3
    uint16_t raw_R = __HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
    int32_t deltaR = (int16_t)(raw_R - lastCountR);
    countR += deltaR;

    countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    int32_t deltaL = (int32_t)(EncoderCount_t)(countL - lastCountL);

    lastCountL = countL;
    lastCountR = raw_R;
    

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

    double left_cmd,right_cmd;
    if(motionType == STOP)
    {
      left_cmd = 0;
      right_cmd = 0;
    }
    else 
    {
      left_cmd = leftCtrl.compute(wheel_speed.left,velLfiltered,dt);
      right_cmd = rightCtrl.compute(wheel_speed.right,velRfiltered,dt);
    }
    
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
     // target v and motion type are written by alogo task and read inside controller&motion tasks f kda b acess el motiontype w target v atomatically
    
    motor_speeds(left_cmd,right_cmd);
  }
}


void controlTask_run(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  double dt = 0.005; 
  
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
    double front_distance = (ir_distance[0] + ir_distance[1]) * 0.5;
    MotionType current_motion = motionType;
    double current_target_v = target_v;
    std::vector<Point> path_copy = current_path;
    bool wall_front = walls[0];
    bool wall_left = walls[1];
    bool wall_right = walls[2];
    taskEXIT_CRITICAL();

    if(current_motion != lastMotionTypeCtrl){
      headingHoldPD.reset();
      lateralPD.reset();
      purePursuit.reset();    
      lastMotionTypeCtrl = current_motion;
      //regenerate points for purepursuit 
    }
   // emergency stop if front wall detected & 🛺 lsa mkml staright
    if (current_motion == STRAIGHT && (wall_front || front_distance < 0.02f ))  // TODO: tune threshold 
    {
      taskENTER_CRITICAL();
      wheel_ref.left = 0.0;
      wheel_ref.right = 0.0;
      taskEXIT_CRITICAL();

      MotionStatus_t status = {false}; // done
      xQueueSend(motionStatusQueue, &status, 0);
      continue; // skip the rest of the loop
    }
    if (current_motion == STRAIGHT)
    {
      //TODO: wrap reading the walls in CRITICAL section
      double base_v = current_target_v;
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
    else if (current_motion == TURN)
    {
       wheelVelocity wheel_speed = purePursuit.computeControl(current_pose, v_measured, omega_measured, current_target_v, path_copy, dt);
          taskENTER_CRITICAL();
           wheel_ref = wheel_speed;
          taskEXIT_CRITICAL();

         if (!path_copy.empty()) {
        const Point& goal = path_copy.back();
        double dist_to_goal = std::hypot(goal.x - current_pose.x, goal.y - current_pose.y);
        
        
      if (dist_to_goal < 0.01) { // threshold to consider the turn complete
        MotionStatus_t status = {false}; // done
        xQueueSend(motionStatusQueue, &status,0 );
      }
    }}
    else  if (current_motion == STOP){ 
    {    // TODO: need to make a case for STOP 
      taskENTER_CRITICAL();
      wheel_ref.left = 0;
      wheel_ref.right = 0;
      taskEXIT_CRITICAL();
      
  
  }
}}}

void algorithmTask_run(void *arg)
{
  for (;;)
  {
    
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