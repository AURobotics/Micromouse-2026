#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
// #include "adc.h"
// #include "cmsis_os2.h"
// #include "gpio.h"
// #include "i2c.h"
// #include "tim.h"
// #include "usart.h"
// #include "queue.h"
#include "app_main.h"
// #include "pure_pursuit.h"
// #include "feedforward_pi.h"
#include "iirFilter.h"
#include "eeprom.h"
// #include "PDcontroller.h"
#include "math.h"
#include "stdio.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
/* TODO: Calibrate adc, check adc calibration modes...
 * useful links: https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples/#introducing-stm32-adc
 *
 * taskname_run 3ashan freertos owns the tasks fa we'll call these functions gwa freertos.c
 * mesh katbeen el tasks henak fy freertos.c 3ashan el global variables kolaha teb2a hena
 * w el tasks and stuff cpp
 */
// na2alt all global variables fy app_main.h kol el global variables in one place insha2allah b2ezn allah maye7salsh moshkela

////////////////////////////////////////////////TASKS////////////////////////////////////////////////////
void StartDefaultTask_run(void *arg)
{
  /////////////////////////////////////////motion////////////////////////////////////
  // TickType_t last = xTaskGetTickCount();

  // // iir filter
  // // write position
  // const float mm_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick
  // double dt = 0.005; // TODO: is it better to calculate dt every loop?
  // // velL.init(1,200.0f );     // cutoff freq, sample rate 5ms
  // ButterworthIIR velR;
  // velR.init(1,200.0f );
  // velR.reset();

  // EncoderCount_t right_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);

  // EncoderCount_t lastCountR = 0;
  // uint32_t notifiedValue;
  // for (;;)
  // {
  //   BaseType_t gotNotification = xTaskNotifyWait(
  //           0x00,
  //           ULONG_MAX,
  //           &notifiedValue,
  //           pdMS_TO_TICKS(5));

  //   uint16_t raw_R = __HAL_TIM_GET_COUNTER(&htim3);
  //   int16_t deltaR = (int16_t)(raw_R - lastCountR);
  //   right_count += deltaR;
  //   lastCountR = raw_R;

  //   float rawVelR = (deltaR * mm_per_tick) / ENCODER_TASK_DT_S;

  //     float distance = deltaR * mm_per_tick;
  //   printf("raw_R:%d  total_cnt_R:%ld   rawVel: %f  distance: %f\n",raw_R,right_count,rawVelR,distance);
  //   motor_speed(100);

  /////////////////////////////////////////testingeeprom////////////////////////////////////
  int first = 1;
  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(500));

    if (first)
    {
      int16_t fake_calib[11] = {-21,15,3,189,-496,57,2,0,-1,1000,957};
      uint8_t calib_buf[22];
      memcpy(calib_buf, fake_calib, sizeof(calib_buf));
      
      if (writeCalibration(&hi2c1, 20, calib_buf, 22))
      {
        printf("yay wrote calibration successfully!\n");
        first = 0;
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      else
      {
        printf(":( didnt write calibration on eeprom\n trying again...\n");
        first = 1;
      }
    }
    else
    {
      uint8_t read_data[22];
      if (readCalibration(&hi2c1, 20, read_data, 22))
      {
        int16_t calib_data[11];
        memcpy(calib_data,read_data,22);
        for (int i = 0; i < 11; i++)
        {
          printf("%d ", calib_data[i]);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    }
  }
}

// void bnoTask_run(void *arg)
// {
  // TickType_t last = xTaskGetTickCount();
  // for (;;)
  // {
  //   vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
  //   // read i2c
  //   // write euler, gyro
  // }
// }

// void motionTask_run(void *arg)
// {
//   TickType_t last = xTaskGetTickCount();

//   // iir filter
//   // write position
//   const float mm_per_tick = (float)M_PI * WHEEL_DIAMETER / ENCODER_CPR; // meter of travel per encoder tick
//   double dt = 0.005; // TODO: is it better to calculate dt every loop?
//   // velL.init(1,200.0f );     // cutoff freq, sample rate 5ms
//   velR.init(1,200.0f );
//   // velL.reset();
//   velR.reset();
//   // for(int i=0;i<6;i++){
//   //   ir_iir[i].init(1,200.0f);//TODO:cutoff frequency, sample rate
//   //   ir_iir[i].reset();
//   // }

//   /*tim3 is 16bits fa left/right_count feehom total count bas lazem el 16bit ne handle its overflow*/
//   // EncoderCount_t left_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
//   EncoderCount_t right_count = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);
//   // EncoderCount_t lastCountL = 0;
//   EncoderCount_t lastCountR = 0;
//   MotionType lastMotionTypeMotion = STOP;
//   uint32_t notifiedValue;
//   for (;;)
//   {
//     BaseType_t gotNotification = xTaskNotifyWait(
//             0x00,
//             ULONG_MAX,
//             &notifiedValue,
//             pdMS_TO_TICKS(5));

//     // if(gotNotification == pdTRUE && notifiedValue)
//     // {
//     //   processIrBuffer(ir_adc_buf);
//     //   for(int i=0;i<6;i++){
//     //     ir_readings[i] = ir_iir[i].filter(ir_readings[i]);
//     //   }
//     // }

//     // EncoderCount_t countL = (EncoderCount_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
//     uint16_t raw_R = __HAL_TIM_GET_COUNTER(&htim3);
//     int16_t deltaR = (int16_t)(raw_R - lastCountR);
//     right_count += deltaR;
//     lastCountR = raw_R;

//     // lastCountL = countL;

//     // --- counts -> distance (m) -> raw velocity (m/s) ---
//     // float rawVelL = (deltaL * mm_per_tick) / ENCODER_TASK_DT_S;
//     float rawVelR = (deltaR * mm_per_tick) / ENCODER_TASK_DT_S;
//     //////?????
//     // float velLfiltered = velL.filter(rawVelL);
//     // float velRfiltered = velR.filter(rawVelR);
//     // float v = (velLfiltered + velRfiltered) * 0.5f;       // m/s, forward speed
//     // float w = (velRfiltered - velLfiltered) / WHEEL_BASE; // rad/s, positive = turning left
//     // float dTheta = w * ENCODER_TASK_DT_S;
//     // TODO: et2akedy men dool
//     // float distance_center = ((deltaL * mm_per_tick) + (deltaR * mm_per_tick)) / 2.0f;
//       float distance = deltaR * mm_per_tick;
//     printf("raw_R:%d  total_cnt_R:%ld   rawVel: %ld   /*velFiltered: %ld*/   distance: %ld\n",raw_R,right_count,rawVelR,distance);
//     motor_speed(100);

//     /*straight line: PD Controller + IR centering + longitudinal correction with diagonal IRs
//       turns: Pure pursuit only, corrected using gyro. there is no IR correction in turns

//       error generated from purepursuit is corrected by the IRs later
//       the error should be negligible for one turn
//     */

//     // if(motionType != lastMotionTypeMotion){
//     //   //do i reset these? ana mayla le both reset or not reset so idk
//     //   leftCtrl.reset();
//     //   rightCtrl.reset();
//     //   lastMotionTypeMotion = motionType;
//     // }
//     // taskENTER_CRITICAL();
//     // wheelVelocity wheel_speed = wheel_ref;
//     // taskEXIT_CRITICAL();

//     // double left_cmd = leftCtrl.compute(wheel_speed.left,velLfiltered,dt);
//     // double right_cmd = rightCtrl.compute(wheel_speed.right,velRfiltered,dt);

//     // // update global
//     // // 2 critical blocks 3ashan mesh taba3 ba3d w law 3ayez ye3mel interrupt mabenhom no problem
//     // taskENTER_CRITICAL();
//     // position.theta = euler.y(); // wont calculate angle from encoders
//     // position.x += distance_center * cos(position.theta);
//     // position.y += distance_center * sin(position.theta);
//     // taskEXIT_CRITICAL();

//     // taskENTER_CRITICAL();
//     // robot_velocity.omega = w;
//     // robot_velocity.v = v;
//     // robot_velocity.vL = velLfiltered;
//     // robot_velocity.vR = velRfiltered;
//     // taskEXIT_CRITICAL();
//     //  // target v and motion type are written by algo task and read inside controller&motion tasks f kda b acess el motiontype w target v atomatically
//     // taskENTER_CRITICAL();
//     // MotionType current_motion = motionType;
//     // double current_target_v = target_v;
//     // taskEXIT_CRITICAL();

//     // TODO:drive motors dont know pins and stuff yet
//   }
// }

/*
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
    double front_distance = (ir_distance[0] + ir_distance[1]) * 0.5;
    MotionType current_motion = motionType;
    double current_target_v = target_v;
    std::vector<Point> path_copy = current_path;
    bool wall_front = walls[0];
    bool wall_left = walls[1];
    bool wall_right = walls[2];
    taskEXIT_CRITICAL();

    if(motionType != lastMotionTypeCtrl){
      headingHoldPD.reset();
      lateralPD.reset();
      purePursuit.reset();
      lastMotionTypeCtrl = motionType;
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
    else if (motionType == TURN)
    {
       wheelVelocity wheel_speed = purePursuit.computeControl(current_pose, v_measured, omega_measured, target_v, path, dt);
          taskENTER_CRITICAL();
           wheel_ref = wheel_speed;
          taskEXIT_CRITICAL();

         if (!path.empty()) {
        const Point& goal = path.back();
        double dist_to_goal = std::hypot(goal.x - current_pose.x, goal.y - current_pose.y);


      if (dist_to_goal < 0.01) { // threshold to consider the turn complete
        MotionStatus_t status = {false}; // done
        xQueueSend(motionStatusQueue, &status,0 );
      }
    }}
    else  if (motionType == STOP){
    {    // TODO: need to make a case for STOP
      taskENTER_CRITICAL();
      wheel_ref.left = 0;
      wheel_ref.right = 0;
      taskEXIT_CRITICAL();


  }
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

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_TIM_DISABLE(&htim8);
        __HAL_TIM_SET_COUNTER(&htim8, 0);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(motionTaskHandle, 1, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void processIrBuffer(uint32_t * buf){
  //TODO: 3ala asas en el ranking goes like this --> side,diagonal,front // ehna mota7akemeen ay haga 3ayzenha el mohem yeb2a consistent

    ir_readings[0] = buf[0] & 0x0FFF;
    ir_readings[1] = (buf[0] >> 16) & 0x0FFF;

    ir_readings[2] = buf[1] & 0x0FFF;
    ir_readings[3] = (buf[1] >> 16) & 0x0FFF;

    ir_readings[4] = buf[2] & 0x0FFF;
    ir_readings[5] = (buf[2] >> 16) & 0x0FFF;

}
//////////////////////////////////////////////////END TASKS//////////////////////////////////////////////

void app_main() {
  // Write your C++ application code here
  // This acts as your new int main()
  while(1) {

  }
}

*/