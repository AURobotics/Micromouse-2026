#include "app_main.h"
#include <../Inc/motor.h>
#include <cmath>
#include <vector>
#include "../Inc/iirFilter.h"
#include "BNO055.h"
#include "FreeRTOS.h"
#include "PDcontroller.h"
#include "adc.h"
#include "cmsis_os.h"
#include "eeprom.h"
#include "feedforward_pi.h"
#include "i2c.h"
#include "imu_utility.h"
#include "main.h"
#include "pure_pursuit.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "tim.h"
#include "usart.h"

#include "test_lib.h"
// TODO: wrap ir stuff in CRITICAL sections cuz they're shared across more than 1 task
/* TODO: Calibrate adc, check adc calibration modes...
 * useful links:
 * https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples/#introducing-stm32-adc
 *
 * taskname_run 3ashan freertos owns the tasks fa we'll call these functions gwa freertos.c
 * mesh katbeen el tasks henak fy freertos.c 3ashan el global variables kolaha teb2a hena
 * w el tasks and stuff cpp
 */
#define IR_PULSE_US 100u
#define N_WIN 15

static void init_ir();

extern "C" {
extern osThreadId_t MotionTaskHandle; // defined in freertos.c
}

bool pausedAndReset = true; // started it as true 3ashan yebda2 el motors mesh sha8aleen and then
                            // start on pressing the button
bool calibrateIR = false;
bool calibrateBNO = false;
uint32_t lastIrCalTick = 0;
uint32_t lastBnoCalTick = 0;
uint32_t lastStartStopTick = 0;

Motor right_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, &RIGHT_ENCODER_TIM);
Motor left_motor(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, &LEFT_ENCODER_TIM);
// use this to know the type of motion 3ashan ne center the robot only in STRAIGHT segments
// algorithm task controls it
enum class MotionType : uint8_t
{
    FORWARD,
    STOP,
    TURN,
    TURN_IN_PLACE
};

typedef struct
{
    MotionType type;
} MotionCommand_t;

typedef struct
{
    bool status; // 0 = done, 1 = running
} MotionStatus_t;

struct velocity
{
    double v; // m/s
    double omega; // rad/s
    double vL; // m/s    left wheel
    double vR; // m/s     right wheel
};

/* wrap reading/writing structs aw variables related le ba3d b taskENTER_CRITICAL() and
  taskEXIT_CRITICAL() bas keep them short and fast with no blocking functions inside 3ashan
  mayektebsh half the data and then ye7sal interrupt fa yeb2a nos el data new w nos old*/

volatile uint32_t adc_dma_buffer[3];
uint16_t ir_sequence[9] = {
    IR_PULSE_US, 0,           0, /* SIDE     -> sub-slot 2 */
    0,           IR_PULSE_US, 0, /* DIAGONAL -> sub-slot 3 */
    0,           0,           IR_PULSE_US /* FRONT    -> sub-slot 1 of the next cycle */
};

bool walls[3] = {false}; // front left right
vec_3 euler; // TODO:IMPORTANT CHECK THE UNITS OF EULER
vec_3 gyro;
uint16_t ir_readings[6] = {0}; // left_front, right_front, left,right,left_diag, right_diag
// double ir_distance[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // l7d ma el ir task ytktb  // in meters
int16_t ir_thresh[6];
// uint32_t calReadings[4][6];
// uint16_t calDistances[6] = {2,4,6,8,10,14};
Pose position = {0, 0, 0};
velocity robot_velocity = {0, 0, 0, 0};

QueueHandle_t motionCmdQueue;
QueueHandle_t motionStatusQueue; // queue 3ashan ye trigger algorithm when status changes
MotionType motionType = MotionType::STOP;
double target_v = 0.0; // m/s, set by algorithm task, read by control task
// TODO: tune these // km_ff tau_ff kp ki
FFPIConfig left_config = {0.0057f, 0.125f, 1.6, 16}; // max speed = 0.63
FFPIConfig right_config = {0.0056f, 0.125f, 0.8, 11}; // max speed = 0.62
static VelocityController leftCtrl(left_config);
static VelocityController rightCtrl(right_config);
// lookahead, wheel_base, kp_omega, kd_omega
static PurePursuitPD purePursuit(0, 0, 0, 0);
struct wheelVelocity wheel_ref = {0, 0}; // purepursuit writes this
std::vector<Point> current_path; // pure pursuit reads this & algorithm writes this
SemaphoreHandle_t pathMutex; // mutex to protect access to current_path
volatile uint32_t path_version = 0; // incremented by algorithm task when it writes a new path, read
                                    // by control task to know if it needs to copy the new path
volatile uint32_t cmd_id = 0; // incremented by algorithm task when it writes a new command

/////////////////////////////////////////////////HELPER
/// FUNCTIONS//////////////////////////////////////////////////
// uint32_t micros(void) { return __HAL_TIM_GET_COUNTER(&htim5); }
// uint32_t millis(void) { return __HAL_TIM_GET_COUNTER(&htim5) / 1000; }
uint32_t millis(void) { return HAL_GetTick(); }
extern "C" int _write(int file, char* ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// ai generated functions for testing purepursuit
// probably will need something liek this nedeeh el cells el ehna 3ayzeen nero7ha w it generates a
// line
std::vector<Point> makeLine(double x0, double y0, double x1, double y1, double step = 0.01)
{
    std::vector<Point> p;
    double len = std::hypot(x1 - x0, y1 - y0);
    int n = std::max(1, (int)(len / step));
    for (int i = 0; i <= n; ++i)
    {
        double t = (double)i / n;
        p.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
    }
    return p;
}

std::vector<Point> makeArc(double cx, double cy, double r, double a0, double a1, double step = 0.01)
{
    std::vector<Point> p;
    int n = std::max(1, (int)(std::fabs(a1 - a0) * r / step));
    for (int i = 0; i <= n; ++i)
    {
        double a = a0 + (a1 - a0) * i / n;
        p.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    return p;
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

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_TIM_DISABLE(&htim8); // stop ADC trigger and LED pulses
        __HAL_TIM_SET_COUNTER(&htim8, __HAL_TIM_GET_AUTORELOAD(&htim8)); // next tick = update event

        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream7->CR & DMA_SxCR_EN)
        {
        }
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7;
        DMA2_Stream7->NDTR = 9;
        DMA2_Stream7->CR |= DMA_SxCR_EN;

        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, IR_PULSE_US);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR((TaskHandle_t)MotionTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_Pin == BTN_STARTSTOP_Pin)
    {
        uint32_t now = millis();
        if (now - lastStartStopTick > 200)
        {
            lastStartStopTick = now;
            vTaskNotifyGiveFromISR((TaskHandle_t)defaultTaskHandle, &xHigherPriorityTaskWoken);
        }
    }
    else if (GPIO_Pin == BTN_IRCAL_PIN)
    {
        uint32_t now = millis();
        if (now - lastIrCalTick > 200)
        {
            lastIrCalTick = now;
            calibrateIR = true;
            vTaskNotifyGiveFromISR((TaskHandle_t)HMITaskHandle, &xHigherPriorityTaskWoken);
        }
    }
    else if (GPIO_Pin == BTN_BNOCAL_PIN)
    {
        uint32_t now = HAL_GetTick();
        if (now - lastBnoCalTick > 200)
        {
            lastBnoCalTick = now;
            calibrateBNO = true;
            vTaskNotifyGiveFromISR((TaskHandle_t)BnoTaskHandle, &xHigherPriorityTaskWoken);
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void IRCalibration(/*uint8_t sensor*/)
{

    printf("IR Calibration: position robot and then press button 2\r\n");

    // Block here until the EXTI ISR gives a notification for this button
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    long sum[6] = {0};
    const size_t n_samples = 20;
    for (int s = 0; s < n_samples; s++)
    {
        for (int i = 0; i < 6; i++)
            sum[i] += ir_readings[i];
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // calReadings[sensor][i] = sum / 20;
    for (int i = 0; i < 6; i++)
        ir_thresh[i] = static_cast<int16_t>(sum[i] / n_samples);
    // }

    // printf(" calReadings[%d] = ", sensor);
    // for (int i = 0; i < 6; i++) printf("%ld, ", calReadings[sensor][i]);
    printf("IR calibration done\r\n");
    calibrateIR = false;
}

bool saveIRCalToEEPROM(I2C_HandleTypeDef* i2c, int16_t thresholds[6])
{
    // write data block first
    if (!writeCalibration(i2c, EEPROM_ADDR_IR_CAL, (uint8_t*)thresholds, 6 * sizeof(int16_t)))
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

bool loadIRCalFromEEPROM(I2C_HandleTypeDef* i2c)
{
    uint8_t magic = 0;
    if (!readCalibration(i2c, EEPROM_ADDR_IR_VALID, &magic, 1))
        return false;

    if (magic != EEPROM_MAGIC)
    {
        printf("No valid IR calibration in EEPROM\r\n");
        return false;
    }
    if (!readCalibration(i2c, EEPROM_ADDR_IR_CAL, (uint8_t*)ir_thresh, 6 * sizeof(int16_t)))
    {
        printf("IR calibration EEPROM read FAILED\r\n");
        return false;
    }
    // printf("IR thresholds loaded from EEPROM: ");
    for (int i = 0; i < 6; i++)
        printf("%d ", ir_thresh[i]);
    printf("\r\n");
    return true;
}

////////////////////////////////////////////////TASKS////////////////////////////////////////////////////
void StartDefaultTask_run(void* arg)
{
    // im going to use this task for START/STOP button
    TickType_t last = xTaskGetTickCount();
    static uint8_t buf[40];
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));
        // motor_speeds(-100, -100);
        // vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));
        // motor_speeds(100, 100);
        // vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
        // for (int i = 0; i < 6; i += 2)
        // {
        //   ir_readings[i] = adc_dma_buffer[i / 2] & 0xFFFF;
        //   ir_readings[i + 1] = (adc_dma_buffer[i / 2] >> 16) & 0xFFFF;
        // }

        // int n = snprintf((char *)buf, sizeof buf, "%u %u %u %u %u %u \r\n",
        //                  ir_readings[0], ir_readings[1], ir_readings[2],
        //                  ir_readings[3], ir_readings[4], ir_readings[5]);
        // HAL_UART_Transmit(&huart2, buf, n, 20);
        // osDelay(20);
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // int n = snprintf((char *)buf, sizeof buf, "bro\r\n");
        // HAL_UART_Transmit(&huart2, buf, n, 20);

        if (pausedAndReset)
        {
            pausedAndReset = false;
            int n = snprintf((char*)buf, sizeof buf, "START\r\n");
            HAL_UART_Transmit(&huart2, buf, n, 20);
            // motor_speeds(100, 100);
            // TODO:start the run: signal algorithm task, or set motionType or whatever
        }
        else if (!pausedAndReset)
        {
            pausedAndReset = true;
            // STOP motors
            int n = snprintf((char*)buf, sizeof buf, "STOP\r\n");
            HAL_UART_Transmit(&huart2, buf, n, 20);
            // motor_speeds(0, 0);
            taskENTER_CRITICAL();
            motionType = MotionType::STOP;
            taskEXIT_CRITICAL();
            // TODO:reset position
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void bnoTask_run(void* arg)
{
    // Convention: CCW positive, 0 = starting heading, degrees in euler.x(), radians in
    // position.theta.
    //  turn left +ve, turn right -ve .....3,2,1,0,-1,-2,-3....
    TickType_t last = xTaskGetTickCount();

    imu bno(&hi2c2, 0x29); // TODO: check the address
    static uint8_t buf[64];
    int n;
    bno.init();

    double prevRawYaw = bno.euler().x();
    double yawOffset = 0;
    double yawJumpThresh = 50; // TODO: tune

    //TODO change to error codes on leds
    if (imu_utility::loadBnoCalibration(bno))
        printf("loaded bnoCalibration successfully\r\n");
    else
        printf("No calibration data to load\r\n");

    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
        if (calibrateBNO)
        {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            imu_utility::calibrateBnoAndSave(bno);
            calibrateBNO = false;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        }

        else
        {
            vec_3 v = bno.euler();
            vec_3 g = bno.gyro();
            if (fabs(imu_utility::angleDiff(prevRawYaw, v.x())) > yawJumpThresh)
            {
                yawOffset += imu_utility::angleDiff(v.x(), prevRawYaw);
                n = snprintf((char*)buf, sizeof buf, "-------Yaw jump detected!\r\n");
                HAL_UART_Transmit(&huart2, buf, n, 20);
            }

            prevRawYaw = v.x();

            float h = -(v.x() + yawOffset);
            while (h > 180.0f)
                h -= 360.0f;
            while (h < -180.0f)
                h += 360.0f;
            taskENTER_CRITICAL();
            euler = v;
            euler.x() = h;
            gyro = g;
            taskEXIT_CRITICAL();

            // n = snprintf((char *)buf, sizeof buf, "yaw:%.2f  offset:%.2f\r\n", euler.x(),
            // yawOffset); HAL_UART_Transmit(&huart2, buf, n, 20);
        }
    }
}

//TODO: was motion task run before
void ir_filter_task_run(void* arg)
{
    float irFiltered[6] = {}; //TODO should be moved to outer scope
    ButterworthIIR ir_filter[6] = {
        ButterworthIIR(10.0f, 1000.0f),
        ButterworthIIR(10.0f, 1000.0f),
        ButterworthIIR(10.0f, 1000.0f),
        ButterworthIIR(10.0f, 1000.0f),
        ButterworthIIR(10.0f, 1000.0f),
        ButterworthIIR(10.0f, 1000.0f)
    };

    //TODO replace with error code on leds
    // if (loadIRCalFromEEPROM(&hi2c1))
    //     // printf("loaded IR calibration\n");
    // else
    //     printf("no IR calibration in EEPROM\n");

    for (;;)
    {
        //notified from adc dma callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        std::memcpy(ir_readings, const_cast<const uint32_t*>(adc_dma_buffer), sizeof(ir_readings));

        taskENTER_CRITICAL(); // eliminate concurrent access
        for (int i = 0; i < 6; i++)
            irFiltered[i] = ir_filter[i].filter(ir_readings[i]);
        taskEXIT_CRITICAL();
    }
}

void controlTask_run(void* arg)
{
    TickType_t last = xTaskGetTickCount();
    double dt = 0.005;
    std::vector<Point> path = makeLine(0, 0, 2, 0);
    PDController headingHoldPD(0.003, 0.001, -0.12, 0.12); // TODO:tune kp,kd
    PDController lateralPD(0, 0, -10, 10); // TODO:tune kp,kd
    PDController turnInPlacePD(0, 0, -0.3,
                               0.3); // TODO: tune kp,kd i made the output ranges half max velocity
    float target_heading = 0; // read only the moment we lose wall reference
    float lateral_correction = 0.0f;

    //////////////////////////////////////////////////////shelnahom 3ashan teseting headingpd
    // int last_mode = -1; // to detect change in motion type
    // MotionType lastMotionTypeCtrl = STOP;
    // uint32_t last_cmd_id = 0; // b3rad el command id 3ashan ne know law command gdeda w nreset el
    // controllers bool status_sent = false; // 3shan ne send status only once when motion is done

    Pose straight_start_pose;
    float straight_target_dist = 0.2f; // cell in meters

    std::vector<Point> path_copy = makeLine(0.0, 0.15, 2.0, 0.15);

    // uint32_t local_ver = 0;

    // purePursuit.reset();
    // ###############################################################################
    // encoders and ffpi stuff
    constexpr float meters_per_encoder_tick = M_PI * WHEEL_DIAMETER /
        PULSES_PER_WHEEL_REVOLUTION; // meter of travel per encoder tick

    static int32_t totalL = 0;
    static int32_t totalR = 0;
    static int32_t histL[N_WIN + 1] = {0};
    static uint8_t hidxL = 0;
    ButterworthIIR left_encoder_iir(10, 200.0f);

    static int32_t histR[N_WIN + 1] = {0};
    static uint8_t hidxR = 0;
    ButterworthIIR right_encoder_iir(20, 200.0f);

    static uint8_t buf[64];
    int n;

    target_heading = 0.0f;
    walls[1] = walls[2] = walls[0] = 0;

    MotionType current_motion = MotionType::TURN; // added when testing purepursuit

    float velRwinFiltered;
    float velLwinFiltered;

    left_motor.move(0.0f);
    right_motor.move(0.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));

    position.x = 0;
    position.y = 0;
    leftCtrl.reset();
    rightCtrl.reset();
    straight_start_pose = position;
    double dist_traveled;
    vTaskDelay(1000);


    for (;;)
    {
        printf("%lu\n\r", millis());
        /// TODO: notified from where??
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));

        // ovf logic handled inside class, gets delta directly
        int32_t right_encoder_delta = right_motor.encoder().delta();
        int32_t left_encoder_delta = left_motor.encoder().delta();

        auto raw_vel_left = static_cast<float>(static_cast<float>(left_encoder_delta) * meters_per_encoder_tick / ENCODER_TASK_DT_S);
        auto raw_vel_right = static_cast<float>(static_cast<float>(right_encoder_delta) * meters_per_encoder_tick / ENCODER_TASK_DT_S);
        velLwinFiltered = left_encoder_iir.filter(raw_vel_left); // change to raw vell for iir tuning
        velRwinFiltered = right_encoder_iir.filter(raw_vel_right);




        // ##############TUNING FFFPI################################ss
        float left_cmd = leftCtrl.compute(wheel_ref.left, velLwinFiltered, dt);
        float right_cmd = rightCtrl.compute(wheel_ref.right, velRwinFiltered, dt);
        right_motor.move(right_cmd);
        left_motor.move(left_cmd);

        // n = snprintf((char *)buf, sizeof(buf), "%ld,%f,%f\r\n",
        //              (unsigned long)millis(),
        //              (left_cmd), (right_cmd));
        // HAL_UART_Transmit(&huart2, buf, n, 10);

        float v = (velLwinFiltered + velRwinFiltered) * 0.5f; // m/s, forward speed
        float w =
            (velRwinFiltered - velLwinFiltered) / WHEEL_BASE; // rad/s, positive = turning left
        // // TODO: et2akedy men dool
        float distance_center =
            ((left_encoder_delta * meters_per_encoder_tick) + (right_encoder_delta * meters_per_encoder_tick)) * 0.5f;

        // /*straight line: PD Controller + IR centering + longitudinal correction with diagonal IRs
        //   turns: Pure pursuit only, corrected using gyro. there is no IR correction in turns

        //   error generated from purepursuit is corrected by the IRs later
        //   the error should be negligible for one turn
        // */
        taskENTER_CRITICAL();
        // MotionType current_motion = motionType;
        // uint32_t cmd = cmd_id;
        float heading = euler.x(); // degree
        taskEXIT_CRITICAL();

        // if (motionType != lastMotionTypeMotion || cmd != last_cmd_id)
        // {
        //   // do i reset these? ana mayla le both reset or not reset so idk
        //   leftCtrl.reset();
        //   rightCtrl.reset();
        //   lastMotionTypeMotion = current_motion;
        //   last_cmd_id = cmd;
        // }
        // wheelVelocity wheel_speed = wheel_ref;
        // float left_cmd = 0.0f;
        // float right_cmd = 0.0f;

        // if (current_motion != STOP)
        // {
        //   left_cmd = leftCtrl.compute(wheel_speed.left, velLfiltered, dt);
        //   right_cmd = rightCtrl.compute(wheel_speed.right, velRfiltered, dt);
        // }
        // motor_speeds(left_cmd, right_cmd);


        // n = snprintf((char *)buf, sizeof(buf), "%f,%f, %f\r\n",
        //              left_cmd, right_cmd, euler.x());
        // HAL_UART_Transmit(&huart2, buf, n, 10);
        // // update global
        // // 2 critical blocks 3ashan mesh taba3 ba3d w law 3ayez ye3mel interrupt mabenhom no
        // problem
        taskENTER_CRITICAL();
        position.theta = (heading * (M_PI / 180.0)); // wont calculate angle from encoders
        position.x += distance_center * cos(position.theta);
        position.y += distance_center * sin(position.theta);
        taskEXIT_CRITICAL();

        // n = snprintf((char *)buf, sizeof(buf), "%f %f %f\r\n",
        //              dist_traveled,
        //              position.x,
        //              position.y);
        // HAL_UART_Transmit(&huart2, buf, n, 10);
        // taskENTER_CRITICAL();
        robot_velocity.omega = w;
        robot_velocity.v = v;
        robot_velocity.vL = velLwinFiltered;
        robot_velocity.vR = velRwinFiltered;
        // taskEXIT_CRITICAL();

        // #######################################################################
        // if (path_version != local_ver)
        // {
        //   xSemaphoreTake(pathMutex, portMAX_DELAY);
        //   path_copy = current_path;
        //   local_ver = path_version;
        //   xSemaphoreGive(pathMutex);
        // }
        taskENTER_CRITICAL();
        Pose current_pose = position;
        double v_measured = robot_velocity.v;
        double omega_measured = robot_velocity.omega;
        // double left_distance = ir_distance[2];
        // double right_distance = ir_distance[3];
        // double front_distance = (ir_distance[0] + ir_distance[1]) * 0.5;
        double current_target_v = target_v;
        uint32_t cmd = cmd_id;
        // float heading = euler.x();  //heading_sign *euler.x(); //degree ccw positive
        bool wall_front = walls[0];
        bool wall_left = walls[1];
        bool wall_right = walls[2];
        taskEXIT_CRITICAL();

        // if (motionType != lastMotionTypeCtrl || cmd != last_cmd_id)
        // {
        //   headingHoldPD.reset();
        //   lateralPD.reset();
        //   turnInPlacePD.reset();
        //   purePursuit.reset();
        //   lastMotionTypeCtrl = current_motion;
        //   last_cmd_id = cmd;
        //   status_sent = false; // b nreeset el flag
        //   last_mode = -1;      // reset last mode to force re-evaluation of motion type

        //   straight_start_pose = current_pose; // 3ashan a3raf el distance el mashaha fy STRAIGHT
        //   segment
        // }
        // emergency stop if front wall detected & 🛺 lsa mkml staright
        // if (current_motion == STRAIGHT && (wall_front))
        // { // TODO: i think en hena wall_front is not correct 3ashan momken yeb2a shayef front
        // wall bas mashy sa7
        //   // lesa mesh hayekhbat ya3ny fa IMPORTANT

        //   taskENTER_CRITICAL();
        //   wheel_ref.left = 0.0;
        //   wheel_ref.right = 0.0;
        //   motionType = STOP;
        //   taskEXIT_CRITICAL();

        //   if (!status_sent)
        //   {
        //     MotionStatus_t status = {0};
        //     xQueueSend(motionStatusQueue, &status, 0);
        //     status_sent = true;
        //   }
        //   continue;
        // }

        if (current_motion == MotionType::FORWARD)
        {
            // TODO: wrap reading the walls in CRITICAL section
            double base_v = 0.5; // current_target_v;
            float lateral_error = 0.0f;
            if (wall_left && wall_right)
            { // 2 side walls
                // ir_thresh[2]-ir_thresh[3] offset between 2 sensors readings
                // wont be entirely accurate law bo3ad 3an ba3d awy bas insha2allah accurate law el
                // robot roughly fy nos el cell
                lateral_error = ir_readings[2] - (ir_thresh[2] - ir_thresh[3]) - ir_readings[3];
                // last_mode = 0; // STRAIGHT
                lateral_correction = lateralPD.compute(0.0f, lateral_error, dt);
            }
            else if (wall_left)
            { // only left wall
                lateral_error = ir_readings[2] - ir_thresh[2];
                // last_mode = 1; // LEFT_WALL
                lateral_correction = lateralPD.compute(0.0f, lateral_error, dt);
            }
            else if (wall_right)
            { // only right wall
                lateral_error = ir_thresh[3] - ir_readings[3];
                // last_mode = 2; // RIGHT_WALL
                lateral_correction = lateralPD.compute(0.0f, lateral_error, dt);
            }

            else
            { // no side walls then hold current heading //dont know law dah momken ye7sal aslan bas
              // better safe
                // if (last_mode == -1)
                // {
                //   taskENTER_CRITICAL();
                //   target_heading = euler.x();
                //   // last_mode = 3; // NO_WALLS
                //   taskEXIT_CRITICAL();
                // }
                taskENTER_CRITICAL();
                float heading_error = imu_utility::wrapAngle(target_heading - heading);
                taskEXIT_CRITICAL();
                lateral_correction = headingHoldPD.compute(0.0f, heading_error, dt);
            }
            // TODO :
            taskENTER_CRITICAL();
            wheel_ref.left = base_v + lateral_correction;
            wheel_ref.right = base_v - lateral_correction;
            taskEXIT_CRITICAL();

            dist_traveled = std::hypot(current_pose.x - straight_start_pose.x,
                                       current_pose.y - straight_start_pose.y);
            if (dist_traveled >= 0.8 - 0.1 /*&& !status_sent && motionStatusQueue != NULL*/)
            {
                taskENTER_CRITICAL();
                wheel_ref.left = 0.0;
                wheel_ref.right = 0.0;
                motionType = MotionType::STOP;
                taskEXIT_CRITICAL();
                right_motor.move(0.0f);
                left_motor.move(0.0f);
                // vTaskDelay(pdMS_TO_TICKS(2000));

                MotionStatus_t status = {false};
                // xQueueSend(motionStatusQueue, &status, 0);
                // status_sent = true;
            }
        }

        // to rotate set motiontype and also set target_heading
        else if (current_motion == MotionType::TURN_IN_PLACE)
        {
            taskENTER_CRITICAL();
            float heading_error = imu_utility::wrapAngle(target_heading - euler.x());
            taskEXIT_CRITICAL();
            lateral_correction = turnInPlacePD.compute(0.0f, heading_error, dt);

            wheel_ref.left = lateral_correction;
            wheel_ref.right = -lateral_correction;

            if (heading_error < 2.0)
            {
                // stop?
                wheel_ref.left = 0;
                wheel_ref.right = 0;
                // TODO
                // change motionType?
                // Signal the algorithm
            }
        }

        else if (current_motion == MotionType::TURN)
        {
            wheelVelocity wheel_speed = purePursuit.computeControl(
                current_pose, v_measured, omega_measured, current_target_v, path_copy, dt);
            taskENTER_CRITICAL();
            wheel_ref = wheel_speed;
            taskEXIT_CRITICAL();

            // n = snprintf((char *)buf, sizeof(buf), "%f %f %f\r\n",
            //              position.x,
            //              position.y,
            //              position.theta * 180.0f / M_PI);
            // HAL_UART_Transmit(&huart2, buf, n, 10);
            if (!path_copy.empty() /*&& !status_sent && motionStatusQueue != NULL*/)
            {

                const Point& goal = path_copy.back();
                // double dist_to_goal = std::hypot(goal.x - current_pose.x, goal.y -
                // current_pose.y);

                double gx = goal.x - current_pose.x;
                double gy = goal.y - current_pose.y;
                double dist_to_goal = std::hypot(gx, gy);

                // positive while the goal is still ahead of the robot's heading
                double forward =
                    gx * std::cos(current_pose.theta) + gy * std::sin(current_pose.theta);

                if (dist_to_goal < 0.05 || forward < 0.0 || current_pose.x >= goal.x)
                { // threshold to consider the turn complete
                    MotionStatus_t status = {false}; // done
                    if (motionStatusQueue != NULL)
                    {
                        MotionStatus_t status = {false};
                        xQueueSend(motionStatusQueue, &status, 0);
                        // status_sent = true;
                    }

                    right_motor.move(0.00f);
                    left_motor.move(0.0f);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

                    n = snprintf((char*)buf, sizeof(buf), "done here\r\n");
                    HAL_UART_Transmit(&huart2, buf, n, 10);

                    // TODO: is this correct?
                    current_motion = MotionType::STOP;
                    wheel_ref.left = wheel_ref.right = 0;

                    // vTaskDelay(pdMS_TO_TICKS(20000));
                }
            }
        }
        // else if (current_motion == MotionType::TESTING_WHEEL_SPEEDS)
        // {
        //   taskENTER_CRITICAL();
        //   wheel_ref.left = current_target_v;
        //   wheel_ref.right = current_target_v;
        //   taskEXIT_CRITICAL();
        // }
        else if (current_motion == MotionType::STOP)
        {
            // TODO: need to make a case for STOP
            taskENTER_CRITICAL();
            wheel_ref.left = 0;
            wheel_ref.right = 0;
            taskEXIT_CRITICAL();
            ////kda mafrood el el algo ykteen el current_path f el pathMutex w b3den path ver++
            /// w yzwed el cmd_id f col 2mr gded 7ta lw nafs el motion type 3shan a3rf a3ml reset w
            /// el status_sent flag
        }
    }
}

void algorithmTask_run(void* arg)
{ // have to check pausedAndReset bool before pushing any new command
    MotionStatus_t status;
    for (;;)
    {
        if (xQueueReceive(motionStatusQueue, &status, portMAX_DELAY) == pdTRUE)
        {
            printf("in algorithm task...\n");
        }
    }
}

void HMIConfigTask_run(void* arg)
{
    static uint8_t buf[64];
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (calibrateIR)
        {
            int n = snprintf((char*)buf, sizeof buf, "calibrate IR\r\n");
            HAL_UART_Transmit(&huart2, buf, n, 20);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
            IRCalibration();
            if (saveIRCalToEEPROM(&hi2c1, ir_thresh))
            {
                n = snprintf((char*)buf, sizeof buf,
                             "calibration saved to EEPROM:)\r\n %d %d %d %d %d %d", ir_thresh[0],
                             ir_thresh[1], ir_thresh[2], ir_thresh[3], ir_thresh[4], ir_thresh[5]);
                HAL_UART_Transmit(&huart2, buf, n, 20);
            }
            else
            {
                n = snprintf((char*)buf, sizeof buf,
                             "failed to save IRCalibration to EEPROM:(\r\n");
                HAL_UART_Transmit(&huart2, buf, n, 20);
            }

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            calibrateIR = false;
        }
    }
}

void loggerTask_run(void* arg)
{
    for (;;)
    {

    }
    osDelay(100);
}
//////////////////////////////////////////////////END
/// TASKS//////////////////////////////////////////////
void app_main()
{
    init_ir();
    right_motor.init();
    left_motor.init();
    right_motor.encoder().init();
    left_motor.encoder().init();

    HAL_TIM_Base_Start(&htim5); // millis or micros?


}

void init_ir()
{
    // set the timer to be at ARR 3ashan lama tim4 ye trigger it yebda2 men zero
    // probably wont matter if we set it at zero 3ady
    HAL_TIM_DMABurst_MultiWriteStart(&htim8, TIM_DMABASE_CCR1, TIM_DMA_CC4,
                                     (const uint32_t*)ir_sequence, TIM_DMABURSTLENGTH_3TRANSFERS,
                                     9);

    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 3);

    // ir trig/adc conversion
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    // master timer for ir
    HAL_TIM_Base_Start(&htim4);
}
