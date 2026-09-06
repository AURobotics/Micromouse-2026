#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

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
std::vector<Point> current_path; // pure pursuit reads this & algorithm writes this


void StartDefaultTask_run(void *arg);
void bnoTask_run(void *arg);
void motionTask_run(void *arg);
void controlTask_run(void *arg);
void algorithmTask_run(void *arg);
void HMIConfigTask_run(void *arg);
void loggerTask_run(void* arg);
void app_main(); // This is your C++ entry function

#ifdef __cplusplus
}
#endif

#endif