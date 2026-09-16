#include <Arduino.h>
#include <Wire.h>
#include <BNO055.h>
#include <RotaryEncoderPCNT.h>
#include <EEPROM.h>
#include "esp_adc/adc_continuous.h"
#include "esp_system.h"
#include "ekf.h"
#undef F

/*not sure how buttons will work yet so for now hastakhdem changePin to toggle wifi*/
int modeByte = 2;      //byte where the mode will be stored
int changePin = 11;    //button that will trigger menu
int selectorPin = 12;  //button that will be used to select mode floodfill,right,left
volatile bool menu = false;
volatile char option;
unsigned long interTimer;

// inline void getPosition();  // odom
inline float getOrientationX();
inline float getRate();
bool calibrateBnoAndSave(imu& bno);
bool loadBnoCalibration(imu& bno);

bool moveF(double tiles);
void turn(double angle);
bool wallLeft();
bool wallRight();
bool wallFront();
bool frontEmergency();

double angleDiff(double start, double goal);
double fixSpeed(double speed);
inline float getLin();
inline double calculateDistance(double x, double y);
inline double ekfCalculateDistance(double x, double y);


#define MAX_H 18 //18
#define MAX_W 18 //18
#define QUEUE_MAX (MAX_H * MAX_W)

// struct queue;
using queue = struct queue;

struct queue {
  short head, tail, size, counter;
  char items[MAX_H * MAX_W];
};

char curr_dir = 0;  // 0--> North, 1 --> East, 2 --> South, 3 --> West
char curr_r = 6, curr_c = 1;//TODO: change this according to where you start

int current_run;
int previous_run;

bool maze[MAX_H][MAX_W][5] = { 0 };  // represents the maze, first 4 bits represent the walls N E S W, the last bit represents the visiting status
//leh mn3melsh byte/char maze[MAX_H][MAX_W] ?

short dis[MAX_H][MAX_W] = {
  { 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16 },
  { 15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  { 14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
  { 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13 },
  { 12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12 },
  { 11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
  { 10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10 },
  { 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
  { 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8 },
  { 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8 },
  { 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
  { 10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10 },
  { 11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
  { 12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12 },
  { 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13 },
  { 14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
  { 15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  { 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16 }
};

// short dis[MAX_H][MAX_W]= {
//   {4,3,2,2,3,4},
//   {3,2,1,1,2,3},
//   {2,1,0,0,1,2},
//   {2,1,0,0,1,2},
//   {3,2,1,1,2,3},
//   {4,3,2,2,3,4}
// };
queue r_q;
queue c_q;

//TODO:
#define ticksperlafa 1400
#define circumference 10.681
#define distance_between_wheels 9
// #define PI 3.141592653589

// right motor pins 23 22
#define rightMotorForward 23
#define rightMotorBackward 22
RotaryEncoderPCNT rightEncoder(15, 6);
double previousRight;

// left motor pins 25 24
#define leftMotorForward 25
#define leftMotorBackward 24
RotaryEncoderPCNT leftEncoder(26, 27);  // 8 7
double previousLeft;


int left_revolutions, prev_left_revolutions;
int right_revolutions, prev_right_revolutions;


double xPosition = 0, yPosition = 0;
double yaw = 0;
double yawOffset = 0;
inline float getRawYaw();
//###################################BNO######################################
//TODO:
#define EEPROM_SIZE        32  //need more for saving the map          
#define CALIB_FLAG_ADDR    4           //after modebyte
#define CALIB_DATA_ADDR    5           // actual calibProfile struct starts here 22 bytes
#define CALIB_MAGIC        0x42        // if found then data is valid

#define SCL_PIN            13       
#define SDA_PIN            14
#define yawJumpThresh  10         //--------------------------------------------------------------------------------------------->need to set this
portMUX_TYPE yawMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t bnoOffsetTaskHandle = NULL;
SemaphoreHandle_t bnoMutex;

imu bno(SCL_PIN,SDA_PIN,I2C_NUM_0, 0x29);

constexpr uint8_t ADC1_FRONT = 2;
constexpr uint8_t ADC2_RIGHT = 3;
constexpr uint8_t ADC1_LEFT = 4;

//TODO
int frontThresh = 100;
int leftThresh = 100;
int rightThresh = 100;   
int SAMPLES_PER_BURST = 32; 

struct IR {
  uint8_t trig_pin;
  uint8_t echo_pin;
};

IR front_ir = { 35, ADC1_FRONT };
IR left_ir = { 34, ADC1_LEFT };
IR right_ir = { 36, ADC2_RIGHT};

std::array ir_array = { front_ir, left_ir, right_ir};
int readings[3];

int currentSensor = 0;
bool litPhase = false;
int32_t darkVal = 0;

adc_continuous_handle_t adcHandle = NULL;
TaskHandle_t irTaskHandle = NULL;
SemaphoreHandle_t readingsMutex = NULL;

String tostr(short x) {
  //log("tostr");
  if (x == 0) return "0";
  String ret = "";
  while (x) {
    ret = char((x % 10) + '0') + ret;
    x /= 10;
  }
  return ret;
}

// queue implementation

void initialise(queue &q, short size) {
  q.head = 0;
  q.tail = 0;
  q.size = size;
  q.counter = 0;
}
bool isfull(queue &q) {
  if (q.counter == q.size)
    return (1);
  else
    return (0);
}
bool isempty(queue &q) {
  if (q.counter == 0)
    return (1);
  else
    return (0);
}

void enqueue(queue &q, char value) {
  if (!isfull(q)) {
    q.items[q.tail] = value;
    q.tail = (q.tail + 1) % q.size;
    q.counter++;
  }
}
char dequeue(queue &q) {
  if (!isempty(q)) {
    char result;
    result = q.items[q.head];
    q.head = (q.head + 1) % q.size;
    q.counter--;
    return result;
  }
  else return 0; //not sure law dah momken yebawaz logic el code bas it throws an error without it (reaches end of non-void function)
}

//end of queue implementation

// change r, c to move to: N, E, S, W
signed char r_mov[4] = { -1, 0, 1, 0 };
signed char c_mov[4] = { 0, 1, 0, -1 };

bool isValid(char r, char c) {
  return ((r >= 0) && (r < MAX_H)) && ((c >= 0) && (c < MAX_W));
}

bool isAccessible(char r, char c, int dir) {
  return !(maze[r][c][dir] || maze[r + r_mov[dir]][c + c_mov[dir]][(dir + 2) % 4]);
}


void flood(bool goal = 1) {  // make goal = 0 to change the goal to the start
  Serial.println("starting flood");
  for (char i = 0; i < MAX_W; i++) {  //initialize all cells with -1
    for (char j = 0; j < MAX_H; j++) {
      dis[i][j] = -1;
    }
  }

  if (goal) {
    // for (char x = 12; x < 14; x++) {  //change middle cells with 0
    //   for (char w = 4; w < 6; w++) {
    //     dis[x][w] = 0;
    //     enqueue(r_q, x);
    //     enqueue(c_q, w);
    //   }
    // }
    // dis[14][3] = 0;
    // enqueue(r_q, 14);
    // enqueue(c_q, 3);
    for (char x = MAX_W / 2 - 1; x < MAX_W / 2 + 1; x++)
    { // change middle cells with 0
        for (char w = MAX_H / 2 - 1; w < MAX_H / 2 + 1; w++)
        {
            dis[x][w] = 0;
            // r_q.push(x);
            // c_q.push(w);
            enqueue(r_q, x);
            enqueue(c_q, w);
        }
    }

  } else {
    dis[16][1] = 0;
    enqueue(r_q, 16);
    enqueue(c_q, 1);
  }

  while (!isempty(c_q) && !isempty(r_q)) {
    char r = dequeue(r_q);
    char col = dequeue(c_q);
    Serial.println("flooding from " + String((int)r) + " " + String((int)col));
    for(int i=0;i<4;i++){
      Serial.println(maze[r][col][i]);
      Serial.print(" ");
    }
    Serial.println();
    for (int i = 0; i < 4; i++) {
      Serial.println(String(isValid(r + r_mov[i], col + c_mov[i]))  + " " + String(isAccessible(r, col, i)) + " " +dis[r + r_mov[i]][col + c_mov[i]] );
      if (isValid(r + r_mov[i], col + c_mov[i]) && isAccessible(r, col, i) && dis[r + r_mov[i]][col + c_mov[i]] == -1) {
        Serial.println("enqueuing " + String((int)(r + r_mov[i])) + " " + String((int)(col + c_mov[i])) );
        dis[r + r_mov[i]][col + c_mov[i]] = dis[r][col] + 1;
        enqueue(r_q, r + r_mov[i]);
        enqueue(c_q, col + c_mov[i]);
      }
    }
  }
  
}


bool moveTo(char r, char c) {
  // get where I want to move relative to abolute direction (y3ny lw el robot bases north) ana lesa m2alef el term dah
  short dir;
  if (r < curr_r) dir = 0;
  if (r > curr_r) dir = 2;
  if (c < curr_c) dir = 3;
  if (c > curr_c) dir = 1;
  // ////Serial.println(String((int) r)+" "+ String((int) c));
  //log(String("direction: ") + tostr(dir));

  // compare the movement direction to the current directoin to know how should I turn

  if (dir - curr_dir == -1 || dir - curr_dir == 3)  // turn left
  {
    Serial.println("turning left");
    turn(-90);  //turnLeft();
    delay(100);
    //moveF(1);       //moveForward();
    curr_dir += 3;  // b3mel +3 msh -1 because el negative numbers don't work/work differently fel mod//
    curr_dir %= 4;
    Serial.println("moving forward");
    if (!moveF(1)) return 0; 
  }

  else if (dir - curr_dir == 1 || dir - curr_dir == -3)  // turn right
  {
    Serial.println("turning right");
    turn(90);  //turnRight();
    delay(100);
    curr_dir++;
    curr_dir %= 4;
    Serial.println("moving forward");
    if (!moveF(1)) return 0;  //moveForward(); //this function returns 0 if it was unable to move,so we leave the func, 3shan man8ayarsh el curr c wel curr r

  } 
  else if (dir == curr_dir)  // move forward
  {
    Serial.println("moving forward");
    if (!moveF(1)) return 0;  //moveForward();
  } 
  else                    // turn 180
  {
    //turnRight();
    Serial.println("turning 180");
    turn(180);  //turnRight();
    delay(100);
    curr_dir += 2;
    curr_dir %= 4;
    Serial.println("moving forward");
    if (!moveF(1)) return 0;  //moveForward();
  }

  curr_dir %= 4;
  curr_c = c;
  curr_r = r;
  //delay(100);
  return 1;
}
bool motionSuccessful = 0;
int flooded = 0;
void exploreToCenter() {
  motionSuccessful = 1;
  flooded = 0;
  while(!(((curr_r == MAX_H / 2 - 1) || (curr_r == MAX_H / 2)) && ((curr_c == MAX_W / 2 - 1) || (curr_c == MAX_W / 2)))){
  //while (!(((curr_r == 12) || (curr_r == 13)) && ((curr_c == 4) || (curr_c == 5))) && !menu ) {
    //while(!(curr_r == 14 && curr_c == 3) && !menu){
    Serial.println("Exploring to center" + String((int)curr_c )+" "+String((int)curr_r) + String((int)curr_dir));
    
    //log("start: " + tostr(dis[curr_r][curr_c]));
    //log(String((int)curr_r)+" "+String((int)curr_c )+" "+String((int)curr_dir));
    if(!maze[curr_r][curr_c][4] || !motionSuccessful){
      //motionSuccessful = 1;
      bool walls[4];
      walls[0] = wallFront();
      walls[1] = wallRight();
      //walls [2] = wallBack(); wall back isn't available();
      walls[3] = wallLeft();
      if (curr_r == 16 && curr_c == 1 && curr_dir == 0) walls[2] = 1;
      else walls[2] = 0;
      Serial.println("done reading the walls");
      //for(int i=0;i<4;i++) std::cerr<<walls[i]<<" ";
      //std::cerr<<std::endl;
      // dataTosend = String(wallLeft()) + " " + String(wallFront()) + " " + String(wallRight()) + "\n" ;
      //Serial.println("wussup");
      //Serial.print((int)curr_c);
      //Serial.print(" ");
      //Serial.println((int)curr_r);
      //Serial.print(wallLeft());
      //Serial.print(" ");
      //Serial.print(wallFront());
      //Serial.print(" ");
      //Serial.println(wallRight());

      char d = curr_dir, w = 0;
      do {
        if(!(w == 2 && walls[w] == 0)){
          maze[curr_r][curr_c][d] = walls[w];
          maze[curr_r + r_mov[d]][curr_c + c_mov[d]][(d + 2) % 4] = walls[w]; // set the wall for the neighbouring cell too
        }
        d = (d + 1) % 4;
        w++;
      } while (d != curr_dir);
    }
    maze[curr_r][curr_c][4] = 1;
    //set_wall();
    char next_r = curr_r, next_c = curr_c;

    for (char i = 0; i < 4; i++) {
      if (isValid(curr_r + r_mov[i], curr_c + c_mov[i]) && isAccessible(curr_r, curr_c, i) && dis[curr_r + r_mov[i]][curr_c + c_mov[i]] < dis[next_r][next_c]) {
        next_r = curr_r + r_mov[i];
        next_c = curr_c + c_mov[i];
      }
    }
    //log("to: " + tostr(dis[next_r][next_c]));

    if ((next_r == curr_r) && (next_c == curr_c)){ // you re-flood when you can't find a place to go
      Serial.println("next == curr flood");        // fa if you re-flood more than once, then the ir readings are most probablly wrong, fa sent mostionSuccessful to 0 to retake them
      flood();
      Serial.println("done the next == curr flood");
      flooded ++;
      if(flooded > 1)
      {
        motionSuccessful = 0;
        flooded = 0;
      }
    }
    else
    { 
    // while (!Serial.available());
    // Serial.read();
    // Serial.flush();
    flooded = 0;
    Serial.println("Start moving");
    motionSuccessful = moveTo(next_r, next_c); // if failed, i want it to retake the ir readings
    Serial.println("done moving");
    }
    
  }

  return;
}

void exploreToStart() {
  motionSuccessful = 1;
  flooded = 0;
  while (!(curr_c == 1 && curr_r == 16) && !menu) {
    Serial.println("Exploring to start" + String((int)curr_c )+" "+String((int)curr_r) + String((int)curr_dir));
    //log("start: " + tostr(dis[curr_r][curr_c]));
    //log(String((int)curr_r)+" "+String((int)curr_c )+" "+String((int)curr_dir));
    if(!maze[curr_r][curr_c][4] || !motionSuccessful){
      bool walls[4];
      walls[0] = wallFront();
      walls[1] = wallRight();
      //walls [2] = wallBack(); wall back isn't working
      walls[3] = wallLeft();
      if (curr_r == 16 && curr_c == 1 && curr_dir == 0) walls[2] = 1;
      else walls[2] = 0;
      Serial.println("done reading the walls");
      //for(int i=0;i<4;i++) std::cerr<<walls[i]<<" ";
      //std::cerr<<std::endl;



      char d = curr_dir, w = 0;
      do {
        if(!(w == 2 && walls[w] == 0)){
          maze[curr_r][curr_c][d] = walls[w];
          maze[curr_r + r_mov[d]][curr_c + c_mov[d]][(d + 2) % 4] = walls[w];
        }
        d = (d + 1) % 4;
        w++;
      } while (d != curr_dir);
    }
    maze[curr_r][curr_c][4] = 1;
    //set_wall();
    char next_r = curr_r, next_c = curr_c;

    for (char i = 0; i < 4; i++) {
      if (isValid(curr_r + r_mov[i], curr_c + c_mov[i]) && isAccessible(curr_r, curr_c, i) && dis[curr_r + r_mov[i]][curr_c + c_mov[i]] < dis[next_r][next_c]) {
        next_r = curr_r + r_mov[i];
        next_c = curr_c + c_mov[i];
      }
    }
    // log("to: " + tostr(dis[next_r][next_c]));

    if ((next_r == curr_r) && (next_c == curr_c)){
      Serial.println("next == curr flood");
      flood(0);
      Serial.println("done the next == curr flood");
      flooded ++;
      if(flooded > 1)
      {
        motionSuccessful = 0;
        flooded = 0;
      }
    }
    else{
      flooded = 0;
      Serial.println("starting motion");
      motionSuccessful = moveTo(next_r, next_c);
      Serial.println("done motion");
    }
  }

  return;
}

int theoreticalHeading = 0;

float calDistances[6] = { 4, 6, 8, 10, 12, 15 };   // cm values ana mekhtaraha 
int   calReadings[3][6] = { 0}; //TODO
//values tal3a men calibration // 3amalt calibration then hardcoded them 3ashan probably mesh hayet8ayaro

/*this part 3ashan ne2dar ne7seb el distance 3ashan ne2dar ne3del el heading using el walls
and also 3ashan ne fuse the data bardo fy el kalman filter ma3 el encoders and gyro*/

void IRCalibration(int sensor) {
  Serial.print("IR calibration("); Serial.print(sensor); Serial.println("): position robot");

  for (int i = 0; i < 6; i++) {
    Serial.print("Move "); Serial.print(sensor); Serial.print(" to ");
    Serial.print(calDistances[i]);
    Serial.println(" cm from wall, then press any key + enter");

    while (!Serial.available());
    while (Serial.available()) Serial.read();  // clear buffer

    long sum = 0;
    for (int s = 0; s < 20; s++) {
      sum += readings[sensor];
      delay(20);
    }

    calReadings[sensor][i] = sum / 20;
  }

  Serial.print(" calReadings["); Serial.print(sensor); Serial.print("] = ");
  for (int i = 0; i < 6; i++) {
    Serial.print(calReadings[sensor][i]); Serial.print(", ");
  }
  Serial.println();

  Serial.print(sensor);
  Serial.println(" calibration done. Copy calReadings[] values into your code");
}

float irToDist(int reading,int sensor) {
  // linear interpolation between calibration points
  if (reading >= calReadings[sensor][0]) return calDistances[0]; 
  for (int i = 0; i < 5; i++) {
    int r0 = calReadings[sensor][i], r1 = calReadings[sensor][i+1];
    if (reading <= r0 && reading >= r1) {
      float m = (float)(r0 - reading) / (float)(r0 - r1);
      return calDistances[i] + m * (calDistances[i+1] - calDistances[i]);
    }
  }

  //beyond last point not accurate bas can tell us law odamna kaza cell fadya masalan...not sure yet
  int i = 4;
  float slope = (calDistances[i+1] - calDistances[i]) / (float)(calReadings[sensor][i+1] - calReadings[sensor][i]);//used same slope as last point fa not accurate
  float extrapolated = calDistances[5] + slope * (reading - calReadings[sensor][5]);
  return max(0.0f, min(200.0f, extrapolated)); 
}

int32_t readOneBurst() {
  adc_continuous_start(adcHandle);
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)); 

  uint8_t buf[SOC_ADC_DIGI_RESULT_BYTES * SAMPLES_PER_BURST];
  uint32_t outLen = 0;
  esp_err_t readErr = adc_continuous_read(adcHandle, buf, sizeof(buf), &outLen, 0);

  int32_t sum = 0;
  int count = 0;
  if (readErr == ESP_OK) {
    for (uint32_t i = 0; i < outLen; i += SOC_ADC_DIGI_RESULT_BYTES) {
      sum += ((adc_digi_output_data_t*)&buf[i])->type2.data;
      count++;
    }
  }
  adc_continuous_stop(adcHandle);
  return (count > 0) ? (sum / count) : 0;
}

static bool IRAM_ATTR onConvDone(adc_continuous_handle_t h, const adc_continuous_evt_data_t *e, void *arg) {
  BaseType_t mustYield = pdFALSE;
  vTaskNotifyGiveFromISR(irTaskHandle, &mustYield);
  return mustYield == pdTRUE;
}

void configureChannel(uint8_t echo_pin) {
  adc_digi_pattern_config_t pattern[1] = {};
  pattern[0].atten = ADC_ATTEN_DB_12;
  pattern[0].channel = (adc_channel_t)(echo_pin - 1);
  pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t cfg = {};
  cfg.sample_freq_hz = 80000;
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  cfg.pattern_num = 1;
  cfg.adc_pattern = pattern;
  adc_continuous_config(adcHandle, &cfg);
}

void irTask(void *pv) {
  while(1){
    IR &s = ir_array[currentSensor];

    if (!litPhase) {
      configureChannel(s.echo_pin);      
      digitalWrite(s.trig_pin, LOW);
      darkVal = readOneBurst();
      digitalWrite(s.trig_pin, HIGH);
      litPhase = true;
    } else {
      int32_t litVal = readOneBurst();
      digitalWrite(s.trig_pin, LOW);

      xSemaphoreTake(readingsMutex, portMAX_DELAY);
      readings[currentSensor] = litVal - darkVal;
      xSemaphoreGive(readingsMutex);

      currentSensor = (currentSensor + 1) % ir_array.size();
      litPhase = false;
    }
    vTaskDelay(1);
  }
}




void setupIR() {
  for (const auto &[trig_pin, echo_pin] : ir_array) {
    pinMode(trig_pin, OUTPUT);
    digitalWrite(trig_pin, LOW);
    pinMode(echo_pin,INPUT);
  }
  adc_continuous_handle_cfg_t adc_config = {};
  adc_config.max_store_buf_size = SAMPLES_PER_BURST * SOC_ADC_DIGI_RESULT_BYTES * 4;
  adc_config.conv_frame_size = SAMPLES_PER_BURST * SOC_ADC_DIGI_RESULT_BYTES;
  Serial.printf("new_handle: %d\n", adc_continuous_new_handle(&adc_config, &adcHandle));
  
  adc_continuous_evt_cbs_t cbs = { .on_conv_done = onConvDone };
  adc_continuous_register_event_callbacks(adcHandle, &cbs, NULL);

  readingsMutex = xSemaphoreCreateMutex();
  configureChannel(ir_array[0].echo_pin);

  xTaskCreate(irTask, "irTask", 4096, NULL, 3, &irTaskHandle);
}

bool frontEmergency()
{
  if (readings[0] > 1250) return 1;
  return 0;
}

//TODO:channels beto3 irs
bool wallFront() {
  for (int i = 0; i < 10; i++) {
    if (readings[0] > frontThresh ) return 1;
  }
  return 0;
}
bool wallRight() {
  for (int i = 0; i < 10; i++) {
    if (readings[1] > rightThresh) return 1;
  }
  return 0;
}
bool wallLeft() {
  for (int i = 0; i < 10; i++) {
    if (readings[2] > leftThresh) return 1;
  }
  return 0;
}


class PoseEKF : public ekf {
public:
  PoseEKF() : ekf(3, 1) {} //3 states (x,y,theta)// 1 control v

  void Init() override {
    for (int i = 0; i < NUMX; i++)
      for (int j = 0; j < NUMX; j++)
        P(i, j) = (i == j) ? 0.1f : 0.0f;  
    X(0,0) = 0; X(1,0) = 0; X(2,0) = 0;     

    Q(0,0) = 0.02f; Q(0,1) = 0.0f;          
    Q(1,0) = 0.0f;  Q(1,1) = 0.02f; //------------------------------------------------------------------------need to tune
    Q(2,2) = 0.0f; 
  }

  dspm::Mat StateXdot(dspm::Mat &x, float *u) override {
    dspm::Mat xdot(3, 1);
    float v = u[0];      
    float thetaRad = x(2, 0) * PI / 180.0f; 
    xdot(0,0) = v * sinf(thetaRad);
    xdot(1,0) = v * cosf(thetaRad);
    xdot(2,0) = 0.0f; // no gyro
    return xdot;
  }

  void LinearizeFG(dspm::Mat &x, float *u) override {
    float v = u[0];
    float thetaRad = x(2, 0) * PI / 180.0f;
    F(0,0)=0; F(0,1)=0; F(0,2)= v*cosf(thetaRad) * PI / 180.0f;  
    F(1,0)=0; F(1,1)=0; F(1,2)=-v*sinf(thetaRad) * PI / 180.0f;
    F(2,0)=0; F(2,1)=0; F(2,2)=0;

    G(0,0)=sinf(thetaRad);
    G(1,0)=cosf(thetaRad);
    G(2,0)=0;
  }
};

PoseEKF poseEkf;
TaskHandle_t ekfTaskHandle = NULL;
SemaphoreHandle_t poseMutex;

long ekfPrevRightTicks = 0;
long ekfPrevLeftTicks = 0;

void ekfPredict(float dt) {
  long rightPos = rightEncoder.position();
  long leftPos  = leftEncoder.position();

  long rTicks = rightPos - ekfPrevRightTicks;
  long lTicks = leftPos  - ekfPrevLeftTicks;

  ekfPrevRightTicks = rightPos;
  ekfPrevLeftTicks  = leftPos;

  float rightDist = rTicks / (float)ticksperlafa * circumference;
  float leftDist  = lTicks / (float)ticksperlafa * circumference;

  float rightSpeed_raw = rightDist / dt;
  float leftSpeed_raw  = leftDist / dt;
  // float rightSpeed = rightVFilter.update(rightSpeed_raw);
  // float leftSpeed  = leftVFilter.update(leftSpeed_raw);
 
  // float v = (rightSpeed + leftSpeed) / 2.0f;
  float v = (rightSpeed_raw + leftSpeed_raw) / 2.0f;
  // float omega = getRate();// degrees
  poseEkf.X(2,0) = getOrientationX();   
  poseEkf.P(2,2) = 0.0f;
  poseEkf.P(0,2) = poseEkf.P(2,0) = 0.0f;
  poseEkf.P(1,2) = poseEkf.P(2,1) = 0.0f;

  float u[1] = { v };
  poseEkf.Process(u, dt);
}

void ekfTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50);

  
  for (;;) {
    float dt = 20 / 1000.0f;

    if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      ekfPredict(dt);
      xSemaphoreGive(poseMutex);
    }
    
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

//1 4
void turn(double angle) {
  xSemaphoreTake(poseMutex, pdMS_TO_TICKS(5));
  double currentAngle = poseEkf.X(2,0);
  xSemaphoreGive(poseMutex);

  double desiredAngle = currentAngle + angle;
  // desiredAngle = fmod(desiredAngle + 360.0, 360.0); 
  double error = angleDiff(currentAngle, desiredAngle);
  bool direction = (error > 0 ? true : false);  // true -> turn right | false -> turn left
  double errorPrev = error;
  double totalerror = 0;
  unsigned long lastPrint = millis();
  unsigned long lastLoopTime = millis(); 
  double minSpeed = 15;//------------------------------------------------------------------------TODO:need to tune this

  double kp = 1.2;  // TODO:Kp and Kd will be set with testing
  double ki = 0.05;
  double kd = -0.09 ;

  const double integralMax = 30.0;
  double speed = 100;

  int counter = 0;

  while (abs(error) > 1 || fabs(getRate()) > 0.5) {
    if(xSemaphoreTake(poseMutex, pdMS_TO_TICKS(5)) == true){
      currentAngle = poseEkf.X(2,0);
      xSemaphoreGive(poseMutex);
    }
    
    error = angleDiff(currentAngle, desiredAngle);

    unsigned long now = millis();
    double dt = (now - lastLoopTime) / 1000.0;
    lastLoopTime = now;

    double pTerm = kp * error;
    double dTerm = kd * error/dt;

    //anti-windUp
    double iTermTentative = ki * error * dt;
    bool saturating = (pTerm + iTermTentative + dTerm > 100) || (pTerm + iTermTentative + dTerm < -100);
    if (!saturating) {
      totalerror += error * dt;
    }
    double iTerm = constrain(ki * totalerror, -integralMax, integralMax);
    
    speed = pTerm + iTerm + dTerm;
    speed = fixSpeed(speed);
  
    // if (fabs(speed) > 1 && fabs(speed) < minSpeed) {
    //   speed = (speed > 0 ? minSpeed : -minSpeed);
    // }
    
    direction = (speed > 0 ? true : false);
    analogWrite(leftMotorForward, (direction)*abs(speed));
    analogWrite(leftMotorBackward, (!direction) * abs(speed));

    analogWrite(rightMotorForward, (!direction) * abs(speed));
    analogWrite(rightMotorBackward, (direction)*abs(speed));


    errorPrev = error;
    totalerror += error*dt;
    
    if (abs(getRate()) < 0.1) counter++;
    if (counter >= 40) break;

    if(lastPrint-millis() >=100)
    {
      Serial.print("turning ");
      Serial.print(desiredAngle);
      Serial.print(" yaw =");
      Serial.print(poseEkf.X(2,0));
      Serial.print(" error =");
      Serial.print(error);
      Serial.print(" speed =");
      Serial.print(speed);
      Serial.print(" rate =");
      Serial.print(getRate());
      Serial.print("  yaw offset =");
      Serial.println(yawOffset);
      lastPrint = millis();

    }
  }
  Serial.println("done turning");
  
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  theoreticalHeading = currentAngle;
  theoreticalHeading -= (theoreticalHeading>360)? 360:0;
  theoreticalHeading += (theoreticalHeading<0)? 360:0;
}
//3 -1
//04 -1


//2 -1
//-1.2 1.5
bool moveF(double tiles = 16)           // if you want to move tile by tile use moveF(1), if you want continuous use moveF();
{                                       // just need to add to make it stop using the irs
  double desiredDistance = tiles * 19.7;  // el tile el mafrood 18cm, bas we found it would move slightly less than what we wanted, fa we increased it

  double startX = poseEkf.X(0,0), startY = poseEkf.X(1,0);
  double startYaw = theoreticalHeading;

  long startRight = rightEncoder.position();
  long startLeft = leftEncoder.position();
  long rightTicks, leftTicks;
  long startTime = millis();

  rightEncoder.setPosition(0);
  leftEncoder.setPosition(0);
  // previousLeft=0;
  // previousRight=0;
  ekfPrevRightTicks = 0; 
  ekfPrevLeftTicks = 0;
  // for the distance
  double errorL = desiredDistance - ekfCalculateDistance(startX, startY);
  double errorLPrev = errorL;

  bool direction = (errorL >= 0 ? true : false);  // true -> forward, false -> backward

  // to keep moving staight
  double errorA = angleDiff(poseEkf.X(2,0), startYaw);
  double errorAPrev = errorA;

  //double errorTicks = 0;
  double errorTicksPrev = 0;

  unsigned long t = millis();

  double Kpl = 3;  // KD AND KP are changed with testing
  double Kdl = -1;

  double Kpa = -2.95;  // changed
  double Kda = 1.2;    // decreased

  double KpTicks = 0.0;
  double KdTicks = 0.0;
  double kiTicks = 0.0;


  double speedl;
  double speeda;
  double speedTicks = 0;
  double speed;
  ////Serial.println(errorL);

  //unsigned long  timeout_timer = millis();
  char timeout_ctr = 0;
  
  while ((abs(errorL) > 0.2) && timeout_ctr < 50)  // this 1 might change
  {    

    rightTicks = rightEncoder.position() - startRight;
    leftTicks = leftEncoder.position() - startLeft;
    long deltaTicks = rightTicks - leftTicks;
    // static double integralval = 0 ;
    // if (fabs(integralval) > 255)
    //     integralval += kiTicks*(deltaTicks - errorTicksPrev)*(millis() - t) ;
    // speedTicks = KpTicks*deltaTicks + KdTicks * (deltaTicks - errorTicksPrev)/(millis() - t)  + integralval;

    errorL = desiredDistance - ekfCalculateDistance(startX, startY);
    errorA = angleDiff(poseEkf.X(2,0), startYaw);


    speedl = Kpl * errorL + Kdl * (errorL - errorLPrev) / (millis() - t);
    speeda = Kpa * errorA + Kda * getRate();

    direction = (speedl >= 0 ? true : false);

    ////Serial.print(errorL);
    ////Serial.print(" ");
    //Serial.print(yaw);
    //Serial.print(" ");
    ////Serial.print(speedl);
    //Serial.print(fixSpeed(speedl - speeda));
    //Serial.print(" ");
    ////Serial.println(speeda);
    //Serial.print(fixSpeed(speedl + speeda));

    //Serial.print(" ");

    analogWrite(leftMotorForward, direction * abs(fixSpeed(speedl - speeda)));      // might change the +- signs here
    analogWrite(leftMotorBackward, (!direction) * abs(fixSpeed(speedl - speeda)));  // also might add a constrain fa lw el speed less/greater than the limits, it doesn't overflow

    analogWrite(rightMotorForward, direction * abs(fixSpeed(speedl + speeda)));
    analogWrite(rightMotorBackward, (!direction) * abs(fixSpeed(speedl + speeda)));

    errorLPrev = errorL;
    errorAPrev = errorA;
    errorTicksPrev = deltaTicks;

    t = millis();
    //if(t - startTime > 5000)
    if (fabs(getLin()) < 0.1) {
      // timeout_timer = millis();
      timeout_ctr++;
    }
    if(frontEmergency())break;
    //Serial.println(getLin());
  }

  // Logln("Done moveF");
  Serial.println("Done moveF");
  //   //Serial.println(calculateDistance(startX,startY));
  //Serial.println(errorL);

  // analogWrite(rightMotorForward, 0);
  // analogWrite(leftMotorForward, 0);
  // analogWrite(leftMotorBackward, 0);
  // analogWrite(rightMotorBackward, 0);

  // analogWrite(rightMotorForward, 255);
  // analogWrite(leftMotorForward, 255);
  // analogWrite(rightMotorBackward, 255);
  // analogWrite(leftMotorBackward, 255);
  // // delayMicroseconds(20);
  // delay(50);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);



  if (timeout_ctr >= 50) return 0;
  if(errorL > 10)return 0;
  return 1;
}

double fixSpeed(double speed) {
  int maxx = 100;
  speed = constrain(speed, -maxx, maxx);
  if (abs(speed) < 2) return 0;
  if (speed > 0) return map(speed, 0, maxx, 45, maxx);
  if (speed < 0) return map(speed, -maxx, 0, -maxx, -45);
  else return 0;
}

inline double calculateDistance(double x, double y) {
  return sqrt(pow(x - xPosition, 2) + pow(y - yPosition, 2));
}
inline double ekfCalculateDistance(double x, double y){
  return sqrt(pow(x - poseEkf.X(0,0), 2) + pow(y - poseEkf.X(1,0), 2));
}

double angleDiff(double start, double goal) {
  //goal  = (goal + 360) % 360.0;
  double diff = fmod(goal - start, 360.0);
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return diff;
}

/*#################################BNO#######################################*/
inline float getRawYaw() {
  xSemaphoreTake(bnoMutex, portMAX_DELAY);
  vec_3 euler = bno.euler();
  xSemaphoreGive(bnoMutex);
  return euler.x() ;//returns degrees 
}

inline float getOrientationX() {
  double offset;
  portENTER_CRITICAL(&yawMux);
  offset = yawOffset;
  portEXIT_CRITICAL(&yawMux);
  return fmod(getRawYaw()- offset + 360.0,360);
}

inline float getRate() {
  xSemaphoreTake(bnoMutex, portMAX_DELAY);
  vec_3 gyro = bno.gyro();
  xSemaphoreGive(bnoMutex);
  return gyro.vec[0]  *180.0f/PI;
}

inline float getLin() {
  xSemaphoreTake(bnoMutex, portMAX_DELAY);
  vec_3 lin = bno.linear_acceleration();
  xSemaphoreGive(bnoMutex);
  return lin.vec[2];  // Z axis
}

bool calibrateBnoAndSave(imu& bno) {
    Calibration_t s{};
    unsigned long start = millis();
    const unsigned long TIMEOUT_MS = 120000; 

    Serial.println("BNO Calibration...");
    while (true) {
        bno.calibration_status(s);
        Serial.print("sys = ");Serial.print(s.sys);
        Serial.print("gyro = ");Serial.print(s.gyro);
        Serial.print("accel = ");Serial.print(s.accel);
        Serial.print("mag = ");Serial.println(s.mag);
        if (s.sys == 3 && s.gyro == 3 && s.accel == 3 /*&& s.mag == 3*/)
            break;
        if (millis() - start > TIMEOUT_MS)
        {
          Serial.println("BNO Calibration timed out :(");
          return false; 
        }
            
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
        
    CalibProfile_t p;
    bno.getOffsets(p);
    // for (int i = 0; i < 22; i++) { Serial.print(p.data[i], HEX); Serial.print(" "); }
    // Serial.println();

    EEPROM.write(CALIB_FLAG_ADDR,CALIB_MAGIC);
    EEPROM.put(CALIB_DATA_ADDR, p.data);
    EEPROM.commit();
    Serial.println("BNO calibration saved to EEPROM");
    return true;
}

bool loadBnoCalibration(imu& bno)
{
  if(EEPROM.read(CALIB_FLAG_ADDR) == CALIB_MAGIC)
  {
    CalibProfile_t p;
    EEPROM.get(CALIB_DATA_ADDR,p.data);
    bno.setOffsets(p);
    for (int i = 0; i < 22; i++) { Serial.print(p.data[i], HEX); Serial.print(" "); }
    Serial.println();
    return true;
  }

  return false;     
}

void bnoOffsetTask(void *pv)
{
  double prevRawYaw = getRawYaw();
  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
    double rawYaw   = getRawYaw();                    

    if (fabs(angleDiff(prevRawYaw,rawYaw)) > yawJumpThresh) {
      portENTER_CRITICAL(&yawMux);
      yawOffset += angleDiff(rawYaw, prevRawYaw); 
      portEXIT_CRITICAL(&yawMux);
      Serial.println("-------------------------------------BNO jump detected, discrepancy=" + String(angleDiff(prevRawYaw,rawYaw))
                      + " new offset=" + String(yawOffset));
    }
    
    prevRawYaw = rawYaw;
  
  }
}

// inline void getPosition() {
//   double leftRevolutions = 1.0 * leftEncoder.position() / ticksperlafa;
//   double rightRevolutions = 1.0 * rightEncoder.position() / ticksperlafa;

//   double leftDistance = (leftRevolutions - previousLeft) * circumference;
//   double rightDistance = (rightRevolutions - previousRight) * circumference;
//   double distance = (leftDistance + rightDistance) / 2;

//   double encoderDelta = (rightDistance - leftDistance) / distance_between_wheels;  // won't use that , and this is in radian
//   double bnoDelta = (getOrientationX() - yaw);
//   double deltaAngle = bnoDelta;  // add the encoder delta to it if you want

//   yPosition += distance * cos((yaw + deltaAngle / 2) * PI / 180);
//   xPosition += distance * sin((yaw + deltaAngle / 2) * PI / 180);
//   //yaw += deltaAngle;  // or yaw = getOrientationX();
//   yaw = getOrientationX();
//   //yaw = (yaw+360)%360;

//   previousLeft = leftRevolutions;
//   previousRight = rightRevolutions;
// }



void setup() {

  EEPROM.begin(EEPROM_SIZE);  // Allocate 512 bytes for EEPROM emulation
  
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);

  //delay(5000);

  Serial.begin(115200);
  initialise(c_q, MAX_H * MAX_W);  //queue initialisation for storing row and coloumn
  initialise(r_q, MAX_H * MAX_W);

  delay(1000);
  Serial.println(esp_reset_reason());

  //BNO
  bno.init();
  delay(50);
  if(bno.isConnected())
    Serial.println("BNO Connected yayy");
  else
  {
    Serial.println("lol no BNO detected!");
    delay(50000);
  }
  delay(10);
  Calibration_t s{};
  bno.calibration_status(s);
  Serial.print("pre-load calib: ");
  Serial.print(s.sys); Serial.print("/");
  Serial.print(s.gyro); Serial.print("/");
  Serial.println(s.accel);

  if(loadBnoCalibration(bno) == 0)
  {
    Serial.println("oops No bno offsets to load :(");
    calibrateBnoAndSave(bno);//TODO: calibration using button mayenfa3sh nedkhol el mosab2a keda what if fy el round karar ye3ml calibration
    delay(200000);
  }
  else
  {
    Serial.println("Calibration offsets loaded :)");
    bno.calibration_status(s);
    Serial.print("post-load calib: ");
    Serial.print(s.sys); Serial.print("/");
    Serial.print(s.gyro); Serial.print("/");
    Serial.println(s.accel); 
  }

  // bno.set_mode(operation_mode::IMU);
  // delay(10);
  // uint8_t buf[20];
  // bno.read_register(imu_registers::mode::OPR_MODE,buf,1);
  // Serial.print("mode: ");Serial.println(buf[0],HEX);

  delay(2000);
  bnoMutex = xSemaphoreCreateMutex();
  xTaskCreate(bnoOffsetTask, "bnoOffsetTask", 2048, NULL, 1, &bnoOffsetTaskHandle);
  
  setupIR();
  
  interTimer = millis();
  
  leftEncoder.setPosition(0);
  rightEncoder.setPosition(0);
  
  poseEkf.Init();
  poseMutex = xSemaphoreCreateMutex();
  poseEkf.X(2,0) = theoreticalHeading = getOrientationX();
  xTaskCreate(ekfTask, "ekfTask", 2048, NULL, 1, &ekfTaskHandle);


  //IR calibration
  // for(int i=1;i<4;i++)//starting from 1 cuz 0,1 calibrate ma3 ba3d
  //   IRCalibration(i);
  // vTaskPrioritySet(NULL,3);//raises priority of void loop

  delay(10);
  Serial.println("khalast setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  // moveF(1);
  //turn(90);
  //   getPosition();
  //   analogWrite(leftMotorForward, 0);
  //   analogWrite(leftMotorBackward, 0);
  //   analogWrite(rightMotorForward, 0);
  //   analogWrite(rightMotorBackward, 0);
  // delay(2000);
  // analogWrite(leftMotorForward, 400);
  // analogWrite(leftMotorBackward, 0);
  // analogWrite(rightMotorForward, 400);
  // analogWrite(rightMotorBackward, 0);
  // delay(2000);
  // //Serial.print(leftEncoder.position());
  // //Serial.print(" ");
  // //Serial.println(rightEncoder.position());
  ////Serial.println(getRate());

  // if(readings[3] < 100)
  // {
  //     turn(90);
  //     // theoreticalHeading = (theoreticalHeading + 90) % 360;
  //     delay(500);
  //     moveF(1);
  //     //Serial.println("RIGHT");
  // }
  // else if (readings[0] < 100)
  // {
  //     moveF(1);
  //     //Serial.println("FORWARD");
  // }
  // else
  // {
  //     turn(-90);
  //     //Serial.println("LEFT");
  //     // theoreticalHeading = (theoreticalHeading + 270) % 360;
  // }
  // delay(1000);
  // moveF(1);
  // delay(1000);
  // getPosition();
  // //Serial.print(xPosition);
  // //Serial.print(" ");
  // //Serial.print(yPosition);
  // //Serial.print(" ");
  // Serial.println(yaw);
  // delay(100);
  

  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 90;
  // delay(500);
  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 180;
  // delay(500);
  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 270;
  // delay(500);
  // moveF(3);
  // delay(2000);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 0;
  // delay(500);

  // moveF(2);
  // delay(100);
  // turn(180);
  // delay(100);
  // moveF(2);
  // delay(100);
  // turn(-180);
  // delay(100);

  // //Serial.print(wallLeft());
  // //Serial.print(" ");
  // //Serial.print(wallFront());
  // //Serial.print(" ");
  // //Serial.println(wallRight());


///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // option --> correspondance
  // 0 --> floodfill
  // 1 --> right-hand
  // 2 --> left-hand
  
      //Serial.println("0 no menu");
      delay(2000);
      while (!menu) {
        // server.handleClient();
        Serial.println("0 while");
      // delay(2000);
        flood();
        Serial.println("done flood");
        previous_run = current_run;
        exploreToCenter();
        Serial.println("done exploretocenter");
        current_run = dis[16][1];
        //if (current_run != 0 && current_run == previous_run) break;
        flood(0);
        Serial.println("done flood to begin");
        exploreToStart();
        Serial.println("done exploretostart");
        // Log("done!!!! The best run is "+ current_run);
      }
     
  
}


/*
void loop()
{    
  double x,y,yawww;    
  turn(90);
  xSemaphoreTake(poseMutex, pdMS_TO_TICKS(5));
  x = poseEkf.X(0,0);
  y = poseEkf.X(1,0);
  yawww = poseEkf.X(2,0);
  xSemaphoreGive(poseMutex);
  
  Serial.print("    x=");Serial.print(x);
  Serial.print("  y=");Serial.print(y);
  Serial.print("  yaw=");Serial.println(yawww);
  delay(1000);

  // for(int i=0;i<4;i++)
  // {
  //   Serial.print(irToDist(readings[i],i));Serial.print("  ");
  // }
  // Serial.println();


  // turn(90);
  // moveF(1);
  // turn(90);
  // moveF(1);
  // turn(90);
  // moveF(1);
  // turn(90);
  // moveF(1);

  // Serial.print("x=");Serial.print(xPosition);
  // Serial.print("  y=");Serial.print(yPosition);
  // Serial.print("  yaw=");Serial.print(yaw);
  // Serial.print("    x=");Serial.print(poseEkf.X(0,0));
  // Serial.print("  y=");Serial.print(poseEkf.X(1,0));
  // Serial.print("  theta=");Serial.println(poseEkf.X(2,0));
  // // delay(1000);

  // turn(-90);
  // moveF(1);
  // turn(-90);
  // moveF(1);
  // turn(-90);
  // moveF(1);
  // turn(-90);
  // moveF(1);
  // delay(500);
  // for(auto i : readings)
  // {
  //   Serial.print(i);Serial.print("  ");
  // }
  // Serial.println();

  // yaw = getOrientationX();
  // Serial.print("    yaw: ");
  // Serial.println(yaw);
  // delay(5);

}*/
