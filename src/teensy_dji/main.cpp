#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "vuart.h"
#include <math.h>

#include "PWMServo.h"

#include "DJI.h"

// CAN Node settings
static constexpr uint32_t nodeID = 102;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.dji";

// application settings
static constexpr float framerate = 1000;

// common
uint8_t bat_alm = 0;

// DJI
int rc_update_rate = 50;
MonotonicTime last_rc_update = MonotonicTime::fromMSec(0);
DJI dji(Serial2);

// Power
int power_update_rate = 5;
MonotonicTime last_power_update = MonotonicTime::fromMSec(0);
#define CELL3_PIN A5
#define CELL2_PIN A11
#define CELL1_PIN A10
#define CELL4_PIN A0
#define CURR_PIN  A1
#define BAT_V_THRESH 0.5
#define CELL_R1 14000.0
#define CELL_R2 2000.0
#define BAT_V_THRESH 0.5
#define A_REF 3.3
#define CURR_FACTOR 0.00161133/4 // 0R01 + 200V/V
#define V_ALM_FINAL 3.0

// buzzer
#define BUZZER_PIN 11
uint8_t buf_state = 0;
int buzzer_beep_rate = 2;
MonotonicTime last_buzzer_update = MonotonicTime::fromMSec(0);

// Buttons
int button_update_rate = 10;
MonotonicTime last_button_update = MonotonicTime::fromMSec(0);
#define BUTTON_PIN A4
uint8_t bit_but = 0;

// Vesc
int motor_state_update_rate = 100;
MonotonicTime last_motor_state_update = MonotonicTime::fromMSec(0);
struct bldcMeasure measuredVal_motor3;
struct bldcMeasure measuredVal_motor4;
float max_fet_temperature = 80;
float vveh = 0;

// servos for steering
PWMServo steering_servo_3;
PWMServo steering_servo_4;
float steering_servo_position_3;
float steering_servo_position_4;
float steering_servo_offset_3 = 88.8;
float steering_servo_offset_4 = 101;
uint8_t steering_servo_3_pin = 20;
uint8_t steering_servo_4_pin = 5;

// motor target misc
uint32_t last_motor_target_receive = 0;
#define MOTOR_COM_TIMEOUT 500             // max accepted time between motor commands in ms

// Pepperl+Fuchs / parallel parking
#define PF_LS_PIN 13
int32_t lot_size = 0;
int32_t last_lot_pos = 0;
uint8_t publish_lot_msg = 0;
int par_lot_update_rate = 50;
MonotonicTime last_par_lot_update = MonotonicTime::fromMSec(0);

// odometry
typedef struct {
  int32_t dist_trav;  // mm
  int16_t speed;      // mm/s
} odometry_t;
odometry_t rear;

void v_veh();
void buzzer_routine();
bool check_arm_state();

#include "Publisher.h"
#include "Subscriber.h"

void setup() {
  pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN,1);
  // setup UART port for vesc
  
  Serial1.begin(250000);
  Serial3.begin(250000);
  SetSerialPort(&Serial1, &Serial3);

  Serial.begin(115200);

  // setup power
  analogReadRes(12);
  analogReadAveraging(4);

  // setup servos for steering
  steering_servo_3.attach(steering_servo_3_pin);
  steering_servo_4.attach(steering_servo_4_pin);

  // Pepperl+Fuchs
  //RISING/HIGH/CHANGE/LOW/FALLING
  attachInterrupt (PF_LS_PIN, pf_ir_routine, CHANGE);

  // init LEDs
  initLeds();

  // create a node
  systemClock = &initSystemClock();
  canDriver = &initCanDriver();
  node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
  initNode(node, nodeID, nodeName, swVersion, hwVersion);

  // init publisher
  initPublisher(node);

  // init subscriber
  initSubscriber(node);

  // set up filters
  configureCanAcceptanceFilters(*node);

  // start up node
  node->setModeOperational();

  // setup DJI remote
  dji.begin(&Serial2);

  digitalWrite(BUZZER_PIN,0);
}

void loop() {

  // wait in cycle
  cycleWait(framerate);

  // do some CAN stuff
  cycleNode(node);

  // cycle publisher
  cyclePublisher(dji);

  // toggle heartbeat
  toggleHeartBeat();

  buzzer_routine();

  if (millis() - last_motor_target_receive > MOTOR_COM_TIMEOUT) {
    VescUartSetCurrent(0, 0);
    VescUartSetCurrent(0, 1);
  }
  
}

void v_veh() {
  float mean_rounds = ((float)measuredVal_motor3.rpm + (float)measuredVal_motor4.rpm)/14; // RPM
  mean_rounds /= 60; // RPS
  vveh = mean_rounds * 2 * M_PI * WHEEL_RADIUS_M; // m/s;
}

void buzzer_routine() {
  if (bat_alm) {
    if(last_buzzer_update +
      MonotonicDuration::fromMSec(1000/(float)buzzer_beep_rate) <
      systemClock->getMonotonic())
   {
      last_buzzer_update = systemClock->getMonotonic();
      static uint8_t buzzer_mode=0;
      buzzer_mode ^= 1;
      digitalWrite(BUZZER_PIN,buzzer_mode);
   }
  }
}

bool check_arm_state() {
  // TODO
  return true;
}