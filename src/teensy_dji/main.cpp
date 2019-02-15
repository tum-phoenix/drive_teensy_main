#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "parameter.hpp"
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
static constexpr float framerate = 500;

// common

// node state
const int node_state_update_rate_us = 1000000 / 20;     // in us -> 20Hz
MonotonicTime next_node_state_update = MonotonicTime::fromMSec(0);
uint8_t node_fault_code1 = 0;
uint8_t node_fault_code2 = 0;

// DJI
//#define DJI_DEBUG_OUTPUT
const int rc_update_rate_us = 1000000 / 50;             // in us -> 20Hz
const int rc_timeout_us = 1000 * 100;                   // timeout if last rc command is 100ms old
MonotonicTime next_rc_update = MonotonicTime::fromMSec(0);
MonotonicTime last_rc_receive = MonotonicTime::fromMSec(0);
DJI dji(Serial2);

// Power
#define IGNORE_CELL4          // use this to run teensy without connected CAN-cable (it is used for V4 measurement)
const int power_update_rate_us = 1000000 / 2;         // in us -> 2Hz
MonotonicTime next_power_update = MonotonicTime::fromMSec(0);
#define CELL3_PIN A5
#define CELL2_PIN A11
#define CELL1_PIN A10
#define CELL4_PIN A0
#define CURR_PIN  A1
#define CELL_R1 14000.0
#define CELL_R2 2000.0
#define BAT_V_THRESH 0.5
#define A_REF 3.3
#define CURR_FACTOR (float)(0.00161133 / 4) // 0R01 + 200V/V
#define V_ALM_FINAL 3.0

// buzzer
#define BUZZER_PIN 11
const int buzzer_beep_rate_us = 1000000 / 2;              // in us -> 2Hz
MonotonicTime next_buzzer_update = MonotonicTime::fromUSec(0);
uint8_t battery_alarm = 0;

// Buttons
//#define BUTTONS_DEBUG_OUTPUT
const int button_update_rate_us = 1000000 / 10;           // in us -> 10Hz
MonotonicTime next_button_update = MonotonicTime::fromUSec(0);
#define BUTTON_PIN A4
uint8_t bitwise_buttons = 0;

// Vesc
//#define VESC_DEBUG_OUTPUT
const int motor_state_update_rate_us = 1000000 / 100;    // in us  -> 100Hz
MonotonicTime next_motor_state_request_time = MonotonicTime::fromUSec(0);
MonotonicTime next_motor_state_update_time = MonotonicTime::fromUSec(0);
int motor_state_reply_duration_us = 3500;   // time between request and receive of a motor_state
int vesc_motor_state_requests[2] = {0, 0};
int vesc_motor_state_receives[2] = {0, 0};
int vesc_motor_state_received[2] = {0, 0};
int vesc_motor_state_alive[2] = {0, 0};     // is larger 0 if VESC is alive
struct bldcMeasure measuredVal_motor3;      // rear left
struct bldcMeasure measuredVal_motor4;      // rear right

// servos for steering
PWMServo steering_servo_3;
PWMServo steering_servo_4;
float steering_servo_offset_3 = 90;
float steering_servo_offset_4 = 90;
uint8_t steering_servo_3_pin = 5;
uint8_t steering_servo_4_pin = 20;

// motor target misc
MonotonicTime last_motor_target_receive = MonotonicTime::fromMSec(0);
#define MOTOR_COM_TIMEOUT_US 1000 * 100     // max accepted time between motor commands in us

// Pepperl+Fuchs / parallel parking
const int par_lot_update_rate_us = 1000000 / 50;     // in us  -> 50Hz
MonotonicTime next_par_lot_update = MonotonicTime::fromMSec(0);
#define PF_LS_PIN 13
float lot_size = 0;
float last_lot_pos = 0;
uint8_t publish_lot_msg_to_send = 0;

// odometry
typedef struct {
  float dist_trav;  // m
  float speed;      // m/s
} odometry_t;
odometry_t rear;

#define WHEEL_RADIUS_M 0.033

float v_veh();
float x_veh();
void buzzer_routine();
bool check_arm_state();

#include "Publisher.h"
#include "Subscriber.h"

void setup() {
  // init Buzzer Pin and turn Buzzer on during setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 1);

  // setup UART port for vesc
  Serial1.begin(230400);
  Serial3.begin(230400);
  SetSerialPort(&Serial1, &Serial3);

  // setup UART for usb port as debug output
  Serial.begin(115200);

  // setup power
  analogReadRes(12);
  analogReadAveraging(4);

  // Pepperl+Fuchs
  //RISING/HIGH/CHANGE/LOW/FALLING
  attachInterrupt(PF_LS_PIN, pf_ir_routine, CHANGE);

  // init LEDs
  initLeds();

  // create CAN node
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

  // init parameter
  initParameter(node);

  // start up node
  node->setModeOperational();

  // setup DJI remote
  dji.begin(&Serial2);

  // turn buzzer off after setup
  digitalWrite(BUZZER_PIN, 0);

  // setup servos for steering
  steering_servo_offset_3 = configuration.steeringOff_RL + 90;
  steering_servo_offset_4 = configuration.steeringOff_RR + 90;
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

  // update buzzer
  buzzer_routine();

  // in case no recent motor target current is received via CAN: turn them off
  if ((systemClock->getMonotonic() - last_motor_target_receive).toUSec() > MOTOR_COM_TIMEOUT_US) {
    VescUartSetCurrent(0, 0);
    VescUartSetCurrent(0, 1);
    steering_servo_3.detach();
    steering_servo_4.detach();
  }
  
}

float v_veh() 
{
  // erpm / 7 = RPM                 rounds per minute
  // erpm / 7 / 60 = RPS            rounds per second
  // erpm / 7 / 60 * (2 pi r) = m/s
  float mean_speed = 0;
  float num = 0;
  if (vesc_motor_state_alive[0] > 0) {
    mean_speed += (float)measuredVal_motor3.erpm / 7. / 60. * 2. * M_PI * WHEEL_RADIUS_M;
    num ++;
  }
  if (vesc_motor_state_alive[1] > 0) {
    mean_speed += (float)measuredVal_motor4.erpm / 7. / 60. * 2. * M_PI * WHEEL_RADIUS_M;
    num ++;
  }

  if (num == 0) {
    rear.speed = 0;
  } else {
    rear.speed = mean_speed / num;    // m/s;
  }
  return rear.speed;
}

float x_veh()
{
  v_veh();
  static MonotonicTime last_x_veh_update = systemClock->getMonotonic();
  static MonotonicTime this_x_veh_update = systemClock->getMonotonic();
  last_x_veh_update = this_x_veh_update;
  this_x_veh_update = systemClock->getMonotonic();
  float dt = (float)(this_x_veh_update - last_x_veh_update).toUSec();     // micros
  float ds = (rear.speed * dt) / (float)1000000.;                         // m
  rear.dist_trav += ds;
  return rear.dist_trav;
}

void buzzer_routine() {
  if ((battery_alarm) && (systemClock->getMonotonic() > next_buzzer_update)) {
     next_buzzer_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(buzzer_beep_rate_us);
      static uint8_t buzzer_mode = 0;
      buzzer_mode ^= 1;
      digitalWrite(BUZZER_PIN, buzzer_mode);
  } else if (battery_alarm == 0) {
      digitalWrite(BUZZER_PIN, 0);
  }
}
