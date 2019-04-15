#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "parameter.hpp"
#include "vuart.h"
#include <math.h>

#include "Teensy3_2_pwm.h"

#include "DJI.h"

// general
uint8_t arm_state = 0;

// CAN Node settings
static constexpr uint32_t nodeID = 102;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char *nodeName = "org.phoenix.dji";

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
#define CURR_PIN A1
#define BAT_V_THRESH 0.5
#define CELL_R1 14000.0
#define CELL_R2 2000.0
#define BAT_V_THRESH 0.5
#define A_REF 3.3
#define CURR_FACTOR 0.00161133 / 4 // 0R01 + 200V/V
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

#define MOT_POL_NUM 14
#define MOTOR_Y_WIND_FACTOR 1.7

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

bool custom_vesc_config_set = 0;
typedef struct {
    float min_current = -20.0;
    float max_current = 20.0;
    float min_erpm = -20000;
    float max_erpm = 20000;
} mcconf_t;
mcconf_t mcconf;

int vesc_com_start_delay_ms = 5000;


// servos for steering
PWM_T32 steering_servo_3;
PWM_T32 steering_servo_4;
float steering_servo_position_3;
float steering_servo_position_4;
float steering_servo_offset_3 = 90;
float steering_servo_offset_4 = 90;
uint8_t steering_servo_3_pin = 5;
uint8_t steering_servo_4_pin = 20;

// motor target misc
uint32_t last_motor_target_receive = 0;
#define MOTOR_COM_TIMEOUT 500 // max accepted time between motor commands in ms

// Pepperl+Fuchs / parallel parking
const int par_lot_update_rate_us = 1000000 / 50;     // in us  -> 50Hz
MonotonicTime next_par_lot_update = MonotonicTime::fromMSec(0);
#define PF_LS_PIN 13
float lot_size = 0;
float last_lot_pos = 0;
uint8_t publish_lot_msg_to_send = 0;

// odometry
typedef struct
{
  int32_t dist_trav; // mm
  int16_t speed;     // mm/s
} odometry_t;
odometry_t rear;

#define WHEEL_RADIUS_M 0.033

float v_veh();

float x_veh();

void buzzer_routine();
uint8_t check_arm_state();

#include "Publisher.h"
#include "Subscriber.h"

void setup()
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 1);
  // setup UART port for vesc

  Serial1.begin(230400);
  Serial3.begin(230400);
  SetSerialPort(&Serial1, &Serial3);

    // setup UART port for vesc
    Serial1.begin(230400);
    Serial3.begin(230400);
    SetSerialPort(&Serial1, &Serial3);

    // setup UART for usb port as debug output
    Serial.begin(115200);

  // Pepperl+Fuchs
  //RISING/HIGH/CHANGE/LOW/FALLING
  attachInterrupt(PF_LS_PIN, pf_ir_routine, CHANGE);

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

  // setup DJI remote
  dji.begin();

  digitalWrite(BUZZER_PIN, 0);

  // setup servos for steering
  steering_servo_3.attach(steering_servo_3_pin);
  steering_servo_4.attach(steering_servo_4_pin);
  steering_servo_offset_3 = configuration.steeringOff_RL + 90;
  steering_servo_offset_4 = configuration.steeringOff_RR + 90;
}

void loop()
{

  // wait in cycle
  cycleWait(framerate);

  // do some CAN stuff
  cycleNode(node);

  // cycle publisher
  cyclePublisher(dji);

  // toggle heartbeat
  toggleHeartBeat();

  buzzer_routine();

  if (millis() - last_motor_target_receive > MOTOR_COM_TIMEOUT)
  {
    VescUartSetCurrent(0, 0);
    VescUartSetCurrent(0, 1);
  }
}

void v_veh()
{
  float mean_rounds = ((float)measuredVal_motor3.rpm + (float)measuredVal_motor4.rpm) / 14; // RPM
  mean_rounds /= 60;                                                                        // RPS
  vveh = mean_rounds * 2 * M_PI * WHEEL_RADIUS_M;                                           // m/s;
}

void buzzer_routine()
{
  if (bat_alm)
  {
    if (last_buzzer_update +
            MonotonicDuration::fromMSec(1000 / (float)buzzer_beep_rate) <
        systemClock->getMonotonic())
    {
      last_buzzer_update = systemClock->getMonotonic();
      static uint8_t buzzer_mode = 0;
      buzzer_mode ^= 1;
      digitalWrite(BUZZER_PIN, buzzer_mode);
    }
  }
}

uint8_t check_arm_state()
{
  if (arm_state == RemoteControl::AUX_MODE_CENTER || arm_state == RemoteControl::AUX_MODE_UP)
  {
    if (!steering_servo_3.attached() || !steering_servo_4.attached())
    {
      // setup servos for steering
      steering_servo_3.attach(steering_servo_3_pin);
      steering_servo_4.attach(steering_servo_4_pin);
    }
    return true;
  }
  else
  {
    steering_servo_3.detach();
    steering_servo_4.detach();
    return false;
  }
}
