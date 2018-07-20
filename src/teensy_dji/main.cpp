
#define DEBUG         // activate for vesc debug output
#include <VescUart.h>
#include <datatypes.h>

#include "PWMServo.h"

#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "DJI.h"

// CAN Node settings
static constexpr uint32_t nodeID = 101;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.dji";

// application settings
static constexpr float framerate = 100;

// DJI
int rc_update_rate = 10;
MonotonicTime last_rc_update = MonotonicTime::fromMSec(0);
DJI dji(Serial1);

// Vesc
int motor_state_update_rate = 10;
MonotonicTime last_motor_state_update = MonotonicTime::fromMSec(0);
struct bldcMeasure measuredVal_motor0;
struct bldcMeasure measuredVal_motor1;
float max_fet_temperature = 80;

// servos for steering
PWMServo steering_servo_0;
PWMServo steering_servo_1;
float steering_servo_position_0 = 0;
float steering_servo_position_1 = 0;
float steering_servo_offset_0 = 90;
float steering_servo_offset_1 = 90;
uint8_t steering_servo_0_pin = 21;
uint8_t steering_servo_1_pin = 22;


#include "Publisher.h"
#include "Subscriber.h"


void setup() {
  // setup UART port for vesc
  SetSerialPort(&Serial1, &Serial2, NULL, NULL);
  Serial1.begin(115200);
  Serial2.begin(115200);

  #ifdef DEBUG
    // setup debug port for vesc
    SetDebugSerialPort(&Serial);
  #endif
  Serial.begin(115200);

  // setup servos for steering
  steering_servo_0.attach(steering_servo_0_pin);
  steering_servo_1.attach(steering_servo_1_pin);

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
  dji.begin();

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
}
