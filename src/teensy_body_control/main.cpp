#include "Arduino.h"
#include "phoenix_can_shield.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <VescUart.h>
#include "parameter.hpp"

// CAN Node settings
static constexpr uint32_t nodeID = 102;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.body_control";

// application settings
static constexpr float framerate = 100;

// Driving dynamics
enum WHEEL_POS{
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT,
};

typedef struct {
  float thr;
  float steer_f;
  float steer_r;
  float drive_state;
} drive_comm_t;
drive_comm_t RC_coms;
// Vesc
static struct actor_comm_t{
  float motor_amps[4] = {0,0,0,0};
  float servo_angles[4] = {90,90,90,90};
} actor_comms;

bldcMeasure measuredVal_motor1;
bldcMeasure measuredVal_motor2;

// BNO055 imu
Adafruit_BNO055 bno055 = Adafruit_BNO055();


#include "Publisher.h"
#include "Subscriber.h"

void dynamics_control();

void setup() {
  //get EEPROM Parameters
  readParamsFromEEPROM();

  Serial.begin(115200);

  // setup UART port for vesc
  Serial1.begin(115200);
  Serial2.begin(115200);
  SetSerialPort(&Serial1, &Serial2, &Serial1, &Serial1);
  //SetDebugSerialPort(&Serial);
  

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

  // set up BNO055 IMU Adafruit_Sensor
  bno055.begin();

}

imu_t bno_data;

void loop() {
  // wait in cycle
  cycleWait(framerate);

  // get RC data, high level commands, motor telemetry rear motors
  cycleNode(node);

  // update motor front left information
  if (!VescUartGetValue(measuredVal_motor1, 0)) Serial.println("failed to get motor data front left!");
  // update motor front right information
  if (!VescUartGetValue(measuredVal_motor2, 1)) Serial.println("failed to get motor data front right!");

  // BNO055 data aquisition
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  bno_data.lin_acc = bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno_data.gyro = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //bno_data.euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);

  // main driving dynamics calculations
  dynamics_control();

  // cycle publisher
  cyclePublisherBNO(bno_data);
  cyclePublisher_Mot_State(measuredVal_motor1, FRONT_LEFT);
  cyclePublisher_Mot_State(measuredVal_motor2, FRONT_RIGHT);
  cyclePublisher_Actor_Comms(actor_comms);
  
  // set the connected Motors 
  VescUartSetCurrent(actor_comms.motor_amps[FRONT_LEFT],0);
  VescUartSetCurrent(actor_comms.motor_amps[FRONT_RIGHT],1);

  
  // toggle heartbeat
  toggleHeartBeat();
}

void dynamics_control() {
  #define TICKS_P_DEG 6 // servo steps per actual degree
  // calculates currents and steering angles for each wheel
  actor_comms.motor_amps[FRONT_LEFT] = RC_coms.thr*configuration.maxMotorAmps;
  actor_comms.motor_amps[FRONT_RIGHT] = RC_coms.thr*configuration.maxMotorAmps;
  actor_comms.motor_amps[REAR_LEFT] = RC_coms.thr*configuration.maxMotorAmps;
  actor_comms.motor_amps[REAR_RIGHT] = RC_coms.thr*configuration.maxMotorAmps;
  
  actor_comms.servo_angles[FRONT_LEFT] = RC_coms.steer_f*TICKS_P_DEG;
  actor_comms.servo_angles[FRONT_RIGHT] = RC_coms.steer_f*TICKS_P_DEG;
  actor_comms.servo_angles[REAR_LEFT] = RC_coms.steer_r*TICKS_P_DEG;
  actor_comms.servo_angles[REAR_RIGHT] = RC_coms.steer_r*TICKS_P_DEG;
}