#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "Publisher.h"
#include "Subscriber.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// CAN Node settings
static constexpr uint32_t nodeID = 102;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.body_control";

// application settings
static constexpr float framerate = 100;

Adafruit_BNO055 bno055 = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);

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

  // do some CAN stuff
  cycleNode(node);

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
  bno_data.euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);


  // cycle publisher
  cyclePublisher(bno_data);

  // toggle heartbeat
  toggleHeartBeat();
}