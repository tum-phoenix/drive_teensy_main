#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "DJI.h"
#include "Publisher.h"

// CAN Node settings
static constexpr uint32_t nodeID = 101;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.dji";

// application settings
static constexpr float framerate = 100;

// DJI
DJI dji(Serial1);


void setup() {
  Serial.begin(115200);

  // init LEDs
  initLeds();

  // create a node
  systemClock = &getSystemClock();
  canDriver = &getCanDriver();
  node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
  initNode(node, nodeID, nodeName, swVersion, hwVersion);

  // init publisher
  initPublisher(node);

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
