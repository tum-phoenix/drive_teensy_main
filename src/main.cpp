#include "Arduino.h"
#include "teensy_uavcan.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "parameter.hpp"

// Node settings
static constexpr uint32_t nodeID = 101;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.example_node";

// application settings
static constexpr float framerate = 100;


void setup()
{
  delay(3000);
  Serial.begin(9600);
  Serial.println("Setup");

  // init LEDs
  initLeds();

  // Create a node
  systemClock = &getSystemClock();
  canDriver = &getCanDriver();
  node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
  initNode(node, nodeID, nodeName, swVersion, hwVersion);

  // init publisher
  initPublisher(node);

  // init subscriber
  initSubscriber(node);

  // start up node
  node->setModeOperational();

  // init parameter
  initParameter(node);
}



void loop()
{
  // wait in cycle
  cycleWait(framerate);

  // do some CAN stuff
  cycleNode(node);

  // publish messages
  cyclePublisher();

  // toggle heartbeat
  toggleHeartBeat();
}
