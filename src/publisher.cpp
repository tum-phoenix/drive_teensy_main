#include "Arduino.h"
#include "teensy_uavcan.hpp"
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

// Node settings
static constexpr uint32_t nodeID = 101;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.publisher";

// application settings
static constexpr float framerate = 100;

// publisher and subscriber
Publisher<protocol::debug::LogMessage> *logPublisher;
Publisher<protocol::debug::KeyValue> *keyPublisher;


void setup()
{
  delay(3000);
  Serial.begin(9600);
  Serial.println("Setup");

  // init heart beat LED
  initHeartBeat();

  // Create a node
  systemClock = &getSystemClock();
  canDriver = &getCanDriver();
  node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
  initNode(node, nodeID, nodeName, swVersion, hwVersion);

  // Create a publishers and subscribers
  logPublisher = new Publisher<protocol::debug::LogMessage>(*node);
  keyPublisher = new Publisher<protocol::debug::KeyValue>(*node);

  // Initiliaze publishers and subscribers
  if(logPublisher->init() < 0)
  {
    Serial.println("Unable to initialize log message publisher!");
  }
  if(keyPublisher->init() < 0)
  {
    Serial.println("Unable to initialize key message publisher!");
  }

  // set TX timeout
  logPublisher->setTxTimeout(MonotonicDuration::fromUSec(800));
  keyPublisher->setTxTimeout(MonotonicDuration::fromUSec(800));

  // start up node
  node->setModeOperational();
}

void loop()
{
  // wait in cycle
  waitCycle(framerate);

  // do some CAN stuff
  canCycle(node);

  // toggle heartbeat
  toggleHeartBeat(2);

  // send a very important log message to everyone

  {
    protocol::debug::LogMessage msg;

    msg.level.value = protocol::debug::LogLevel::DEBUG;
    msg.text = "TUM PHOENIX Robotics is cool";
    msg.source = "Teensy";

    const int pres = logPublisher->broadcast(msg);
    if (pres < 0)
    {
      Serial.println("Error while broadcasting log message");
    }
  }


  // send everyone the truth
  {
    protocol::debug::KeyValue msg;

    msg.value = 42;
    msg.key = "Solution to all Problems!";

    const int pres = keyPublisher->broadcast(msg);
    if (pres < 0)
    {
      Serial.println("Error while broadcasting key message");
    }
  }
}
