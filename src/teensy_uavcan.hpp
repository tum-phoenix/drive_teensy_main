#ifndef	TEENSY_UAVCAN_HPP
#define	TEENSY_UAVCAN_HPP

#include <UAVCAN.hpp>

using namespace uavcan;

// git hash for software version
#ifdef __GIT_HASH__
  #define GIT_HASH __GIT_HASH__
#else
  #define GIT_HASH 0
#endif

// some other parameter
static constexpr uint32_t bitrate = 1000000;      // bit rate of can bus
static const unsigned NodeMemoryPoolSize = 8192;  // size of node memory
static constexpr int nodeSpinTimeUS = 1000;       // spin in each cycle to fetch new can messages in [us]

// uavcan node
Node<NodeMemoryPoolSize> *node;

// heartbeat LED
bool heartBeatLed = false;
int heartBeatLedPin = 16;
MonotonicTime lastBeat = MonotonicTime::fromMSec(0);
int heartBeatFreq = 2;

// traffic (or general purpose) LED
int trafficLedPin = 17;
bool trafficLed = false;

// teensy LED
int teensyLedPin = 13;
bool teensyLed = false;

// interfaces to systemclock and canDriver
ISystemClock* systemClock;
ICanDriver* canDriver;

// end of last cycle
MonotonicTime oldTime = MonotonicTime::fromMSec(0);

// get system clock interface
ISystemClock& getSystemClock()
{
  return uavcan_nxpk20::SystemClock::instance();
}

// get can driver interface
ICanDriver& getCanDriver()
{
  static bool initialized = false;
  if (!initialized)
  {
      initialized = true;
      int res = uavcan_nxpk20::CanDriver::instance().init(bitrate);
      if (res < 0)
      {
          Serial.println("Error on CanDriver initialization");
      }
  }
  return uavcan_nxpk20::CanDriver::instance();
}


// restarts the teensy if restart request is being send
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

class : public uavcan::IRestartRequestHandler
{
    bool handleRestartRequest(uavcan::NodeID request_source) override
    {
        Serial.println("Got a remote restart request!");
        WRITE_RESTART(0x5FA0004);
        return true;
    }
} restart_request_handler;


// initialize heart beat
bool initLeds()
{
  pinMode(heartBeatLedPin, OUTPUT);
  pinMode(trafficLedPin, OUTPUT);
  pinMode(teensyLedPin, OUTPUT);
  return true;
}

// initialize UAVCAN node
bool initNode(Node<NodeMemoryPoolSize> *node, const uint32_t nodeID, const char* nodeName,
              const uint8_t swVersion, const uint8_t hwVersion)
{

  protocol::SoftwareVersion sw_ver;
  sw_ver.major = swVersion;
  sw_ver.minor = 0;
  sw_ver.vcs_commit = GIT_HASH;

  protocol::HardwareVersion hw_ver;
  hw_ver.major = hwVersion;
  hw_ver.minor = 0;

  node->setNodeID(nodeID);
  node->setName(nodeName);
  node->setSoftwareVersion(sw_ver);
  node->setHardwareVersion(hw_ver);
  node->setRestartRequestHandler(&restart_request_handler);

  if(node->start() >= 0)
  {
    Serial.println("Application node created");
    return true;
  }
  Serial.println("Unable to start node!");
  return false;
}

// toggle state of heartbeat led
void toggleHeartBeat()
{
  if(lastBeat + MonotonicDuration::fromMSec(1000/(float)heartBeatFreq)
      < systemClock->getMonotonic())
  {
    heartBeatLed = !heartBeatLed;
    digitalWrite(heartBeatLedPin, heartBeatLed);
    lastBeat = systemClock->getMonotonic();
  }
}

// toggle state of traffic led
void toggleTraffic()
{
  trafficLed = !trafficLed;
  digitalWrite(trafficLedPin, trafficLed);
}

// toggle state of teensy led
void toogleTeensy()
{
  teensyLed = ! teensyLed;
  digitalWrite(teensyLedPin, teensyLed);
}

// spin node
void cycleNode(Node<NodeMemoryPoolSize> *node)
{
  const int res = node->spin(uavcan::MonotonicDuration::fromUSec(nodeSpinTimeUS));
  if (res < 0)
  {
      Serial.println("Error while spinning...");
  }
}

// wait in each cycle (given a framerate)
void cycleWait(const float framerate)
{
  if(oldTime.isZero()){
    oldTime = systemClock->getMonotonic();
  }

  MonotonicTime newTime = systemClock->getMonotonic();
  MonotonicDuration diff = newTime - oldTime;
  delay(1000/(float)framerate - (diff.toUSec())/(float)1000);
  oldTime = systemClock->getMonotonic();
}

#endif
