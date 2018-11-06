#ifndef	PHOENIX_CAN_SHIELD_H
#define	PHOENIX_CAN_SHIELD_H

#include <uavcan/uavcan.hpp>
#include <uavcan_nxpk20/uavcan_nxpk20.hpp>
#include <uavcan/transport/can_acceptance_filter_configurator.hpp>

extern "C"{
  int _getpid(){ return -1;}
  int _kill(int pid, int sig){ return -1; }
  int _write(){return -1;}
}

using namespace uavcan;

// git hash for software version
#ifdef __GIT_HASH__
  #define GIT_HASH __GIT_HASH__
#else
  #define GIT_HASH 0
#endif

// CAN interface default parameter    
uavcan_nxpk20::IfaceParams iface_param[] = {
    // configuration of CAN0
    {
      .bitrate=1000000,               // bit rate of can bus
      .tx_buff_size=50,               // TX ring buffer size
      .rx_buff_size=50,               // RX ring buffer size
      .use_alt_tx_pin = false,        // use alternative TX pin
      .use_alt_rx_pin = false,        // use alternative RX pin
      .dis_all_RX_by_default = false  // disables all RX mailboxes and only allows receiving of can messages configured via configureCanAcceptanceFilters()
    }
  };

// uavcan node
static const unsigned NodeMemoryPoolSize = 8192;  // size of node memory
Node<NodeMemoryPoolSize> *node;

// heartbeat LED
bool heartBeatLed = true;
const int heartBeatLedPin = 16;
MonotonicTime lastBeat = MonotonicTime::fromMSec(0);
float heartBeatFreq = 5;

// traffic LED
const int trafficLedPin = 17;
bool trafficLed = false;

// teensy LED
const int teensyLedPin = 13;
bool teensyLed = true;

// RGB LED
const int rgbBLedPin = 21;
const int rgbGLedPin = 22;
const int rgbRLedPin = 23;

// interfaces to systemclock and canDriver
ISystemClock* systemClock;
ICanDriver* canDriver;

// end of last cycle
MonotonicTime oldTime = MonotonicTime::fromMSec(0);

// init system clock interface
ISystemClock& initSystemClock()
{
  return uavcan_nxpk20::SystemClock::instance();
}

// init can driver interface
ICanDriver& initCanDriver()
{
  static bool initialized = false;
  if (!initialized)
  {
      initialized = true;
      
      uavcan_nxpk20::CanDriver::instance().init(iface_param);
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
  // set pin mode
  pinMode(rgbBLedPin,      OUTPUT);
  pinMode(rgbGLedPin,      OUTPUT);
  pinMode(rgbRLedPin,      OUTPUT);
  pinMode(trafficLedPin,   OUTPUT);
  //pinMode(teensyLedPin,    OUTPUT);
  pinMode(heartBeatLedPin, OUTPUT);

  // write first output
  digitalWrite(rgbBLedPin,      false);
  digitalWrite(rgbGLedPin,      false);
  digitalWrite(rgbRLedPin,      false);
  digitalWrite(trafficLedPin,   trafficLed);
  //digitalWrite(teensyLedPin,    teensyLed);
  digitalWrite(heartBeatLedPin, heartBeatLed);

  return true;
}

// initialize UAVCAN node
bool initNode(Node<NodeMemoryPoolSize> *node, const uint32_t nodeID, const char* nodeName,
              const uint8_t swVersion, const uint8_t hwVersion)
{
  // adjust UTC
  uavcan::UtcDuration adjustment;
  systemClock->adjustUtc(adjustment.fromMSec(0)); // no adjustment for now

  protocol::SoftwareVersion sw_ver;
  sw_ver.major = swVersion;
  sw_ver.minor = 0;
  sw_ver.vcs_commit = GIT_HASH;
  sw_ver.optional_field_flags = sw_ver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

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

// toggle simple led
void toggleLED(const uint8_t pin, bool& status)
{
  status = !status;
  digitalWrite(pin, status);
}

// toggle led at framerate
void toggleLED(const uint8_t pin, bool& status, const float rate, MonotonicTime& last)
{
  if(last + MonotonicDuration::fromMSec(1000/(float)rate) < systemClock->getMonotonic())
  {
    toggleLED(pin, status);
    last = systemClock->getMonotonic();
  }
}

// toggle state of heartbeat led
void toggleHeartBeat()
{
  toggleLED(heartBeatLedPin, heartBeatLed, heartBeatFreq, lastBeat);
}

// toggle state of traffic led
void toggleTraffic()
{
  toggleLED(trafficLedPin, trafficLed);
}

// toggle state of teensy led
void toggleTeensy()
{
  toggleLED(teensyLedPin, teensyLed);
}

// spin node
void cycleNode(Node<NodeMemoryPoolSize> *node)
{
  const int res = node->spinOnce();
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

void setRGBled(uint8_t r, uint8_t g, uint8_t b)
{
  analogWrite(rgbRLedPin, 0xff-r);
  analogWrite(rgbGLedPin, 0xff-g);
  analogWrite(rgbBLedPin, 0xff-b);

}

// signum function since missing in math.h (!?)
inline int sgn(double x)
{
    if (x==0)
        return 0;
    else
        return (x>0) ? 1 : -1;
}

#endif
