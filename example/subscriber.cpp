#include <UAVCAN.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

using namespace uavcan;


static const unsigned NodeMemoryPoolSize = 8192;
static constexpr uint32_t bitrate = 1000000;
static constexpr uint32_t nodeID = 100;

Node<NodeMemoryPoolSize> *node;
Subscriber<protocol::debug::LogMessage> *logSubscriber;

ISystemClock& getSystemClock()
{
    return uavcan_nxpk20::SystemClock::instance();
}

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

void background_processing()
{
  ; // do nothing
}

void logMessageCallback(const uavcan::protocol::debug::LogMessage& msg)
{
  Serial.println("Received log message");
}


void setup()
{
  delay(3000);
  Serial.begin(9600);
  node = new Node<NodeMemoryPoolSize>(getCanDriver(), getSystemClock());
  Serial.println("Application node created");

  node->setNodeID(nodeID);
  node->setName("org.phoenix.subscriber");

  // Create a subsciber for LogMessages
  logSubscriber = new Subscriber<protocol::debug::LogMessage>(*node);

  // Start Node and then the subscriber
  if(node->start() < 0)
  {
    Serial.println("Unable to start node!");
  }
  if(logSubscriber->start(logMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }

  node->setModeOperational();
}

void loop()
{
  // Wait for frames 100ms, then do other stuff
  Serial.println("Application start spinning");
  const int res = node->spin(uavcan::MonotonicDuration::fromMSec(100));
  if (res < 0)
  {
      Serial.println("Error while spinning...");
  }
  // do other stuff
  background_processing();
}
