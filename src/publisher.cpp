#include <UAVCAN.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

using namespace uavcan;

static const unsigned NodeMemoryPoolSize = 8192;
static constexpr uint32_t bitrate = 1000000;
static constexpr uint32_t nodeID = 101;

Node<NodeMemoryPoolSize> *node;
Publisher<protocol::debug::LogMessage> *logPublisher;


ISystemClock& getSystemClock()
{
  Serial.println("SystemClock get");
  return uavcan_nxpk20::SystemClock::instance();
}

ICanDriver& getCanDriver()
{
  Serial.println("CanDriver get");
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
  Serial.println("Setup");

  // Create a node
  node = new Node<NodeMemoryPoolSize>(getCanDriver(), getSystemClock());
  Serial.println("Application node created");
  // Create a publisher for LogMessages
  logPublisher = new Publisher<protocol::debug::LogMessage>(*node);

  node->setNodeID(nodeID);
  node->setName("org.phoenix.publisher");

  // Start Node and then the subscriber
  if(node->start() < 0)
  {
    Serial.println("Unable to start node!");
  }
  if(logPublisher->init() < 0)
  {
    Serial.println("Unable to initialize log message publisher!");
  }

  node->setModeOperational();
}

void loop()
{
  // Wait for frames 100ms, then do other stuff
  Serial.println("Application spinning");
  const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
  if (res < 0)
  {
      Serial.println("Error while spinning...");
  }

  protocol::debug::LogMessage msg;
  const int pres = logPublisher->broadcast(msg);
  if (pres < 0)
  {
    Serial.println("Error while broadcasting message");
  }
}
