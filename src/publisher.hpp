#ifndef	PUBLISHER_HPP
#define	PUBLISHER_HPP

#include <UAVCAN.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

using namespace uavcan;

// publisher
Publisher<protocol::debug::LogMessage> *logPublisher;
Publisher<protocol::debug::KeyValue> *keyPublisher;


void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  logPublisher = new Publisher<protocol::debug::LogMessage>(*node);
  keyPublisher = new Publisher<protocol::debug::KeyValue>(*node);

  // initiliaze publishers
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
}


void cyclePublisher()
{
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


#endif
