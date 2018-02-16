#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <UAVCAN.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

using namespace uavcan;

Subscriber<protocol::debug::LogMessage> *logSubscriber;
Subscriber<protocol::debug::KeyValue> *keySubscriber;


void logMessageCallback(const uavcan::protocol::debug::LogMessage& msg)
{
  Serial.println("Received log message");
}

void keyMessageCallback(const uavcan::protocol::debug::KeyValue& msg)
{
  Serial.println("Received key message!");
  Serial.print("Value:");
  Serial.println(msg.value);
}



void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  logSubscriber = new Subscriber<protocol::debug::LogMessage>(*node);
  keySubscriber = new Subscriber<protocol::debug::KeyValue>(*node);

  // start subscriber
  if(logSubscriber->start(logMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }

  if(keySubscriber->start(keyMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }
}

#endif
