#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <UAVCAN.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

using namespace uavcan;

Subscriber<protocol::debug::KeyValue> *keySubscriber;

void keyMessageCallback(const uavcan::protocol::debug::KeyValue& msg)
{
  Serial.print("Recei: ");
  Serial.println(msg.value);
}


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  keySubscriber = new Subscriber<protocol::debug::KeyValue>(*node);

  if(keySubscriber->start(keyMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }
}

#endif
