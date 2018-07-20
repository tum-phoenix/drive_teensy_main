#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/RemoteControl.hpp"

using namespace uavcan;
using namespace phoenix_msgs;

Subscriber<RemoteControl> *motorSubscriber;

void motorMessageCallback(const RemoteControl& msg)
{
  
}


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  motorSubscriber = new Subscriber<RemoteControl>(*node);

  if(motorSubscriber->start(motorMessageCallback) < 0)
  {
    Serial.println("Unable to start subscriber!");
  }
}

#endif
