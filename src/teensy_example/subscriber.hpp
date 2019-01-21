#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include "phoenix_can_shield.h"

using namespace uavcan;

Subscriber<protocol::debug::KeyValue> *keySubscriber;

MonotonicTime last_time;

void keyMessageCallback(const uavcan::protocol::debug::KeyValue &msg)
{
  Serial.print("Key Diff: ");
  MonotonicTime curr_time = systemClock->getMonotonic();

  Serial.println((uint32_t)(curr_time - last_time).toUSec());
  last_time = curr_time;
}

void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  keySubscriber = new Subscriber<protocol::debug::KeyValue>(*node);

  if (keySubscriber->start(keyMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }
}

#endif
