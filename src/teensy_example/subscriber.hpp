#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include "phoenix_can_shield.h"

using namespace uavcan;

Subscriber<protocol::debug::KeyValue> *keySubscriber;
Subscriber<equipment::ahrs::RawIMU> *imuSubscriber;

MonotonicTime last_time;

void keyMessageCallback(const uavcan::protocol::debug::KeyValue& msg)
{
  Serial.print("Key Diff: ");
  MonotonicTime curr_time = systemClock->getMonotonic();

  Serial.println((uint32_t)(curr_time - last_time).toUSec());
  last_time = curr_time;

}

void imuMessageCallback(const equipment::ahrs::RawIMU& msg)
{
  Serial.println("Imu Callback");
}


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  keySubscriber = new Subscriber<protocol::debug::KeyValue>(*node);
  imuSubscriber = new Subscriber<equipment::ahrs::RawIMU>(*node);

  if(keySubscriber->start(keyMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }

  if(imuSubscriber->start(imuMessageCallback) < 0)
  {
    Serial.println("Unable to start imu message subscriber!");
  }  
}

#endif
