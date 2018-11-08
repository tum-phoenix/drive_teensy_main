#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/MotorTarget.hpp"

using namespace uavcan;
using namespace phoenix_msgs;

// subscribing tasks:
// we want to subscribe the MotorTarget messages to set the motors.


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  
}

#endif