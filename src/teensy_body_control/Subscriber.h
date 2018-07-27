#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/RemoteControl.hpp"
#include <VescUart.h>

using namespace uavcan;
using namespace phoenix_msgs;

Subscriber<RemoteControl> *remote_control_Subscriber;


void remote_control_callback(const RemoteControl& msg)
{
  // TODO

  RC_coms.thr = msg.velocity;
  RC_coms.steer_f = msg.steer_front;
  RC_coms.steer_r = msg.steer_rear;
  RC_coms.drive_state = msg.drive_mode;
  
}


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  remote_control_Subscriber = new Subscriber<RemoteControl>(*node);

  if(remote_control_Subscriber->start(remote_control_callback) < 0)
  {
    Serial.println("Unable to start subscriber!");
  }
}

#endif
