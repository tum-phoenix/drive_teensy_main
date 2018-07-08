#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <UAVCAN.hpp>
#include "drive_teensy_uavcan_msgs/RemoteControl.hpp"
#include "phoenix_can_shield.h"
#include "DJI.h"

using namespace uavcan;
using namespace drive_teensy_uavcan_msgs;

// publisher
Publisher<RemoteControl> *rcPublisher;

// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  rcPublisher = new Publisher<RemoteControl>(*node);

  // initiliaze publishers
  if(rcPublisher->init() < 0)
  {
    Serial.println("Unable to initialize publisher!");
  }

  // set TX timeout
  rcPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher(DJI& dji)
{

  // turn off traffic led pin
  digitalWrite(trafficLedPin, LOW);

  // remote control
  if(dji.read())
  {
    RemoteControl msg;

    switch(dji.leftSwitch())
    {
      case DJI::DOWN:   msg.mode = RemoteControl::MODE_MANUAL; break;
      case DJI::MIDDLE: msg.mode = RemoteControl::MODE_SEMI_AUTONOMOUS; break;
      case DJI::UP:     msg.mode = RemoteControl::MODE_AUTONOMOUS; break;
      default:          msg.mode = RemoteControl::MODE_RC_DISCONNECTED; break;
    }

    msg.velocity_front = dji.leftVerticalStick();

    const int pres = rcPublisher->broadcast(msg);
    if (pres < 0)
    {
      Serial.println("Error while broadcasting message");
    }else{
      digitalWrite(trafficLedPin, HIGH);
    }
  }
}


#endif
