#ifndef	PUBLISHER_HPP
#define	PUBLISHER_HPP

#include <UAVCAN.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include "phoenix_can_shield.h"

using namespace uavcan;

// publisher
Publisher<protocol::debug::KeyValue> *keyPublisher;


void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  keyPublisher = new Publisher<protocol::debug::KeyValue>(*node);

  // initiliaze publishers
  if(keyPublisher->init() < 0)
  {
    Serial.println("Unable to initialize key message publisher!");
  }

  // set TX timeout
  keyPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

static int counter = 0;
MonotonicTime lastPub = MonotonicTime::fromMSec(0);

void cyclePublisher(const int pubFreq)
{
  // send everyone the truth
  if(lastPub + MonotonicDuration::fromMSec(1000/(float)pubFreq) < systemClock->getMonotonic())
  {
    {
      protocol::debug::KeyValue msg;

      msg.value = counter++;
      msg.key = "this is a longer message for testing with longer messages";

      //Serial.print("Trans: ");
      //Serial.println(msg.value);

      const int pres = keyPublisher->broadcast(msg);
      if (pres < 0)
      {
        Serial.println("Error while broadcasting key message");
      }
    }
    lastPub = systemClock->getMonotonic();
  }
}


#endif
