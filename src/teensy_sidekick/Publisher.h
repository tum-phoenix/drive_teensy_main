#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/NodeState.hpp"
#include "phoenix_msgs/PowerBoard.hpp"
#include "phoenix_can_shield.h"

using namespace uavcan;
using namespace phoenix_msgs;

// publishing tasks:
// we want to publish the rc readings from dji via a RemoteControl Messages
// we want to publish the state of the two motors via two MotorState Messages

// publisher
Publisher<NodeState> *status_Publisher;
Publisher<PowerBoard> *powerboard_Publisher;


// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  status_Publisher = new Publisher<NodeState>(*node);
  powerboard_Publisher = new Publisher<PowerBoard>(*node);

  // initiliaze publishers
  if(status_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize status_Publisher!");
  }
  if(powerboard_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize powerboard_Publisher!");
  }

  // set TX timeout
  status_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  powerboard_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher()
{
  // Power
  if(last_power_update +
    MonotonicDuration::fromMSec(1000/(float)power_update_rate) <
    systemClock->getMonotonic())
  {
    last_power_update = systemClock->getMonotonic();
    float V4_raw = analogRead(CELL4_PIN);
    float curr_raw = analogRead(OWN_CURR_PIN);
    float V4 = V4_raw * Cell4_FACTOR;
    float curr = curr_raw * OWN_CURR_FACTOR;

    NodeState msg;
    msg.id= nodeID;
    msg.voltage = V4;
    msg.current = curr;
    if (status_Publisher->broadcast(msg) < 0)
    {
      Serial.println("Error while broadcasting power state");
    } else {
      digitalWrite(trafficLedPin, HIGH);
    }
    
    float curr_ser_raw = analogRead(SER_CURR_PIN);
    float curr_jt_raw  = analogRead(JT_CURR_PIN);
    float curr_nuc_raw = analogRead(NUC_CURR_PIN);
    float v_ser_raw    = analogRead(SER_V_PIN);
    float v_jt_raw     = analogRead(JT_V_PIN);
    float v_nuc_raw    = analogRead(NUC_V_PIN);
    PowerBoard msg1;
    msg1.curr_Servo = curr_ser_raw * SER_CURR_FACTOR;
    msg1.curr_JT    = curr_jt_raw  * JT_CURR_FACTOR;
    msg1.curr_NUC   = curr_nuc_raw * NUC_CURR_FACTOR;
    msg1.v_Servo    = v_ser_raw    * SER_V_FACTOR;
    msg1.v_JT       = v_jt_raw     * JT_V_FACTOR;
    msg1.v_NUC      = v_nuc_raw    * NUC_V_FACTOR;
    const int pres1 = powerboard_Publisher->broadcast(msg1);
    if (pres1 < 0)
    {
      Serial.println("Error while broadcasting power board");
    } else {
      digitalWrite(trafficLedPin, HIGH);
    }
    
    
    
    
    
  }
}


#endif
