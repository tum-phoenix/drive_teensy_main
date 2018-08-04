#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/RemoteControl.hpp"
#include "phoenix_msgs/MotorState.hpp"
#include "phoenix_can_shield.h"
#include <VescUart.h>

using namespace uavcan;
using namespace phoenix_msgs;

// publishing tasks:
// we want to publish the rc readings from dji via a RemoteControl Messages
// we want to publish the state of the two motors via two MotorState Messages

// publisher
Publisher<RemoteControl> *rc_Publisher;
Publisher<MotorState> *motor_state_Publisher;

// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  rc_Publisher = new Publisher<RemoteControl>(*node);
  motor_state_Publisher = new Publisher<MotorState>(*node);

  // initiliaze publishers
  if(rc_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize rc_Publisher!");
  }
  if(motor_state_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize motor_state_Publisher!");
  }

  // set TX timeout
  rc_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  motor_state_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher(DJI& dji)
{
  // // motor state update -> at motor_state_update_rate -> check time first
  // if(last_motor_state_update +
  //    MonotonicDuration::fromMSec(1000/(float)motor_state_update_rate) <
  //    systemClock->getMonotonic())
  // {
  //   // it is time for an update of motor states
  //   last_motor_state_update = systemClock->getMonotonic();

  //   // update motor 0 information
  //   if (VescUartGetValue(measuredVal_motor0, 0)) {
  // 		MotorState msg;
  //     msg.position = motor0_position;
  //     msg.temp_fet = measuredVal_motor0.tempFetFiltered;
  //     msg.motor_current = measuredVal_motor0.avgMotorCurrent;
  //     msg.input_current = measuredVal_motor0.avgInputCurrent;
  //     msg.input_voltage = measuredVal_motor0.inpVoltage;
  //     msg.rpm = measuredVal_motor0.rpm;
  //     msg.fault_code = measuredVal_motor0.faultCode;
  //     const int pres = motor_state_Publisher->broadcast(msg);
  //     if (pres < 0)
  //     {
  //       Serial.println("Error while broadcasting motor 0 state");
  //     } else {
  //       digitalWrite(trafficLedPin, HIGH);
  //     }
  // 	}
  // 	else
  // 	{
  // 		Serial.println("Failed to get motor 0 data!");
  // 	}

  //   // update motor 1 information
  //   if (VescUartGetValue(measuredVal_motor1, 1)) {
  // 		MotorState msg;
  //     msg.position = motor1_position;
  //     msg.temp_fet = measuredVal_motor1.tempFetFiltered;
  //     msg.motor_current = measuredVal_motor1.avgMotorCurrent;
  //     msg.input_current = measuredVal_motor1.avgInputCurrent;
  //     msg.input_voltage = measuredVal_motor1.inpVoltage;
  //     msg.rpm = measuredVal_motor1.rpm;
  //     msg.fault_code = measuredVal_motor1.faultCode;
  //     const int pres = motor_state_Publisher->broadcast(msg);
  //     if (pres < 0)
  //     {
  //       Serial.println("Error while broadcasting motor 1 state");
  //     } else {
  //       digitalWrite(trafficLedPin, HIGH);
  //     }
  // 	}
  // 	else
  // 	{
  // 		Serial.println("Failed to get motor 1 data!");
  // 	}
  // }

  // remote control update -> at rc_update_rate -> check time first
  if(last_rc_update +
     MonotonicDuration::fromMSec(1000/(float)rc_update_rate) <
     systemClock->getMonotonic())
  {
    // it is time for an update of the rc readings
    last_rc_update = systemClock->getMonotonic();

    // update dji remote control readings
    if(dji.read()) {
      RemoteControl msg;

      // left switch - drive mode select
      switch(dji.leftSwitch())
      {
        case DJI::DOWN:   msg.drive_mode = RemoteControl::DRIVE_MODE_MANUAL; break;
        case DJI::MIDDLE: msg.drive_mode = RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS; break;
        case DJI::UP:     msg.drive_mode = RemoteControl::DRIVE_MODE_AUTONOMOUS; break;
        default:          msg.drive_mode = RemoteControl::DRIVE_MODE_RC_DISCONNECTED; break;
      }

      // right switch - aux mode select
      switch(dji.rightSwitch())
      {
        case DJI::DOWN:   msg.aux_mode = RemoteControl::AUX_MODE_UP; break;
        case DJI::MIDDLE: msg.aux_mode = RemoteControl::AUX_MODE_CENTER; break;
        case DJI::UP:     msg.aux_mode = RemoteControl::AUX_MODE_DOWN; break;
        default:          msg.aux_mode = RemoteControl::AUX_MODE_RC_DISCONNECTED; break;
      }

      // left stick - up/down - target velocity
      msg.velocity = dji.leftVerticalStick();

      //Serial.print("Vel: ");
      //Serial.println((float)msg.velocity * 10);

      // left stick - left/right - steering rear
      msg.steer_rear = dji.leftHorizontalStick();

      // right stick - left/right - front rear
      msg.steer_front = dji.rightHorizontalStick();

      const int pres = rc_Publisher->broadcast(msg);
      if (pres < 0)
      {
      //  Serial.println("Error while broadcasting rc message");
      } else {
        digitalWrite(trafficLedPin, HIGH);
      }
    }
  }
}


#endif
