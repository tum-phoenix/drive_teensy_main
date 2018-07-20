#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/MotorTarget.hpp"

using namespace uavcan;
using namespace phoenix_msgs;

// subscribing tasks:
// we want to subscribe the MotorTarget messages to set the motors.

Subscriber<MotorTarget> *motor_target_Subscriber;

void motor_target_callback(const MotorTarget& msg)
{
  // validate if motors are ready for these commands:
  if (measuredVal_motor0.tempFetFiltered < max_fet_temperature) {
    VescUartSetCurrent((float)msg.current_front_left, 0);
  } else {
    Serial.println("overtemperature for motor 0");
  }

  if (measuredVal_motor1.tempFetFiltered < max_fet_temperature) {
    VescUartSetCurrent((float)msg.current_front_right, 1);
  } else {
    Serial.println("overtemperature for motor 1");
  }

  // set servos
  steering_servo_position_0 = steering_servo_offset_0 + (float)msg.servo_front_left;
  steering_servo_0.write(steering_servo_position_0);

  steering_servo_position_1 = steering_servo_offset_1 + (float)msg.servo_front_right;
  steering_servo_1.write(steering_servo_position_1);
}


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  motor_target_Subscriber = new Subscriber<MotorTarget>(*node);

  if(motor_target_Subscriber->start(motor_target_callback) < 0)
  {
    Serial.println("Unable to start motor_target_Subscriber!");
  }
}

#endif