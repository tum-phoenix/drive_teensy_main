#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/MotorTarget.hpp"
#include "phoenix_msgs/MotorConfig.hpp"
#include "Publisher.h"

using namespace uavcan;
using namespace phoenix_msgs;

// subscribing tasks:
// we want to subscribe the MotorTarget messages to set the motors.

Subscriber<MotorTarget> *motor_target_Subscriber;
Subscriber<MotorConfig> *mcconf_Subscriber;

void motor_config_callback(const MotorConfig& msg) {
    mcconf.max_current = (float)(msg.max_motor_current / MOTOR_Y_WIND_FACTOR);
    mcconf.min_current = (float)(msg.max_motor_current_brake/ MOTOR_Y_WIND_FACTOR);
    mcconf.min_erpm    = - msg.max_erpm;
    mcconf.max_erpm    = msg.max_erpm;
    custom_vesc_config_set = false;
}

void motor_target_callback(const MotorTarget& msg) {
    if (msg.motor_arm == MotorTarget::MOTORS_ON) {
        if (msg.setpoint_type == MotorTarget::ACCELERATION) {
            VescUartSetCurrent(msg.setpoint_rear_left, 0);
            VescUartSetCurrent(msg.setpoint_rear_right, 1);
        } else if (msg.setpoint_type == MotorTarget::REG_BRAKE) {
            VescUartSetCurrentBrake(fabsf(msg.setpoint_rear_left), 0);
            VescUartSetCurrentBrake(fabsf(msg.setpoint_rear_right), 1);
        } else if (msg.setpoint_type == MotorTarget::HANDBRAKE) {
            VescUartSetHandbrake(fabsf(msg.setpoint_rear_left), 0);
            VescUartSetHandbrake(fabsf(msg.setpoint_rear_right), 1);
        } else {
            VescUartSetCurrent(0, 0);
            VescUartSetCurrent(0, 1);
        }
    } else {
        VescUartSetCurrent(0, 0);
        VescUartSetCurrent(0, 1);
    }

    if (msg.servo_attach == MotorTarget::SERVOS_ON) {
        if (!steering_servo_3.attached()) steering_servo_3.attach(steering_servo_3_pin);
        if (!steering_servo_4.attached()) steering_servo_4.attach(steering_servo_4_pin);

        steering_servo_3.write(steering_servo_offset_3 + msg.servo_rear_left);
        steering_servo_4.write(steering_servo_offset_4 + msg.servo_rear_right);
    } else {
        steering_servo_3.detach();
        steering_servo_4.detach();
    }

    last_motor_target_receive = systemClock->getMonotonic();
}


void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  motor_target_Subscriber = new Subscriber<MotorTarget>(*node);
  mcconf_Subscriber = new Subscriber<MotorConfig>(*node);

  if(motor_target_Subscriber->start(motor_target_callback) < 0)
  {
    Serial.println("Unable to start motor_target_Subscriber!");
  }

  if(mcconf_Subscriber->start(motor_config_callback) < 0)
  {
    Serial.println("Unable to start motor_config_Subscriber!");
  }
}

#endif