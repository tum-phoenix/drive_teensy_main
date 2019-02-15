#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/ImuData.hpp"
#include "phoenix_msgs/MotorState.hpp"
#include "phoenix_msgs/MotorTarget.hpp"
#include "phoenix_msgs/NodeState.hpp"
#include "phoenix_msgs/DriveState.hpp"
#include "phoenix_can_shield.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "vuart.h"

using namespace uavcan;
using namespace phoenix_msgs;

typedef struct {
  imu::Vector<3> lin_acc;
  imu::Vector<3> gyro;
  imu::Vector<3> euler;
} imu_t;

// publisher
Publisher<ImuData> *imuPublisher;
Publisher<MotorState> *motorStatePublisher;
Publisher<MotorTarget> *motorTargetPublisher;
Publisher<NodeState> *status_Publisher;
Publisher<DriveState> *drive_Publisher;


// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  status_Publisher = new Publisher<NodeState>(*node);
  imuPublisher = new Publisher<ImuData>(*node);
  motorStatePublisher = new Publisher<MotorState>(*node);
  motorTargetPublisher = new Publisher<MotorTarget>(*node);
  drive_Publisher = new Publisher<DriveState>(*node);

  // initiliaze publishers
  if(status_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize status_Publisher!");
  }

  if(imuPublisher->init() < 0)
  {
    Serial.println("Unable to initialize imuPublisher!");
  }

  if(motorStatePublisher->init() < 0)
  {
    Serial.println("Unable to initialize motorStatePublisher!");
  }

  if(motorTargetPublisher->init() < 0)
    {
      Serial.println("Unable to initialize motorStatePublisher!");
    }

  if(drive_Publisher->init() < 0)
    {
      Serial.println("Unable to initialize drive_Publisher!");
    }
  // set TX timeout
  imuPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  motorStatePublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  motorTargetPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  drive_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  status_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle BNO publisher
void cyclePublisherBNO(imu_t bno_data)
{
  ImuData msg;

  msg.timestamp = systemClock->getUtc();

  msg.lin_acceleration[0] = bno_data.lin_acc[0];
  msg.lin_acceleration[1] = bno_data.lin_acc[1];
  msg.lin_acceleration[2] = bno_data.lin_acc[3];

  msg.rate_gyro[0] = bno_data.gyro[0];
  msg.rate_gyro[1] = bno_data.gyro[1];
  msg.rate_gyro[2] = bno_data.gyro[2];

  msg.euler[0] = bno_data.euler[0];
  msg.euler[1] = bno_data.euler[1];
  msg.euler[2] = bno_data.euler[2];

  if (imuPublisher->broadcast(msg) < 0)
  {
    Serial.println("Error while broadcasting key message");
  }
}

// cycle Motor publisher
void cyclePublisher_Mot_State(bldcMeasure data, uint8_t motor_position)
{
  MotorState msg;
  msg.position =      motor_position; // MotorState::POS_FRONT_RIGHT;
  msg.motor_current = data.avgMotorCurrent;
  msg.input_current = data.avgInputCurrent;
  msg.erpm =          data.erpm;

  //SerialPrint(measuredVal_motor1);

  if (motorStatePublisher->broadcast(msg) < 0)
  {
    Serial.print("Error while broadcasting motor state ");
    Serial.println(motor_position);
  } else {
    digitalWrite(trafficLedPin, HIGH);
  }
}

void cyclePublisher_Actor_Comms(actor_comm_t data)
{
  MotorTarget msg;
  msg.motor_arm = actor_comms.mot_arm;
  msg.mot_cur_type = actor_comms.mot_cur_type;
  msg.servo_attach = actor_comms.servo_arm;
  msg.current_front_left = data.motor_amps[FRONT_LEFT];
  msg.current_front_right = data.motor_amps[FRONT_RIGHT];
  msg.current_rear_left = data.motor_amps[REAR_LEFT];
  msg.current_rear_right = data.motor_amps[REAR_RIGHT];
  
  msg.servo_front_left = data.servo_angles[FRONT_LEFT];
  msg.servo_front_right = data.servo_angles[FRONT_RIGHT];
  msg.servo_rear_left = data.servo_angles[REAR_LEFT];
  msg.servo_rear_right = data.servo_angles[REAR_RIGHT];

  if (motorTargetPublisher->broadcast(msg) < 0)
  {
    Serial.println("Error while broadcasting motor commads");
  } else {
    digitalWrite(trafficLedPin, HIGH);
  }
}

// cycle status publisher
void cyclePublisher_Status()
{
  // Status
  if(last_status_update +
    MonotonicDuration::fromMSec(1000/(float)status_update_rate) <
    systemClock->getMonotonic())
  {
    last_status_update = systemClock->getMonotonic();
    float V4_raw = analogRead(CELL4_PIN);
    float curr_raw = analogRead(CURR_PIN);
    float V4 = V4_raw * Cell4_FACTOR;
    float curr = curr_raw * CURR_FACTOR;

    NodeState msg;
    msg.id= nodeID;
    msg.voltage = V4; 
    msg.current = curr;
    msg.fault_code_1 = 0;
    msg.fault_code_2 = 0;

    if (status_Publisher->broadcast(msg) < 0)
    {
      Serial.println("Error while broadcasting status state");
    } else {
      digitalWrite(trafficLedPin, HIGH);
    }
  }
}

void cyclePublisher_Drive_State(float v, float s_f, float s_r)
{
  DriveState msg;
  msg.v = v;
  msg.steer_f = s_f;
  msg.steer_r = s_r;
  msg.arm = (RC_coms.drive_state > 1);
  if (drive_Publisher->broadcast(msg) < 0)
  {
    Serial.println("Error while broadcasting drive State comand");
  } else {
    digitalWrite(trafficLedPin, HIGH);
  }
}

#endif
