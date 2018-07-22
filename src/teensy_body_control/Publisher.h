#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/ImuData.hpp"
#include "phoenix_msgs/MotorState.hpp"
#include "phoenix_msgs/ActorCommands.hpp"
#include "phoenix_can_shield.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <VescUart.h>

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
Publisher<ActorCommands> *actorCommandsPublisher;


// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  imuPublisher = new Publisher<ImuData>(*node);
  motorStatePublisher = new Publisher<MotorState>(*node);
  actorCommandsPublisher = new Publisher<ActorCommands>(*node);

  // initiliaze publishers
  if(imuPublisher->init() < 0)
  {
    Serial.println("Unable to initialize imuPublisher!");
  }

  if(motorStatePublisher->init() < 0)
  {
    Serial.println("Unable to initialize motorStatePublisher!");
  }

  if(actorCommandsPublisher->init() < 0)
    {
      Serial.println("Unable to initialize motorStatePublisher!");
    }
  // set TX timeout
  imuPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  motorStatePublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  actorCommandsPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle BNO publisher
void cyclePublisherBNO(imu_t bno_data)
{
  ImuData msg;

  msg.timestamp = systemClock->getUtc();

  msg.accelerometer[0] = bno_data.lin_acc[0];
  msg.accelerometer[1] = bno_data.lin_acc[1];
  msg.accelerometer[2] = bno_data.lin_acc[3];

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
  msg.temp_fet =      data.tempFetFiltered;
  msg.motor_current = data.avgMotorCurrent;
  msg.input_current = data.avgInputCurrent;
  msg.input_voltage = data.inpVoltage;
  msg.rpm =           data.rpm / 7; // RPM = ERPM / 7
  msg.fault_code =    data.faultCode;

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
  ActorCommands msg;
  msg.motor_current1 = data.motor_amps[0];
  msg.motor_current2 = data.motor_amps[1];
  msg.motor_current3 = data.motor_amps[2];
  msg.motor_current4 = data.motor_amps[3];
  
  msg.servo_angle1 = data.servo_angles[0];
  msg.servo_angle2 = data.servo_angles[1];
  msg.servo_angle3 = data.servo_angles[2];
  msg.servo_angle4 = data.servo_angles[3];

  if (actorCommandsPublisher->broadcast(msg) < 0)
  {
    Serial.println("Error while broadcasting motor commads");
  } else {
    digitalWrite(trafficLedPin, HIGH);
  }
}
#endif
