#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/ImuData.hpp"
#include "phoenix_msgs/MotorState.hpp"
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

// Vesc
int motor_state_update_rate = 10; // Hz
MonotonicTime last_motor_state_update = MonotonicTime::fromMSec(0);

// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  imuPublisher = new Publisher<ImuData>(*node);
  motorStatePublisher = new Publisher<MotorState>(*node);

  // initiliaze publishers
  if(imuPublisher->init() < 0)
  {
    Serial.println("Unable to initialize imuPublisher!");
  }

  if(motorStatePublisher->init() < 0)
  {
    Serial.println("Unable to initialize motorStatePublisher!");
  }

  // set TX timeout
  imuPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher(imu_t bno_data)
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

  // motor state update -> at motor_state_update_rate -> check time first
  if(last_motor_state_update +
     MonotonicDuration::fromMSec(1000/(float)motor_state_update_rate) <
     systemClock->getMonotonic())
  {

    // it is time for an update of motor states
    last_motor_state_update = systemClock->getMonotonic();


    // update motor 1 information
    bldcMeasure measuredVal_motor1;
    if (VescUartGetValue(measuredVal_motor1, 0)) {
      MotorState msg;
      msg.position =      MotorState::POS_FRONT_RIGHT;
      msg.temp_fet =      measuredVal_motor1.tempFetFiltered;
      msg.motor_current = measuredVal_motor1.avgMotorCurrent;
      msg.input_current = measuredVal_motor1.avgInputCurrent;
      msg.input_voltage = measuredVal_motor1.inpVoltage;
      msg.rpm =           measuredVal_motor1.rpm / 7; // RPM = ERPM / 7
      msg.fault_code =    measuredVal_motor1.faultCode;

      //SerialPrint(measuredVal_motor1);

      if (motorStatePublisher->broadcast(msg) < 0)
      {
        Serial.println("Error while broadcasting motor 1 state");
      } else {
        digitalWrite(trafficLedPin, HIGH);
      }
    }
    else
    {
      Serial.print("FAILED to get motor 1 data! ReadError: ");
      Serial.print(Serial1.getReadError());
      Serial.print(" WriteError: ");
      Serial.println(Serial1.getWriteError());
      // Serial1.end();
      // Serial1.clear();
      // Serial1.flush();
      // Serial1.begin(115200);
    }
  }
}


#endif
