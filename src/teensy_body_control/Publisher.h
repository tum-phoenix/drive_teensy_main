#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "uavcan/equipment/ahrs/RawIMU.hpp"
#include "phoenix_can_shield.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

using namespace uavcan;

typedef struct {
  imu::Vector<3> lin_acc;
  imu::Vector<3> gyro;
  imu::Vector<3> euler;
} imu_t;

// publisher
Publisher<equipment::ahrs::RawIMU> *imuPublisher;

// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  imuPublisher = new Publisher<equipment::ahrs::RawIMU>(*node);

  // initiliaze publishers
  if(imuPublisher->init() < 0)
  {
    Serial.println("Unable to initialize imuPublisher!");
  }

  // set TX timeout
  imuPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher(imu_t bno_data)
{

  // turn off traffic led pin
  digitalWrite(trafficLedPin, LOW);
}


#endif
