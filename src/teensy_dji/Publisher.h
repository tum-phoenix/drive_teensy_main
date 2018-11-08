#ifndef	PUBLISHER_H
#define	PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/RemoteControl.hpp"
#include "phoenix_msgs/MotorState.hpp"
#include "phoenix_msgs/PowerState.hpp"
#include "phoenix_msgs/ParallelParking.hpp"
#include "phoenix_msgs/UserButtons.hpp"
#include "phoenix_can_shield.h"
#include <VescUart.h>
#include <Filters.h>

using namespace uavcan;
using namespace phoenix_msgs;

// publishing tasks:
// we want to publish the rc readings from dji via a RemoteControl Messages
// we want to publish the state of the two motors via two MotorState Messages


// filters out changes faster that 5 Hz.
float filterFrequency = 10.0;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilterFront(LOWPASS, filterFrequency); 
FilterOnePole lowpassFilterRear(LOWPASS, filterFrequency); 

// publisher
Publisher<RemoteControl> *rc_Publisher;
Publisher<MotorState> *motor_state_Publisher;
Publisher<PowerState> *power_Publisher;
Publisher<ParallelParking> *ppark_Publisher;
Publisher<UserButtons> *user_buttons_Publisher;

// additional configuration
static uint8_t motor3_position = MotorState::POS_REAR_LEFT;
static uint8_t motor4_position = MotorState::POS_REAR_RIGHT;

// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  rc_Publisher = new Publisher<RemoteControl>(*node);
  motor_state_Publisher = new Publisher<MotorState>(*node);
  power_Publisher = new Publisher<PowerState>(*node);
  ppark_Publisher = new Publisher<ParallelParking>(*node);
  user_buttons_Publisher = new Publisher<UserButtons>(*node);

  // initiliaze publishers
  if(rc_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize rc_Publisher!");
  }
  if(motor_state_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize motor_state_Publisher!");
  }  
  if(power_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize power_Publisher!");
  }
  if(ppark_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize ppark_Publisher!");
  }
  if(user_buttons_Publisher->init() < 0)
  {
    Serial.println("Unable to initialize user_buttons_Publisher!");
  }

  // set TX timeout
  rc_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  motor_state_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  power_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  ppark_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  user_buttons_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher(DJI& dji)
{
  // Cell Voltages
   if(last_power_update +
      MonotonicDuration::fromMSec(1000/(float)power_update_rate) <
      systemClock->getMonotonic())
   {
    last_power_update = systemClock->getMonotonic();

    float V1_raw = analogRead(CELL1_PIN);
    float V2_raw = analogRead(CELL2_PIN);
    float V3_raw = analogRead(CELL3_PIN);
    float V4_raw = analogRead(CELL4_PIN);
    float curr_raw = analogRead(CURR_PIN);
    float V1 = V1_raw * Cell1_FACTOR;
    float V2 = V2_raw * Cell2_FACTOR;
    float V3 = V3_raw * Cell3_FACTOR;
    float V4 = V4_raw * Cell4_FACTOR;
    float curr = curr_raw * CURR_FACTOR;

    PowerState msg;
    msg.id= nodeID;
    msg.v1= V1;
    msg.v2= V2-V1;
    msg.v3= V3-V2;
    msg.v4= V4-V3;
    msg.main_voltage = V4; 
    msg.main_current = curr;
    if (msg.v1 < V_ALM_FINAL || msg.v2 < V_ALM_FINAL || msg.v3 < V_ALM_FINAL || msg.v4 < V_ALM_FINAL) bat_alm = 1;

    const int pres = power_Publisher->broadcast(msg);
    if (pres < 0)
    {
      Serial.println("Error while broadcasting power state");
    } else {
      digitalWrite(trafficLedPin, HIGH);
    }
   }

   // motor state update -> at motor_state_update_rate -> check time first
   if(last_motor_state_update +
      MonotonicDuration::fromMSec(1000/(float)motor_state_update_rate) <
      systemClock->getMonotonic())
   {
     
    static uint32_t lastupdate = 0;
    static uint32_t thisupdate = 0;
    lastupdate = thisupdate;
    thisupdate = micros();

     // it is time for an update of motor states
     last_motor_state_update = systemClock->getMonotonic();

     // update motor 3 information
     if (VescUartGetValue(measuredVal_motor3, 0)) { 
   		MotorState msg;
       msg.position      = motor3_position;
       msg.temp_fet      = measuredVal_motor3.tempFetFiltered;
       msg.motor_current = measuredVal_motor3.avgMotorCurrent;
       msg.input_current = measuredVal_motor3.avgInputCurrent;
       msg.input_voltage = measuredVal_motor3.inpVoltage;
       msg.rpm           = measuredVal_motor3.rpm;
       msg.fault_code    = measuredVal_motor3.faultCode;
       const int pres = motor_state_Publisher->broadcast(msg);
       if (pres < 0)
       {
         Serial.println("Error while broadcasting motor 3 state");
       } else {
         digitalWrite(trafficLedPin, HIGH);
       }
   	}
   	else
   	{
   		//Serial.println("Failed to get motor 3 data!");
   	}

     // update motor 4 information
     if (VescUartGetValue(measuredVal_motor4, 1)) {
   		MotorState msg;
       msg.position      = motor4_position;
       msg.temp_fet      = measuredVal_motor4.tempFetFiltered;
       msg.motor_current = measuredVal_motor4.avgMotorCurrent;
       msg.input_current = measuredVal_motor4.avgInputCurrent;
       msg.input_voltage = measuredVal_motor4.inpVoltage;
       msg.rpm           = measuredVal_motor4.rpm;
       msg.fault_code    = measuredVal_motor4.faultCode;
       const int pres = motor_state_Publisher->broadcast(msg);
       if (pres < 0)
       {
         Serial.println("Error while broadcasting motor 4 state");
       } else {
         digitalWrite(trafficLedPin, HIGH);
       }
   	}
   	else
   	{
   		//Serial.println("Failed to get motor 4 data!");
   	}
    
    #define WHEEL_RADIUS_M 0.033
    float mean_rounds = ((float)measuredVal_motor3.rpm + (float)measuredVal_motor4.rpm)/14; // RPM
    mean_rounds /= 60; // RPS
    rear.speed = (uint32_t)(mean_rounds * 2000 * M_PI * WHEEL_RADIUS_M); // mm/s;
    uint32_t dt = thisupdate-lastupdate; // micros
    float ds = (rear.speed * dt)/1000000; // mm
    float temp = (float)rear.dist_trav + ds;
    rear.dist_trav = (uint32_t)temp;
   }

  // remote control update -> at rc_update_rate -> check time first
  if(last_rc_update +
     MonotonicDuration::fromMSec(1000/(float)rc_update_rate) <
     systemClock->getMonotonic())
  {
    // it is time for an update of the rc readings
    last_rc_update = systemClock->getMonotonic();

    // update dji remote control readings
    if(dji.read()) {
      static RemoteControl msg;

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
      msg.velocity = dji.leftVerticalStick(msg.velocity);

      //Serial.print("Vel: ");
      //Serial.println((float)msg.velocity * 10);

      // left stick - left/right - steering rear
      lowpassFilterRear.input(dji.leftHorizontalStick(msg.steer_rear));
      msg.steer_rear = lowpassFilterRear.output();
      

      // right stick - left/right - front rear
      lowpassFilterFront.input(dji.rightHorizontalStick(msg.steer_front));
      msg.steer_front = lowpassFilterFront.output();
      
      const int pres = rc_Publisher->broadcast(msg);
      if (pres < 0)
      {
      //  Serial.println("Error while broadcasting rc message");
      } else {
        digitalWrite(trafficLedPin, HIGH);
      }
    }
  }

  // buttons
  if(last_button_update +
    MonotonicDuration::fromMSec(1000/(float)button_update_rate) <
    systemClock->getMonotonic())
  {
    // it is time for an update of the button readings
    last_button_update = systemClock->getMonotonic();
    analogReadRes(12);
    analogReadAveraging(10);
    #define R0 16
    float raw = analogRead(BUTTON_PIN);
    float r = raw/4096.0;
    r = 1-r;
    r = R0/r;
    r = r-R0;
    r = round(r);
    bit_but = (uint8_t)r ^ 0x1F; // invert bits for bitwise results
    //Serial.print(raw);
    //Serial.print("\t");
    //Serial.print(r);
    //Serial.print("\t");
    //Serial.println(bit_but, BIN);
    analogReadRes(12);
    analogReadAveraging(4);

    UserButtons msg;
    msg.bit_but = bit_but;

    const int pres = user_buttons_Publisher->broadcast(msg);
    if (pres < 0)
    {
      Serial.println("Error while broadcasting button message");
    } else {
      digitalWrite(trafficLedPin, HIGH);
    }
  }

  // parking lot
    if((publish_lot_msg == 10 || last_par_lot_update +
      MonotonicDuration::fromMSec(1000/(float)par_lot_update_rate) <
      systemClock->getMonotonic()) && publish_lot_msg > 0)
    {
      // it is time for an update of the parking lot readings
      last_par_lot_update = systemClock->getMonotonic();

      ParallelParking msg;
      msg.lot_size = lot_size;
      msg.dist_to_lot = rear.dist_trav - last_lot_pos;

      const int pres = ppark_Publisher->broadcast(msg);
      if (pres < 0)
      {
        Serial.println("Error while broadcasting parking message");
      } else {
        digitalWrite(trafficLedPin, HIGH);
      }
      publish_lot_msg --;
    }
}


#endif


void pf_ir_routine() {
  static int32_t start_odom;
  int32_t pos = rear.dist_trav;
  uint8_t state = digitalRead(PF_LS_PIN);
  sei();
  if (state == HIGH) {
    start_odom = pos;
    setRGBled(0,0,255);
  } else {
    lot_size = pos - start_odom;
    last_lot_pos = pos;
    publish_lot_msg = 10;
    setRGBled(255,255,255);
  }
}