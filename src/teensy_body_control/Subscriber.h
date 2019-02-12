#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/RemoteControl.hpp"
#include "phoenix_msgs/MotorState.hpp"
#include "phoenix_msgs/NucDriveCommand.hpp"
#include "phoenix_msgs/StartParking.hpp"
#include "phoenix_msgs/ParallelParking.hpp"
#include "vuart.h"

using namespace uavcan;
using namespace phoenix_msgs;

Subscriber<RemoteControl> *remote_control_Subscriber;
Subscriber<MotorState> *motor_state_Subscriber;
Subscriber<NucDriveCommand> *nuc_drive_Subscriber;
Subscriber<StartParking> *start_parking_Subscriber;
Subscriber<ParallelParking> *parallel_parking_Subscriber;


void remote_control_callback(const RemoteControl& msg)
{
  RC_coms.thr = msg.velocity;
  RC_coms.steer_f = -msg.steer_front;
  RC_coms.steer_r = -msg.steer_rear;
  RC_coms.drive_state = msg.drive_mode;
  RC_coms. aux_mode = msg.aux_mode;
  RC_coms.last_update = millis();
  
}

void Motor_State_callback(const MotorState& msg)
{
  if (msg.position == MotorState::POS_REAR_LEFT) {
    measuredVal_motor[REAR_LEFT].inpVoltage = msg.input_voltage;
    measuredVal_motor[REAR_LEFT].avgMotorCurrent = msg.motor_current;
    measuredVal_motor[REAR_LEFT].avgInputCurrent = msg.input_current;
    measuredVal_motor[REAR_LEFT].erpm = msg.erpm;
    last_mot_state_update[MotorState::POS_REAR_LEFT] = millis();
  } else if (msg.position == MotorState::POS_REAR_RIGHT) {
    measuredVal_motor[REAR_RIGHT].inpVoltage = msg.input_voltage;
    measuredVal_motor[REAR_RIGHT].avgMotorCurrent = msg.motor_current;
    measuredVal_motor[REAR_RIGHT].avgInputCurrent = msg.input_current;
    measuredVal_motor[REAR_RIGHT].erpm = msg.erpm;
    last_mot_state_update[MotorState::POS_REAR_RIGHT] = millis();
  }
}

void nuc_drive_callback(const NucDriveCommand& msg) {
  NUC_drive_coms.lin_vel     = msg.lin_vel;
  NUC_drive_coms.steer_f     = (float)msg.phi_f*180.0/PI;
  NUC_drive_coms.steer_r     = (float)msg.phi_r*180.0/PI;
  NUC_drive_coms.blink       = (uint8_t)msg.blink_com;
  NUC_drive_coms.last_update = millis();
}

void park_com_callback(const StartParking& msg)
{
  got_parking_command = msg.start_flag;
  
}

void parallel_park_callback(const ParallelParking& msg) {
  parking.lot_dist = msg.dist_to_lot;
  parking.lot_size = msg.lot_size;
}

void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  remote_control_Subscriber = new Subscriber<RemoteControl>(*node);
  motor_state_Subscriber = new Subscriber<MotorState>(*node);
  nuc_drive_Subscriber = new Subscriber<NucDriveCommand>(*node);
  start_parking_Subscriber = new Subscriber<StartParking>(*node);
  parallel_parking_Subscriber = new Subscriber<ParallelParking>(*node);

  if(remote_control_Subscriber->start(remote_control_callback) < 0)
  {
    Serial.println("Unable to start subscriber RC!");
  }
  if(motor_state_Subscriber->start(Motor_State_callback) < 0)
  {
    Serial.println("Unable to start subscriber motor_state!");
  }
  if(nuc_drive_Subscriber->start(nuc_drive_callback) < 0)
  {
    Serial.println("Unable to start subscriber nuc_drive!");
  }
  if(start_parking_Subscriber->start(park_com_callback) < 0)
  {
    Serial.println("Unable to start subscriber park_start!");
  }
  if(parallel_parking_Subscriber->start(parallel_park_callback) < 0)
  {
    Serial.println("Unable to start subscriber parallel_parking!");
  }
}
#endif
