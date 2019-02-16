#ifndef    PUBLISHER_H
#define    PUBLISHER_H

#include <uavcan/uavcan.hpp>
#include "phoenix_msgs/RemoteControl.hpp"
#include "phoenix_msgs/MotorState.hpp"
#include "phoenix_msgs/NodeState.hpp"
#include "phoenix_msgs/Battery.hpp"
#include "phoenix_msgs/ParallelParking.hpp"
#include "phoenix_msgs/UserButtons.hpp"
#include "phoenix_msgs/ConfigReceived.hpp"
#include "phoenix_can_shield.h"
#include "vuart.h"
#include <Filters.h>

using namespace uavcan;
using namespace phoenix_msgs;

// publishing tasks:
// we want to publish the rc readings from dji via a RemoteControl Messages
// we want to publish the battery cell voltages from via a Battery Messages
// we want to publish the state of the two motors via two MotorState Messages


// filters out changes faster that 5 Hz.
float filterFrequency = 5.0;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilterFront(LOWPASS, filterFrequency);
FilterOnePole lowpassFilterRear(LOWPASS, filterFrequency);

// publisher
Publisher<RemoteControl> *rc_Publisher;
Publisher<MotorState> *motor_state_Publisher;
Publisher<NodeState> *state_Publisher;
Publisher<Battery> *battery_Publisher;
Publisher<ParallelParking> *ppark_Publisher;
Publisher<UserButtons> *user_buttons_Publisher;
Publisher<ConfigReceived> *conf_rec_Publisher;

// additional configuration
static uint8_t motor3_position = MotorState::POS_REAR_LEFT;
static uint8_t motor4_position = MotorState::POS_REAR_RIGHT;

// initialize all publisher
void initPublisher(Node<NodeMemoryPoolSize> *node) {
    // create publishers
    rc_Publisher = new Publisher<RemoteControl>(*node);
    motor_state_Publisher = new Publisher<MotorState>(*node);
    state_Publisher = new Publisher<NodeState>(*node);
    battery_Publisher = new Publisher<Battery>(*node);
    ppark_Publisher = new Publisher<ParallelParking>(*node);
    user_buttons_Publisher = new Publisher<UserButtons>(*node);
    conf_rec_Publisher = new Publisher<ConfigReceived>(*node);

    // initiliaze publishers
    if (rc_Publisher->init() < 0) {
        Serial.println("Unable to initialize rc_Publisher!");
    }
    if (motor_state_Publisher->init() < 0) {
        Serial.println("Unable to initialize motor_state_Publisher!");
    }
    if (state_Publisher->init() < 0) {
        Serial.println("Unable to initialize state_Publisher!");
    }
    if (battery_Publisher->init() < 0) {
        Serial.println("Unable to initialize battery_Publisher!");
    }
    if (ppark_Publisher->init() < 0) {
        Serial.println("Unable to initialize ppark_Publisher!");
    }
    if (user_buttons_Publisher->init() < 0) {
        Serial.println("Unable to initialize user_buttons_Publisher!");
    }
    if (conf_rec_Publisher->init() < 0) {
        Serial.println("Unable to initialize conf_rec_Publisher!");
    }

    // set TX timeout
    rc_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
    motor_state_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
    state_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
    battery_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
    ppark_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
    user_buttons_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
    conf_rec_Publisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

// cycle all publisher
void cyclePublisher(DJI &dji) {
    // Cell Voltages:
    //      measure -> evaluate -> set battery_alarm for buzzer -> set RGB LED color -> publish to CAN
    if (systemClock->getMonotonic() > next_power_update){
        next_power_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(power_update_rate_us);

        // measure voltages of cells which returns value between [0:4069]
        analogReadRes(12);
        analogReadAveraging(4);
        float V1_raw = analogRead(CELL1_PIN);
        float V2_raw = analogRead(CELL2_PIN);
        float V3_raw = analogRead(CELL3_PIN);
        float V4_raw = analogRead(CELL4_PIN);

        // apply conversion to volt
        float V1 = V1_raw * (float) 0.0064453125;
        float V2 = V2_raw * (float) 0.0064453125;
        float V3 = V3_raw * (float) 0.0064453125;
        float V4 = V4_raw * (float) 0.0052815755;

        // calc cells
        float cell1 = V1;
        float cell2 = V2 - V1;
        float cell3 = V3 - V2;
        float cell4 = V4 - V3;

        // initialize message for CAN bus
        static Battery msg;
        msg.v1 = (uint16_t)(cell1 * 1000);
        msg.v2 = (uint16_t)(cell2 * 1000);
        msg.v3 = (uint16_t)(cell3 * 1000);
        msg.v4 = (uint16_t)(cell4 * 1000);

        // since V4 is measured over the CAN-cable directly on the Teensy-CAN-shield an error occurs if the
        // CAN-cable is not connected properly during testing. Following fix is applied:
        // If this is detected by V4 under voltage -> fix via cell4 = cell3
#ifdef IGNORE_CELL4
        if (V4_raw < 50) cell4 = cell3;
#endif
        // based on the calculated cell voltages in the generated message -> set the battery_alarm
        if (cell1 < V_ALM_FINAL || cell2 < V_ALM_FINAL || cell3 < V_ALM_FINAL || cell4 < V_ALM_FINAL) {
            battery_alarm = 1;
        }
        if (cell1 < BAT_V_THRESH && cell2 < BAT_V_THRESH && cell3 < BAT_V_THRESH && cell4 < BAT_V_THRESH)
            battery_alarm = 0;

        // set RGB led according to battery state
        // blue: no battery detected (all cells below BAT_V_THRESH which is around 0.5V)
        // green: all cells above 3.65V
        // orange: all cells above 3.3V
        // red: at least one cell below 3.3V
        if (cell1 < BAT_V_THRESH && cell2 < BAT_V_THRESH && cell3 < BAT_V_THRESH && cell4 < BAT_V_THRESH)
            setRGBled(0, 0, 255);
        else if (cell1 < 3.3 || cell2 < 3.3 || cell3 < 3.3 || cell4 < 3.3)
            setRGBled(255, 0, 0);
        else if (cell1 < 3.65 || cell2 < 3.65 || cell3 < 3.65 || cell4 < 3.65)
            setRGBled(200, 255, 0);
        else
            setRGBled(0, 255, 0);

        // publish data via CAN and toggle trafficLED on success
        if (battery_Publisher->broadcast(msg) < 0) {
            Serial.println("Error while broadcasting power state");
        } else {
            digitalWrite(trafficLedPin, HIGH);
        }
    }

    // Node State:
    if (systemClock->getMonotonic() > next_node_state_update) {
        next_node_state_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(node_state_update_rate_us);
        // measure voltages of cells which returns value between [0:4069]
        analogReadRes(12);
        analogReadAveraging(4);
        float V4_raw = analogRead(CELL4_PIN);
        float curr_raw = analogRead(CURR_PIN);

        float V4 = V4_raw * (float) 0.0052815755;
        float curr = curr_raw * CURR_FACTOR;

        static NodeState msg;
        msg.id = nodeID;
        msg.voltage = V4;
        msg.current = curr;
        msg.fault_code_1 = node_fault_code1;
        msg.fault_code_2 = node_fault_code2;
    }

    // Motor State:
    // before the motor state can be received it has to be requested. After a request it takes about 7ms for the reply
    if ((systemClock->getMonotonic() > next_motor_state_request_time) && (custom_vesc_config_set)) {
        // clean up input buffer since we are now only interested into the new message
        VescUartFlushAll(0);
        VescUartFlushAll(1);

        // we allow up to 10 messages in a row to be missed until alive state goes to zero -> dead
        if (vesc_motor_state_received[0] == 1) vesc_motor_state_alive[0] = 10;
        else if (vesc_motor_state_alive[0] > 0) vesc_motor_state_alive[0]--;
        else vesc_motor_state_alive[0] = 0;

        if (vesc_motor_state_received[1] == 1) vesc_motor_state_alive[1] = 10;
        else if (vesc_motor_state_alive[1] > 0) vesc_motor_state_alive[1]--;
        else vesc_motor_state_alive[1] = 0;

        // init received flags as not received for next requested update
        vesc_motor_state_received[0] = 0;
        vesc_motor_state_received[1] = 0;

        // send requests to vesc
        vesc_send_status_request(0);
        vesc_motor_state_requests[0]++;
        vesc_send_status_request(1);
        vesc_motor_state_requests[1]++;

        // set the time of next motor state receive
        next_motor_state_update_time = systemClock->getMonotonic() + MonotonicDuration::fromUSec(motor_state_reply_duration_us);

        // set the time of nex motor state request
        next_motor_state_request_time = systemClock->getMonotonic() + MonotonicDuration::fromUSec(motor_state_update_rate_us);

#ifdef VESC_DEBUG_OUTPUT
        Serial.print("vesc state 0 req: \t");
        Serial.print(vesc_motor_state_requests[0] - 1);
        Serial.print("\trec: \t");
        Serial.print(vesc_motor_state_receives[0]);

        Serial.print("\t vesc state 1 req: \t");
        Serial.print(vesc_motor_state_requests[1] - 1);
        Serial.print("\trec: \t");
        Serial.println(vesc_motor_state_receives[1]);

        Serial.println("Motor 3:");
        Serial.print("current min ");  Serial.println(measuredVal_motor3.min_current);
        Serial.print("current max ");  Serial.println(measuredVal_motor3.max_current);
        Serial.print("erpm min ");  Serial.println(measuredVal_motor3.min_erpm);
        Serial.print("erpm max ");  Serial.println(measuredVal_motor3.max_erpm);

        Serial.println("Motor 4:");
        Serial.print("current min ");  Serial.println(measuredVal_motor4.min_current);
        Serial.print("current max ");  Serial.println(measuredVal_motor4.max_current);
        Serial.print("erpm min ");  Serial.println(measuredVal_motor4.min_erpm);
        Serial.print("erpm max ");  Serial.println(measuredVal_motor4.max_erpm);
#endif
    }

    // since we requested motor_state_reply_duration_ms ago there should now be a new VESC motor state message available
    if (systemClock->getMonotonic() > next_motor_state_update_time) {
        static MotorState msg;

        // update motor 3 information
        if ((vesc_motor_state_received[0] == 0) && VescUartGetValue(measuredVal_motor3, 0)) {
            vesc_motor_state_receives[0]++;
            vesc_motor_state_received[0] = 1;

            msg.position = motor3_position;
            msg.motor_current = measuredVal_motor3.avgMotorCurrent;
            msg.input_current = measuredVal_motor3.avgInputCurrent;
            msg.erpm = measuredVal_motor3.erpm;

            // publish data via CAN and toggle trafficLED on success
            if (motor_state_Publisher->broadcast(msg) < 0) {
                Serial.println("Error while broadcasting motor 3 state");
            } else {
                digitalWrite(trafficLedPin, HIGH);
            }
#ifdef VESC_DEBUG_OUTPUT
            MonotonicDuration diff = systemClock->getMonotonic() - next_motor_state_update_time;
            Serial.print("vesc 0 ping: \t");
            Serial.println((long)diff.toUSec());
#endif
        }

        // update motor 4 information
        if ((vesc_motor_state_received[1] == 0) && VescUartGetValue(measuredVal_motor4, 1)) {
            vesc_motor_state_receives[1]++;
            vesc_motor_state_received[1] = 1;

            msg.position = motor4_position;
            msg.motor_current = measuredVal_motor4.avgMotorCurrent;
            msg.input_current = measuredVal_motor4.avgInputCurrent;
            msg.erpm = measuredVal_motor4.erpm;

            // publish data via CAN and toggle trafficLED on success
            if (motor_state_Publisher->broadcast(msg) < 0) {
                Serial.println("Error while broadcasting motor 4 state");
            } else {
                digitalWrite(trafficLedPin, HIGH);
            }
#ifdef VESC_DEBUG_OUTPUT
            MonotonicDuration diff = systemClock->getMonotonic() - next_motor_state_update_time;
            Serial.print("vesc 1 ping: \t");
            Serial.println((long)diff.toUSec());
#endif
        }
    }

    // RC update
    // remote control update -> at rc_update_rate -> check time first
    if (dji.read()) {
#ifdef DJI_DEBUG_OUTPUT
        MonotonicDuration diff = systemClock->getMonotonic() - last_rc_receive;
        Serial.print("RC diff [us]:");
        Serial.print((long)diff.toUSec());
#endif
        last_rc_receive = systemClock->getMonotonic();
    }

    if (systemClock->getMonotonic() > next_rc_update) {
        next_rc_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(rc_update_rate_us);
        static RemoteControl msg;

        if ((systemClock->getMonotonic() - last_rc_receive).toUSec() > rc_timeout_us) {
            // last rc reading is old -> use fail save values:
            msg.drive_mode = RemoteControl::DRIVE_MODE_RC_DISCONNECTED;
            msg.aux_mode = RemoteControl::AUX_MODE_RC_DISCONNECTED;
            msg.velocity = 0;
            msg.steer_rear = 0;
            msg.steer_front = 0;
        } else {
            // left switch - drive mode select
            switch (dji.leftSwitch()) {
                case DJI::DOWN:
                    msg.drive_mode = RemoteControl::DRIVE_MODE_MANUAL;
                    break;
                case DJI::MIDDLE:
                    msg.drive_mode = RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS;
                    break;
                case DJI::UP:
                    msg.drive_mode = RemoteControl::DRIVE_MODE_AUTONOMOUS;
                    break;
                default:
                    msg.drive_mode = RemoteControl::DRIVE_MODE_RC_DISCONNECTED;
                    break;
            }

            // right switch - aux mode select
            switch (dji.rightSwitch()) {
                case DJI::DOWN:
                    msg.aux_mode = RemoteControl::AUX_MODE_DOWN;
                    break;
                case DJI::MIDDLE:
                    msg.aux_mode = RemoteControl::AUX_MODE_CENTER;
                    break;
                case DJI::UP:
                    msg.aux_mode = RemoteControl::AUX_MODE_UP;
                    break;
                default:
                    msg.aux_mode = RemoteControl::AUX_MODE_RC_DISCONNECTED;
                    break;
            }

            // left stick - up/down - target velocity
            msg.velocity = dji.leftVerticalStick(msg.velocity);

            // left stick - left/right - steering rear
            msg.steer_rear = dji.leftHorizontalStick(msg.steer_rear);

            // right stick - left/right - front steer
            msg.steer_front = dji.rightHorizontalStick(msg.steer_front);
        }

        if (rc_Publisher->broadcast(msg) < 0) {
            Serial.println("Error while broadcasting rc message");
        } else {
            digitalWrite(trafficLedPin, HIGH);
        }
    }

    // buttons
    if (systemClock->getMonotonic() > next_button_update) {
        next_button_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(button_update_rate_us);

        analogReadRes(12);
        analogReadAveraging(10);

        #define R0 16
        int raw = analogRead(BUTTON_PIN);
        float r = (float)raw / 4096;
        r = 1 - r;
        r = R0 / r;
        r = r - R0;
        r = round((double)r);
        bitwise_buttons = (uint8_t)((uint8_t) r ^ 0x1F);    // invert bits for bitwise results

#ifdef BUTTONS_DEBUG_OUTPUT
        Serial.print("Buttons: ");
        Serial.print(raw);
        Serial.print("\t");
        Serial.print(r);
        Serial.print("\t");
        Serial.println(bitwise_buttons, BIN);
#endif

        static UserButtons msg;
        msg.bit_but = bitwise_buttons;

        if (user_buttons_Publisher->broadcast(msg) < 0) {
            Serial.println("Error while broadcasting button message");
        } else {
            digitalWrite(trafficLedPin, HIGH);
        }
    }

    // parking lot
    // to ensure that this message really really arrives on the computer, it is sent until publish_lot_msg_to_send is zero
    if ((publish_lot_msg_to_send > 0) && (systemClock->getMonotonic() > next_par_lot_update)) {
        next_par_lot_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(par_lot_update_rate_us);

        static ParallelParking msg;
        msg.lot_size = lot_size;
        msg.dist_to_lot = x_veh() - last_lot_pos;

        if (ppark_Publisher->broadcast(msg) < 0) {
            Serial.println("Error while broadcasting parking message");
        } else {
            digitalWrite(trafficLedPin, HIGH);
        }
        publish_lot_msg_to_send--;
    }
}

void Publisher_config_received(uint8_t type) {
    ConfigReceived msg;
    msg.received_config = type;

    if (conf_rec_Publisher->broadcast(msg) < 0) {
            Serial.println("Error while broadcasting config receive message");
        } else {
            digitalWrite(trafficLedPin, HIGH);
        }
}

void pf_ir_routine() {
    static float start_odom;
    float pos = x_veh();
    uint8_t state = digitalRead(PF_LS_PIN);
    sei();
    if (state == LOW) {
        start_odom = pos;
    } else {
        lot_size = pos - start_odom;
        if (lot_size < 0.65 && lot_size > 1.05) {
            last_lot_pos = pos;
            publish_lot_msg_to_send = 5;
        }
    }
}

#endif