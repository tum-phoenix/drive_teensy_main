#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "parameter.hpp"
#include "vuart.h"
#include <math.h>

#include "PWMServo.h"

#include "DJI.h"

// CAN Node settings
    static constexpr uint32_t nodeID = 102;
    static constexpr uint8_t swVersion = 1;
    static constexpr uint8_t hwVersion = 1;
    static const char *nodeName = "org.phoenix.dji";
//

// application settings
    static constexpr float framerate = 500;
//

// node state
    const int node_state_update_rate_us = 1000000 / 20;     // in us -> 20Hz
    MonotonicTime next_node_state_update = MonotonicTime::fromMSec(0);
    uint8_t node_fault_code1 = 0;
    uint8_t node_fault_code2 = 0;
//

// DJI
    #define DJI_DEBUG_OUTPUT
    const int rc_update_rate_us = 1000000 / 50;             // in us -> 20Hz
    const int rc_timeout_us = 1000 * 100;                   // timeout if last rc command is 100ms old
    MonotonicTime next_rc_update = MonotonicTime::fromMSec(0);
    DJI dji(Serial2);
//

// Power
    #define IGNORE_CELL4          // use this to run teensy without connected CAN-cable (it is used for V4 measurement)
    const int power_update_rate_us = 1000000 / 2;         // in us -> 2Hz
    MonotonicTime next_power_update = MonotonicTime::fromMSec(0);
    #define CELL3_PIN A5
    #define CELL2_PIN A11
    #define CELL1_PIN A10
    #define CELL4_PIN A0
    #define CURR_PIN  A1
    #define CELL_R1 14000.0
    #define CELL_R2 2000.0
    #define BAT_V_THRESH 0.5
    #define A_REF 3.3
    #define CURR_FACTOR (float)(0.00161133 / 4) // 0R01 + 200V/V
    #define V_ALM_FINAL 3.0
//

// buzzer
    void buzzer_routine();
    const int buzzer_beep_rate_us = 1000000 / 2;              // in us -> 2Hz
    MonotonicTime next_buzzer_update = MonotonicTime::fromUSec(0);
    #define BUZZER_PIN 11
    uint8_t battery_alarm = 0;
//

// Buttons
    //#define BUTTONS_DEBUG_OUTPUT
    const int button_update_rate_us = 1000000 / 10;           // in us -> 10Hz
    MonotonicTime next_button_update = MonotonicTime::fromUSec(0);
    #define BUTTON_PIN A4
    uint8_t bitwise_buttons = 0;
//

// Vesc
    void read_vesc_data(bldcMeasure &measuredVal_motor);
    void send_vesc_data(bldcMeasure &measuredVal_motor);

    //#define VESC_DEBUG_OUTPUT
    #define MOT_POL_NUM 14
    #define MOTOR_Y_WIND_FACTOR 1.7

    // timing for vesc status request
    const uint16_t motor_state_update_rate_us = 1000000 / 100;  // in us  -> 100Hz
    const uint32_t motor_config_update_rate_us = 1000000 / 1;   // in us  -> 1Hz
    const uint16_t vesc_com_start_delay_ms = 5000;

    // motor status is stored in bldcMeasure containers
    struct bldcMeasure measuredVal_motor3;      // rear left
    struct bldcMeasure measuredVal_motor4;      // rear right

    // custom vesc config is stored in one mcconf_t container for both ESC
    mcconf_t mcconf;    // min/max current min/max erpm
//

// servos for steering
    PWMServo steering_servo_3;
    PWMServo steering_servo_4;
    float steering_servo_offset_3 = 90;
    float steering_servo_offset_4 = 90;
    uint8_t steering_servo_3_pin = 5;
    uint8_t steering_servo_4_pin = 20;
//

// motor target misc
    MonotonicTime last_motor_target_receive = MonotonicTime::fromMSec(0);
    #define MOTOR_COM_TIMEOUT_US 1000 * 100     // max accepted time between motor commands in us
//

// Pepperl+Fuchs / parallel parking
    const int par_lot_update_rate_us = 1000000 / 50;     // in us  -> 50Hz
    MonotonicTime next_par_lot_update = MonotonicTime::fromMSec(0);
    #define PF_LS_PIN 13
    float lot_size = 0;
    float last_lot_pos = 0;
    uint8_t publish_lot_msg_to_send = 0;
//

// odometry
    typedef struct {
        float dist_trav;  // m
        float speed;      // m/s
    } odometry_t;
    odometry_t rear;
    #define WHEEL_RADIUS_M 0.033
    float v_veh();
    float x_veh();
//

#include "Publisher.h"
#include "Subscriber.h"

void setup() {
    // init Buzzer Pin and turn Buzzer on during setup
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, 1);

    // setup UART port for vesc
    Serial1.begin(230400);
    Serial3.begin(230400);
    SetSerialPort(&Serial1, &Serial3);

    // setup VESC
    measuredVal_motor3.motor_position = MotorState::POS_REAR_LEFT;
    measuredVal_motor3.serial_port = 0;
    measuredVal_motor4.motor_position = MotorState::POS_REAR_RIGHT;
    measuredVal_motor4.serial_port = 1;

    // setup UART for usb port as debug output
    Serial.begin(115200);

    // setup power (voltage measurements)
    analogReadRes(12);
    analogReadAveraging(4);

    // Pepperl+Fuchs
    //RISING/HIGH/CHANGE/LOW/FALLING
    attachInterrupt(PF_LS_PIN, pf_ir_routine, CHANGE);

    // init LEDs
    initLeds();

    // create CAN node
    systemClock = &initSystemClock();
    canDriver = &initCanDriver();
    node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
    initNode(node, nodeID, nodeName, swVersion, hwVersion);

    // init publisher
    initPublisher(node);

    // init subscriber
    initSubscriber(node);

    // set up filters
    configureCanAcceptanceFilters(*node);

    // init parameter
    initParameter(node);

    // start up node
    node->setModeOperational();

    // setup DJI remote
    dji.begin(&Serial2);

    // turn buzzer off after setup
    digitalWrite(BUZZER_PIN, 0);

    // setup servos for steering
    steering_servo_offset_3 = configuration.steeringOff_RL + 90;
    steering_servo_offset_4 = configuration.steeringOff_RR + 90;
}


void loop() {
    // wait in cycle
    cycleWait(framerate);

    // VESC input and output
    // send VESC status requests
    // on VESC status receive publish the received data
    // send and validate VESC custom config
    read_vesc_data(measuredVal_motor3);
    read_vesc_data(measuredVal_motor4);
    send_vesc_data(measuredVal_motor3);
    send_vesc_data(measuredVal_motor4);

    // RC update
    // remote control update reads data, but publishing data at fixed rate is done separately
    dji.read();

    // do some CAN stuff
    cycleNode(node);

    // cycle publisher
    Publisher_cell_voltages();
    Publisher_node_state();
    Publisher_rc();
    Publisher_buttons();
    Publisher_parking();

    // toggle heartbeat
    toggleHeartBeat();

    // update buzzer
    buzzer_routine();

    // in case no recent motor target current is received via CAN: turn them off
    if ((systemClock->getMonotonic() - last_motor_target_receive).toUSec() > MOTOR_COM_TIMEOUT_US) {
        MotorTarget msg;
        msg.motor_arm = MotorTarget::MOTORS_OFF;
        msg.servo_attach = MotorTarget::SERVOS_OFF;
        motor_target_callback(msg);
    }

}


void read_vesc_data(bldcMeasure &measuredVal_motor){
    // read all new VESC messages
    for (uint8_t i = 0; i < 60; i++){
#ifdef VESC_DEBUG_OUTPUT
        if (i > 50) {
            Serial.println("read_vesc_data needs more than 25 loops WARNING");
        }
#endif
        // read messages for motor 3 on com port 0 into the status container measuredVal_motor3
        uint8_t msg_type = VescUartGetValue(measuredVal_motor, measuredVal_motor.serial_port);
        if (msg_type == 0) {
            // no new message available -> exit while (true) loop
            break;
        } else if ((msg_type == COMM_GET_VALUES_SHORT) || (msg_type == COMM_GET_VALUES)) {
            // new VESC status values available
            Publisher_motor_state(measuredVal_motor);

            measuredVal_motor.state_answered++;
            measuredVal_motor.state_answers++;

            uint16_t ping = (uint16_t)(systemClock->getMonotonic() - measuredVal_motor.last_status_req_time).toUSec();
            measuredVal_motor.ping_us += ping;
            measuredVal_motor.ping_us /= 2;

#ifdef VESC_DEBUG_OUTPUT
            Serial.print("vesc ");
            Serial.print(measuredVal_motor.motor_position);
            Serial.print(" ping: \t");
            Serial.print(ping);
            Serial.print(" \t ");
            Serial.println(measuredVal_motor.ping_us);
#endif
        } else if (msg_type == COMM_GET_CUSTOM_MC_CONF_VALUES) {
            // received motor config -> validate with set mcconf -> if diff send motor config again else all fine.
            uint8_t config_diff = 0;
            if (measuredVal_motor.min_erpm != mcconf.min_erpm) config_diff++;
            if (measuredVal_motor.max_erpm != mcconf.max_erpm) config_diff++;
            if (measuredVal_motor.min_current != mcconf.min_current) config_diff++;
            if (measuredVal_motor.max_current != mcconf.max_current) config_diff++;
            if (config_diff > 0) {
                measuredVal_motor.custom_config_set_status = 0;     // 0: not set
                Serial.print("vesc ");
                Serial.print(measuredVal_motor.motor_position);
                Serial.print(" manual config deviation found:");
                Serial.println(config_diff);
            } else {
                measuredVal_motor.custom_config_set_status = 3;     // 3: settings okay
                Publisher_config_received(ConfigReceived::VESC_MOTOR_CONFIG);
                Serial.println("vesc manual config set successfully");
            }
#ifdef VESC_DEBUG_OUTPUT
            Serial.print("vesc ");
            Serial.print(measuredVal_motor.motor_position);
            Serial.print(" settings: ");
            Serial.print("\t current min: ");  Serial.print(measuredVal_motor.min_current);
            Serial.print("\t current max: ");  Serial.print(measuredVal_motor.max_current);
            Serial.print("\t erpm min: ");  Serial.print(measuredVal_motor.min_erpm);
            Serial.print("\t erpm max: ");  Serial.println(measuredVal_motor.max_erpm);
#endif
        }
    }
}

void send_vesc_data(bldcMeasure &measuredVal_motor){
    // give VESC time to start
    if (systemClock->getMonotonic().toMSec() < vesc_com_start_delay_ms) {
        // systemClock->getMonotonic().toMSec() is a uint64 -> no overflow before end of century.
        return;
    }

    // if we have to set the motor config, everything is messed up
    if (measuredVal_motor.custom_config_set_status == 0) {          // 0: not set
        // send custom motor configuration to VESC
        Vesc_send_custom_config(
                mcconf.min_current,
                mcconf.max_current,
                mcconf.min_erpm,
                mcconf.max_erpm,
                measuredVal_motor.serial_port);
        measuredVal_motor.last_config_set_time = systemClock->getMonotonic();
        measuredVal_motor.custom_config_set_status = 1;
    } else if (measuredVal_motor.custom_config_set_status == 1) {   // 1: send but not checked
        // give VESC time to read config and set config (this takes at least 250ms)
        if ((systemClock->getMonotonic() - measuredVal_motor.last_config_set_time).toMSec() > 250) {
            vesc_send_custom_config_request(measuredVal_motor.serial_port);
            measuredVal_motor.last_config_set_time = systemClock->getMonotonic();
            measuredVal_motor.custom_config_set_status = 2;
        }
    } else if (measuredVal_motor.custom_config_set_status == 2) {   // 2: waiting for confirmed settings
        // give VESC time to reconfirm the settings (this takes not more than 10 ms)
        if ((systemClock->getMonotonic() - measuredVal_motor.last_config_set_time).toMSec() > 10) {
            // if still in 'waiting for confirmed settings' state we have to repeat sending the config process
            measuredVal_motor.custom_config_set_status = 0;
        }
    } else {
        // send vesc requests out at fixed frequency
        if (systemClock->getMonotonic() > measuredVal_motor.next_status_req_time) {
            // evaluate alive status of VESC by checking whether the previous request was answered
            if (measuredVal_motor.state_answered != 0) measuredVal_motor.alive = 10;
            else if (measuredVal_motor.alive > 0) measuredVal_motor.alive--;
            else measuredVal_motor.alive = 0;

            // send motor state request
            vesc_send_status_request(measuredVal_motor.serial_port);
            measuredVal_motor.state_requests++;
            measuredVal_motor.state_answered = 0;

            // set the time of nex motor state request
            measuredVal_motor.next_status_req_time = systemClock->getMonotonic() + MonotonicDuration::fromUSec(motor_state_update_rate_us);
            measuredVal_motor.last_status_req_time = systemClock->getMonotonic();

#ifdef VESC_DEBUG_OUTPUT
            Serial.print("vesc ");
            Serial.print(measuredVal_motor.motor_position);
            Serial.print(" com stats: ");
            Serial.print(measuredVal_motor.state_requests - 1);
            Serial.print("\trec: \t");
            Serial.print(measuredVal_motor.state_answeres);
            Serial.print("\tping: \t");
            Serial.print(measuredVal_motor.ping);
#endif
        }

        if (systemClock->getMonotonic() > measuredVal_motor.next_config_req_time) {
            // send motor config request
            vesc_send_custom_config_request(measuredVal_motor.serial_port);

            // set the time of nex motor state request
            measuredVal_motor.next_config_req_time = systemClock->getMonotonic() + MonotonicDuration::fromUSec(motor_config_update_rate_us);
            measuredVal_motor.last_config_req_time = systemClock->getMonotonic();
        }

    }



}

float v_veh() {
    // erpm / 7 = RPM                 rounds per minute
    // erpm / 7 / 60 = RPS            rounds per second
    // erpm / 7 / 60 * (2 pi r) = m/s
    float mean_speed = 0;
    float num = 0;
    if (measuredVal_motor3.alive > 0) {
        mean_speed += (float) measuredVal_motor3.erpm / 7. / 60. * 2. * M_PI * WHEEL_RADIUS_M;
        num++;
    }
    if (measuredVal_motor4.alive > 0) {
        mean_speed += (float) measuredVal_motor4.erpm / 7. / 60. * 2. * M_PI * WHEEL_RADIUS_M;
        num++;
    }

    if (num == 0) {
        rear.speed = 0;
    } else {
        rear.speed = mean_speed / num;    // m/s;
    }
    return rear.speed;
}

float x_veh() {
    v_veh();
    static MonotonicTime last_x_veh_update = systemClock->getMonotonic();
    static MonotonicTime this_x_veh_update = systemClock->getMonotonic();
    last_x_veh_update = this_x_veh_update;
    this_x_veh_update = systemClock->getMonotonic();
    float dt = (float) (this_x_veh_update - last_x_veh_update).toUSec();     // micros
    float ds = (rear.speed * dt) / (float) 1000000.;                         // m
    rear.dist_trav += ds;
    return rear.dist_trav;
}

void buzzer_routine() {
    if ((battery_alarm) && (systemClock->getMonotonic() > next_buzzer_update)) {
        next_buzzer_update = systemClock->getMonotonic() + MonotonicDuration::fromUSec(buzzer_beep_rate_us);
        static uint8_t buzzer_mode = 0;
        buzzer_mode ^= 1;
        digitalWrite(BUZZER_PIN, buzzer_mode);
    } else if (battery_alarm == 0) {
        digitalWrite(BUZZER_PIN, 0);
    }
}
