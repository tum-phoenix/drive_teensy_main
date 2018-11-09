#include "Arduino.h"
#include "phoenix_can_shield.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <VescUart.h>
#include "PWMServo.h"
#include "parameter.hpp"
#include <math.h>

// CAN Node settings
static constexpr uint32_t nodeID = 101;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.body_control";

// application settings
static constexpr float framerate = 100;

// Driving dynamics
#define FRONT_LEFT  MotorState::POS_FRONT_LEFT 
#define FRONT_RIGHT MotorState::POS_FRONT_RIGHT
#define REAR_LEFT   MotorState::POS_REAR_LEFT
#define REAR_RIGHT  MotorState::POS_REAR_RIGHT

typedef struct {
  float thr = 0;
  float steer_f = 0;
  float steer_r = 0;
  float drive_state = 0;
} drive_comm_t;
drive_comm_t RC_coms;

typedef struct {
  float lin_vel = 0;        // m/s
  float ang_vel = 0;    // deg/s
} NUC_drive_coms_t;
NUC_drive_coms_t NUC_drive_coms;

// Vesc
static struct actor_comm_t{
  float motor_amps[4] = {0,0,0,0};
  float servo_angles[4] = {90,90,90,90};
} actor_comms;

bldcMeasure measuredVal_motor[4];

// Power
int power_update_rate = 5;
MonotonicTime last_power_update = MonotonicTime::fromMSec(0);
#define CELL4_PIN A0
#define CURR_PIN  A1
#define Cell4_FACTOR 0.0211263/4 // 10k + 1k8 
#define CURR_FACTOR 0.00161133/4 // 0R01 + 200V/V

// BNO055 imu
Adafruit_BNO055 bno055 = Adafruit_BNO055();

// servos for steering
PWMServo steering_servo[2];
float steering_servo_position[2];
float steering_servo_offset[2] = {98,92};
uint8_t steering_servo_pin[2] = {9,10};

float calc_speed_pid();
void dynamics_control();
void vesc_command();
uint8_t check_arm_state();

#include "Publisher.h"
#include "Subscriber.h"

void setup() {
  //get EEPROM Parameters
  readParamsFromEEPROM();

  Serial.begin(115200);

  // setup UART port for vesc
  Serial1.begin(115200);
  Serial3.begin(115200);
  SetSerialPort(&Serial1, &Serial3, &Serial1, &Serial1);
  //SetDebugSerialPort(&Serial);
  
  // setup power
  analogReadRes(12);
  analogReadAveraging(4);

  // init LEDs
  initLeds();

  // create a node
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

  // set up BNO055 IMU Adafruit_Sensor
  bno055.begin();

}

imu_t bno_data;

uint32_t t_ = 0;
void loop() {
  // wait in cycle
  uint32_t t = micros();
  float cpu_load = (float)(t-t_)/(1000000./(float)framerate);
  setRGBled((uint8_t)(cpu_load*2.55),255-(uint8_t)(cpu_load*2.55),0);
  //Serial.print("CPU Load: "); Serial.print(cpu_load); Serial.prntln(" \%");
  cycleWait(framerate);
  t_ = micros();

  // get RC data, high level commands, motor telemetry rear motors
  cycleNode(node);

  // update motor front left information
  if (!VescUartGetValue(measuredVal_motor[FRONT_LEFT], FRONT_LEFT)){
    Serial.println("failed to get motor data front left!");
  }
  else {
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_LEFT], FRONT_LEFT+1);
  }
  // update motor front right information
  if (!VescUartGetValue(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT)) {
    Serial.println("failed to get motor data front right!");
  }
  else {
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT+1);
  }

  // BNO055 data aquisition
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  bno_data.lin_acc = bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno_data.gyro = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_data.euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  // main driving dynamics calculations
  dynamics_control();

  // cycle publisher
  cyclePublisherBNO(bno_data);
  cyclePublisher_Actor_Comms(actor_comms);

  // set the connected Motors 
  vesc_command();
  steering_servo[FRONT_LEFT].write(steering_servo_offset[FRONT_LEFT]+actor_comms.servo_angles[FRONT_LEFT]);
  steering_servo[FRONT_RIGHT].write(steering_servo_offset[FRONT_RIGHT]+actor_comms.servo_angles[FRONT_RIGHT]);
    
  // spam Power info
  cyclePublisher_Power();
  
  // toggle heartbeat
  toggleHeartBeat();
}

#define WHEEL_RADIUS_M 0.033
                                            // acceleration directions in the car
float a_x_imu() {                           //       _____.
  return (float)bno_data.lin_acc[0];        //      /_    .
}                                           //      |_|   ^ X
                                            //     |      |
float a_y_imu() {                           //     |      |     Y
  return (float)bno_data.lin_acc[0];        //     |    --+----->
}                                           //     | _    |  
                                            //      |_|   . 
                                            //     |______.

float v_wheel(uint8_t wheelindex) {
  float mean_rounds = (float)measuredVal_motor[wheelindex].rpm / 7; // RPM
  mean_rounds /= 60; // RPS
  mean_rounds = mean_rounds * 2 * M_PI * WHEEL_RADIUS_M; // m/s;
  return mean_rounds;
}

float v_veh() {
  //if (a_x_imu()>=0) return (v_wheel(REAR_LEFT)+v_wheel(REAR_RIGHT)) / 2;
  //else              return (v_wheel(FRONT_LEFT)+v_wheel(FRONT_RIGHT)) / 2;
  return (v_wheel(FRONT_LEFT)+v_wheel(FRONT_RIGHT)+v_wheel(REAR_LEFT)+v_wheel(REAR_RIGHT)) / 4;
}

float a_wheel(uint8_t wheelindex) {
  static float v = 0;
  static float prevt, t, dt, vprev;
  prevt = t;
  t = (float)micros()/1000000.;
  dt = t-prevt;
  vprev = v;
  v = v_wheel(wheelindex);
  return (v-vprev)/dt;
}

void dynamics_control() {
  #define TICKS_P_DEG 6 // servo steps per actual degree
  #define MAX_STEER_ANGLE_INNER 50
  #define MAX_STEER_ANGLE_OUTER 30

  if (check_arm_state()) {
    float main_amps = calc_speed_pid();
    actor_comms.motor_amps[FRONT_LEFT]  = main_amps;
    actor_comms.motor_amps[FRONT_RIGHT] = main_amps;
    actor_comms.motor_amps[REAR_LEFT]   = main_amps;
    actor_comms.motor_amps[REAR_RIGHT]  = main_amps;

    if (RC_coms.steer_f > 0) {
      actor_comms.servo_angles[FRONT_LEFT] = -RC_coms.steer_f*MAX_STEER_ANGLE_OUTER;
      actor_comms.servo_angles[FRONT_RIGHT] = -RC_coms.steer_f*MAX_STEER_ANGLE_INNER;
    } else {
      actor_comms.servo_angles[FRONT_LEFT] = -RC_coms.steer_f*MAX_STEER_ANGLE_INNER;
      actor_comms.servo_angles[FRONT_RIGHT] = -RC_coms.steer_f*MAX_STEER_ANGLE_OUTER; 
    }
    if (RC_coms.steer_r > 0) {
      actor_comms.servo_angles[REAR_LEFT] = RC_coms.steer_r*MAX_STEER_ANGLE_OUTER;
      actor_comms.servo_angles[REAR_RIGHT] = RC_coms.steer_r*MAX_STEER_ANGLE_INNER;
    } else {
      actor_comms.servo_angles[REAR_LEFT] = RC_coms.steer_r*MAX_STEER_ANGLE_INNER;
      actor_comms.servo_angles[REAR_RIGHT] = RC_coms.steer_r*MAX_STEER_ANGLE_OUTER;
    }
    //Serial.print(actor_comms.servo_angles[0]);
    //Serial.print("\t");
    //Serial.print(actor_comms.servo_angles[1]);
    //Serial.print("\t");
    //Serial.print(actor_comms.servo_angles[2]);
    //Serial.print("\t");
    //Serial.println(actor_comms.servo_angles[3]);
  } else {
    actor_comms.motor_amps[FRONT_LEFT] = 0;
    actor_comms.motor_amps[FRONT_RIGHT] = 0;
    actor_comms.motor_amps[REAR_LEFT] = 0;
    actor_comms.motor_amps[REAR_RIGHT] = 0;
    
    actor_comms.servo_angles[FRONT_LEFT] = 0;
    actor_comms.servo_angles[FRONT_RIGHT] = 0;
    actor_comms.servo_angles[REAR_LEFT] = 0;
    actor_comms.servo_angles[REAR_RIGHT] = 0;
  }
}

void vesc_command() {
  double sign_v = sgn(v_veh());
  //if (sign_v == sgn(actor_comms.motor_amps[FRONT_LEFT]))  VescUartSetCurrent(actor_comms.motor_amps[FRONT_LEFT],FRONT_LEFT);
  //else                                                    VescUartSetCurrentBrake(actor_comms.motor_amps[FRONT_LEFT],FRONT_LEFT);
  //if (sign_v == sgn(actor_comms.motor_amps[FRONT_RIGHT])) VescUartSetCurrent(actor_comms.motor_amps[FRONT_RIGHT],FRONT_RIGHT);
  //else                                                    VescUartSetCurrentBrake(actor_comms.motor_amps[FRONT_RIGHT],FRONT_RIGHT);
  VescUartSetCurrent(actor_comms.motor_amps[FRONT_LEFT],FRONT_LEFT);
  VescUartSetCurrent(actor_comms.motor_amps[FRONT_RIGHT],FRONT_RIGHT);
}

void traction_control() {
  if (true);
}

uint8_t check_arm_state() {
  if (  RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL 
     || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS
     || RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS) {
    if (!steering_servo[FRONT_LEFT].attached() || !steering_servo[FRONT_RIGHT].attached()) {      
      // setup servos for steering
      steering_servo[FRONT_LEFT].attach(steering_servo_pin[FRONT_LEFT]);
      steering_servo[FRONT_RIGHT].attach(steering_servo_pin[FRONT_RIGHT]);
    }
    return RC_coms.drive_state;
  } else {
    //steering_servo[FRONT_LEFT].detach();
    //steering_servo[FRONT_RIGHT].detach();
    return false;
  }
}

float calc_speed_pid() {
  static float main_amps, v_val;
  static float P_, i_, i_integral, I_, d_, D_, speed_PID, v_error, v_err_prev, derr;
  static float prevt, t, dt;
  prevt = t;
  t = (float)micros()/1000000.;
  dt = t-prevt;

  v_val = v_veh();

  //calculate Velocity error
  static float v_sp = 0;
  if (  RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL 
     || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS) {
    v_sp = RC_coms.thr*configuration.maxSpeed;
  } else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS){
    v_sp = NUC_drive_coms.lin_vel;
  } else {
    v_sp = 0;
  }
  v_error = v_sp - v_val;
  v_err_prev = v_error;
  derr = v_error - v_err_prev;

  // calculate P term
  P_ = configuration.speedKp * v_error;
  // calculate I term
  i_ = dt * (v_err_prev + v_error) / 2;
  i_integral += i_;
  I_ = configuration.speedKi * i_integral;

  // calculate D term
  d_ = derr / dt;
  D_ = configuration.speedKd * d_;

  speed_PID = P_ + I_ + D_;
  main_amps = speed_PID;
  if      (main_amps > configuration.maxMotorAmps) main_amps = configuration.maxMotorAmps;
  else if (main_amps < -configuration.maxMotorAmps) main_amps = -configuration.maxMotorAmps;
  //Serial.print(v_veh());
  //Serial.print("\t");
  //Serial.print(P_);
  //Serial.print("\t");
  //Serial.print(D_);
  //Serial.print("\t");
  //Serial.println(main_amps);
  return main_amps;
}