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

bool traction_lost[4] = {0,0,0,0};
typedef struct {
  float r;
  float rl;
  float delta[4];
  float amps[4];
  float l;
  float lv;
  float lh;
  float v;
  float beta;
  float phi_p;
  float ay;
  float ax;

  float rturnw[4];            // radius of each wheel to cover
} body_state_t;
body_state_t veh_state_sp;    // theoretical values
body_state_t veh_state_ac;    // actual values

typedef struct {
  float l0   = 0.222;  // m    physical wheelbase
  float s    = 0.177;  // m    track width
  float m    = 2500;   // kg
  float lv = 0.0885;   // -     position of center of gravity in x dir
  float calpha = 1e6;  // N/rad     no wheel model availabe -> very high wheel stiffness     
  float rwheel = 0.033; // m
} body_att_t;
static body_att_t veh_att;

typedef struct {
  float thr = 0;
  float steer_f = 0;
  float steer_r = 0;
  float drive_state = 0;
} drive_comm_t;
drive_comm_t RC_coms;

typedef struct {
  float lin_vel = 0;        // m/s
  float steer = 0;    // deg/s
  bool steer_type = 0;
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
#define Cell4_FACTOR 0.0052815755 // 10k + 1k8 
#define CURR_FACTOR 0.00161133/4 // 0R01 + 200V/V

// BNO055 imu
Adafruit_BNO055 bno055 = Adafruit_BNO055();

// servos for steering
PWMServo steering_servo[2];
float steering_servo_position[2];
float steering_servo_offset[2] = {98,92};
uint8_t steering_servo_pin[2] = {9,10};
#define MAX_STEER_ANGLE 32              // degrees
#define MAX_STEER_SERVO_INNER 50        // servo degrees
#define MAX_STEER_SERVO_OUTER 30        // servo degrees
#define AC_TO_SERV_INN (MAX_STEER_SERVO_INNER/MAX_STEER_ANGLE)
#define AC_TO_SERV_OUT (MAX_STEER_SERVO_OUTER/MAX_STEER_ANGLE)

void calc_speed_pid();
void vesc_command();
uint8_t check_arm_state();
void calc_steer_basic();
void calc_servo_angles(float *);
void steer_angle_distribution(float*);
void calc_body_state_sp();
void traction_control();
void torque_vectoring();
void calc_traction();
void dynamics_control();
float v_norm(uint8_t);

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
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_LEFT], FRONT_LEFT);
  }
  // update motor front right information
  if (!VescUartGetValue(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT)) {
    Serial.println("failed to get motor data front right!");
  }
  else {
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT);
  }

  // BNO055 data aquisition
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //bno_data.lin_acc = bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //bno_data.gyro = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //bno_data.euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  // main driving dynamics calculations
  calc_body_state_sp();
  dynamics_control();

  // cycle publisher
  //cyclePublisherBNO(bno_data);
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
  mean_rounds = mean_rounds * 2 * M_PI * veh_att.rwheel; // m/s;
  return mean_rounds;
}

float v_veh() {
  float trusted = 0;
  float vsum = 0;
  calc_traction();
  for (uint8_t i = 0; i<4; i++) {
    if (!traction_lost[i]) {
      trusted = trusted + 1;
      vsum = vsum + v_norm(i);
    }
  }
  return vsum / trusted;
}

float v_norm(uint8_t ind) { // normalized wheelspeed to center of gravity
  return v_wheel(ind) / veh_state_sp.rturnw[ind] * veh_state_sp.r;
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

  if (check_arm_state()) {
    float steer[4];
    calc_servo_angles(steer); // 4 elements
    torque_vectoring();
    traction_control();

    actor_comms.servo_angles[FRONT_LEFT] = steer[FRONT_LEFT];
    actor_comms.servo_angles[FRONT_RIGHT] = steer[FRONT_RIGHT];
    actor_comms.servo_angles[REAR_LEFT] = steer[REAR_LEFT];
    actor_comms.servo_angles[REAR_RIGHT] = steer[REAR_RIGHT];
    
  } else {    
    veh_state_ac.amps[FRONT_LEFT]  = 0;
    veh_state_ac.amps[FRONT_RIGHT] = 0;
    veh_state_ac.amps[REAR_LEFT]   = 0;
    veh_state_ac.amps[REAR_RIGHT]  = 0;
    
    actor_comms.servo_angles[FRONT_LEFT] = 0;
    actor_comms.servo_angles[FRONT_RIGHT] = 0;
    actor_comms.servo_angles[REAR_LEFT] = 0;
    actor_comms.servo_angles[REAR_RIGHT] = 0;
  }
  
  actor_comms.motor_amps[FRONT_LEFT]  = veh_state_ac.amps[FRONT_LEFT];
  actor_comms.motor_amps[FRONT_RIGHT] = veh_state_ac.amps[FRONT_RIGHT];
  actor_comms.motor_amps[REAR_LEFT]   = veh_state_ac.amps[REAR_LEFT];
  actor_comms.motor_amps[REAR_RIGHT]  = veh_state_ac.amps[REAR_RIGHT];
  
}


void torque_vectoring() {
  #define ACC_FACTOR 0.3
  float tv_factor = configuration.tvFactor * constrain(RC_coms.steer_f+RC_coms.steer_r,-1,1);
  float acc_factor = sgn(veh_state_sp.amps[0])*ACC_FACTOR;
  veh_state_sp.amps[FRONT_LEFT]  = veh_state_sp.amps[FRONT_LEFT] * (1+tv_factor-acc_factor);
  veh_state_sp.amps[FRONT_RIGHT] = veh_state_sp.amps[FRONT_RIGHT] * (1-tv_factor-acc_factor);
  veh_state_sp.amps[REAR_LEFT]   = veh_state_sp.amps[REAR_LEFT] * (1+tv_factor+acc_factor);
  veh_state_sp.amps[REAR_RIGHT]  = veh_state_sp.amps[REAR_RIGHT] * (1-tv_factor+acc_factor);
}

void traction_control() {
  #define MODIFIER_STEP 0.1
  static float modifier[4] = {1,1,1,1};
  calc_traction();
  for (uint8_t ind = 0; ind < 4; ind++) {
    if (traction_lost[ind]) modifier[ind] = constrain(modifier[ind] - MODIFIER_STEP,0,1);
    else                    modifier[ind] = constrain(modifier[ind] + MODIFIER_STEP,0,1);
    veh_state_ac.amps[ind]  = veh_state_sp.amps[ind] * modifier[ind];
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

uint8_t check_arm_state() {
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS
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

void calc_speed_pid() {
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
  veh_state_sp.v = v_sp;

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
  veh_state_sp.amps[0] = main_amps;
  veh_state_sp.amps[1] = main_amps;
  veh_state_sp.amps[2] = main_amps;
  veh_state_sp.amps[3] = main_amps;
}

void calc_steer_basic() { 
  float s_sp[2] = {0,0};
  if (  RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL) {
    s_sp[0] = -RC_coms.steer_f*MAX_STEER_ANGLE;
    s_sp[1] = RC_coms.steer_r*MAX_STEER_ANGLE;
  } else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS
          || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS){
    if (NUC_drive_coms.steer_type == 1) {
      NUC_drive_coms.steer = constrain(NUC_drive_coms.steer,-MAX_STEER_ANGLE,MAX_STEER_ANGLE);
      s_sp[0] = NUC_drive_coms.steer;
      s_sp[1] = NUC_drive_coms.steer;
    } else {
      float dist[2];
      steer_angle_distribution(dist); // 2 elements
      NUC_drive_coms.steer = constrain(NUC_drive_coms.steer,-MAX_STEER_ANGLE,MAX_STEER_ANGLE);
      float rest_angle = 0;
      s_sp[0] = dist[0] * NUC_drive_coms.steer - rest_angle;
      s_sp[1] = rest_angle;
    }
  } else {
    s_sp[0] = 0;
    s_sp[1] = 0;
  }
  veh_state_sp.delta[FRONT_LEFT]  = s_sp[0];
  veh_state_sp.delta[FRONT_RIGHT] = s_sp[0];
  veh_state_sp.delta[REAR_LEFT]   = s_sp[1];
  veh_state_sp.delta[REAR_RIGHT]  = s_sp[1];

  veh_state_ac.delta[FRONT_LEFT]  = veh_state_sp.delta[FRONT_LEFT];
  veh_state_ac.delta[FRONT_RIGHT] = veh_state_sp.delta[FRONT_RIGHT];
  veh_state_ac.delta[REAR_LEFT]   = veh_state_sp.delta[REAR_LEFT];
  veh_state_ac.delta[REAR_RIGHT]  = veh_state_sp.delta[REAR_RIGHT];
}

void calc_servo_angles(float *servo_angles) {// servo_angles float[4])
  if (veh_state_sp.delta[FRONT_LEFT] > 0)  servo_angles[FRONT_LEFT]  = -veh_state_ac.delta[FRONT_LEFT]*AC_TO_SERV_INN; // left turn
  else                                     servo_angles[FRONT_LEFT]  = -veh_state_ac.delta[FRONT_LEFT]*AC_TO_SERV_OUT; // right turn
  if (veh_state_sp.delta[FRONT_RIGHT] > 0) servo_angles[FRONT_RIGHT] = -veh_state_ac.delta[FRONT_RIGHT]*AC_TO_SERV_OUT; // left turn
  else                                     servo_angles[FRONT_RIGHT] = -veh_state_ac.delta[FRONT_RIGHT]*AC_TO_SERV_INN; // right turn
  if (veh_state_sp.delta[REAR_LEFT] > 0)   servo_angles[REAR_LEFT]   = -veh_state_ac.delta[REAR_LEFT]*AC_TO_SERV_OUT; // right turn
  else                                     servo_angles[REAR_LEFT]   = -veh_state_ac.delta[REAR_LEFT]*AC_TO_SERV_INN; // left turn
  if (veh_state_sp.delta[REAR_RIGHT] > 0)  servo_angles[REAR_RIGHT]  = -veh_state_ac.delta[REAR_RIGHT]*AC_TO_SERV_INN; // right turn
  else                                     servo_angles[REAR_RIGHT]  = -veh_state_ac.delta[REAR_RIGHT]*AC_TO_SERV_OUT; // left turn
}

void steer_angle_distribution(float *out) { // 2 elements
  out[0] = 1;
  out[1] = 0;
}

float deltav_STM() {
  return (veh_state_sp.delta[0]+veh_state_sp.delta[1])/2;
}

float deltah_STM() {
  return (veh_state_sp.delta[2]+veh_state_sp.delta[3])/2;
}

void calc_geometric_data() {
  veh_state_sp.l  = veh_att.l0 * (deltav_STM()/(deltav_STM()-deltah_STM()+0.001)); // "+0.001" is for avoiding division by 0
  veh_state_sp.lh = veh_state_sp.l-veh_att.lv;
  veh_state_sp.rl = veh_state_sp.l/tan(deg2rad(deltav_STM()));
  veh_state_sp.r  = sqrt(pow(veh_state_sp.l - veh_att.lv,2)+pow(veh_state_sp.rl,2));
  veh_state_sp.rturnw[FRONT_LEFT]  = sqrt(pow(veh_state_sp.rl,2)+pow(veh_state_sp.l,2))-veh_att.s/2;
  veh_state_sp.rturnw[FRONT_RIGHT] = sqrt(pow(veh_state_sp.rl,2)+pow(veh_state_sp.l,2))+veh_att.s/2;
  veh_state_sp.rturnw[REAR_LEFT]   = sqrt(pow(veh_state_sp.rl,2)+pow(veh_att.l0-veh_state_sp.l,2))-veh_att.s/2;
  veh_state_sp.rturnw[REAR_RIGHT]  = sqrt(pow(veh_state_sp.rl,2)+pow(veh_att.l0-veh_state_sp.l,2))+veh_att.s/2;
}

void calc_body_state_sp() {

/*
TODO list:
typedef struct {
  float r;              done
  float rl;             done
  float delta[4];       done - simple parallel steering 
  float amps[4];        done - all 4 the same
  float l;              done
  float lh;             done
  float v;              done
  float beta;
  float phi_p;
  float ay;
  float ax;

  float rturnw[4]
} body_state_t;
*/
  calc_steer_basic();
  calc_speed_pid();
  calc_geometric_data();
}

void calc_traction() {
  #define SLIP_THRESHOLD_ABS 1
  #define SLIP_THRESHOLD_REL 1
  float v_ref = 0;
  // decide if acceleration or deceleration
  if ((veh_state_sp.amps[0]+veh_state_sp.amps[1]+veh_state_sp.amps[2]+veh_state_sp.amps[3]) / 4 > 0) {
    v_ref = min(min(v_norm(0),v_norm(1)), min(v_norm(2),v_norm(3)));
  } else {
    v_ref = max(max(v_norm(0),v_norm(1)), max(v_norm(2),v_norm(3)));
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (abs(v_norm(i)-v_ref) >= SLIP_THRESHOLD_ABS && abs((v_norm(i)-v_ref) / v_ref) >= SLIP_THRESHOLD_REL) {
      traction_lost[i] = true;
    }
  }
}

