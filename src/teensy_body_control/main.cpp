#include "Arduino.h"
#include "phoenix_can_shield.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
//#include <VescUart.h>
#include "Teensy3_2_pwm.h"
#include "parameter.hpp"
#include <math.h>
#include "vuart.h"

// CAN Node settings
static constexpr uint32_t nodeID = 101;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char *nodeName = "org.phoenix.body_control";

// application settings
static constexpr float framerate = 100;

// Driving dynamics
#define FRONT_LEFT MotorState::POS_FRONT_LEFT
#define FRONT_RIGHT MotorState::POS_FRONT_RIGHT
#define REAR_LEFT MotorState::POS_REAR_LEFT
#define REAR_RIGHT MotorState::POS_REAR_RIGHT

typedef struct
{
  float thr = 0;
  float steer_f = 0;
  float steer_r = 0;
  float drive_state = 0;
  float aux_mode = 3;
} drive_comm_t;
drive_comm_t RC_coms;

typedef struct
{
  float lin_vel = 0; // m/s
  float steer_f = 0; // deg/s
  float steer_r = 0;
  uint8_t blink = 0; // 0 - none, 1 - left, 2 - right, 3 - both
} NUC_drive_coms_t;
NUC_drive_coms_t NUC_drive_coms;

typedef struct
{
  float v_sp = 0;
  float v_is = 0;
} v_t;
v_t speed;
// Vesc
static struct actor_comm_t
{
  float motor_amps[4] = {0, 0, 0, 0};
  float servo_angles[4] = {90, 90, 90, 90};
  float steer_angles[2] = {0, 0};
} actor_comms;

bool breaking = false;

bldcMeasure measuredVal_motor[4];

// Power
int power_update_rate = 5;
MonotonicTime last_power_update = MonotonicTime::fromMSec(0);
#define CELL4_PIN A0
#define CURR_PIN A1
#define Cell4_FACTOR 0.0052815755  // 10k + 1k8
#define CURR_FACTOR 0.00161133 / 4 // 0R01 + 200V/V

// BNO055 imu
Adafruit_BNO055 bno055 = Adafruit_BNO055();

// servos for steering
PWM_T32 steering_servo[2];
float steering_servo_position[2];
float steering_servo_offset[2] = {98, 92};
uint8_t steering_servo_pin[2] = {10, 9};
#define MAX_STEER_ANGLE 34 // degrees
#define MAX_STEER_SERVO_INNER 48  // absolute max 50° -> 37° wheel
#define MAX_STEER_SERVO_OUTER 35  // absolute max 40° -> 37° wheel

// lights
#define PWM_LIGHT_PIN 5
#define CLEAR_BIT
enum light_pwm_order {
  PWM_NOT_USED0,
  PWM_NOT_USED1,
  PWM_NOT_USED2,
  PWM_NOT_USED3,
  PWM_NOT_USED4,
  PWM_OFFSET,
  PWM_ODD_PARITY0,
  PWM_ODD_PARITY1,
  PWM_BLINK_LEFT,
  PWM_BLINK_RIGHT,
  PWM_BREAK,
  PWM_RC_LED,
  PWM_ARM,
};
uint16_t pwm_register = B01100000;  // initialize offset bit and odd-parity bit
PWM_T32 lights_pwm;

// routine header
float calc_speed_pid();
void dynamics_control();
void vesc_command();
uint8_t check_arm_state();
void calc_steer(float *);
void steer_angle_distribution(float *);
float v_veh();
void servo_command();
void process_lights();

#include "Publisher.h"
#include "Subscriber.h"

void setup()
{
  //get EEPROM Parameters
  readParamsFromEEPROM();

  Serial.begin(115200);
  // setup UART port for vesc
  Serial1.begin(250000);
  Serial3.begin(250000);
  SetSerialPort(&Serial1, &Serial3);
  //SetDebugSerialPort(&Serial);

  // lights
  /*
  ppm_light.begin(PPM_LIGHT_PIN);
  ppm_light.write(LIGHT_FRONT, ON_PPM);
  ppm_light.write(LIGHT_REAR, OFF_PPM);
  ppm_light.write(BREAK, OFF_PPM);
  ppm_light.write(BLINK_LEFT, OFF_PPM);
  ppm_light.write(BLINK_RIGHT, OFF_PPM);
  ppm_light.write(RC_LED, OFF_PPM);
  */
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

  // set up servos
  steering_servo_offset[FRONT_LEFT] = configuration.steeringOff_FL + 90;
  steering_servo_offset[FRONT_RIGHT] = configuration.steeringOff_FR + 90;

  steering_servo[FRONT_LEFT].attach(steering_servo_pin[FRONT_LEFT]);
  steering_servo[FRONT_RIGHT].attach(steering_servo_pin[FRONT_RIGHT]);

  // lights
  lights_pwm.attach(PWM_LIGHT_PIN);
  lights_pwm.writeMicros(pwm_register);
}

imu_t bno_data;

uint32_t t_ = 0;
void loop()
{

  // wait in cycle
  uint32_t t = micros();
  uint32_t elapsed_time = (t - t_);
  float cpu_load = (float)elapsed_time / (1000000. / (float)framerate);
  setRGBled((uint8_t)(cpu_load * 255), 255 - (uint8_t)(cpu_load * 255), 0);
  //Serial.print("CPU Load: "); Serial.print(cpu_load); Serial.println(" \%");
  if (((1000000. / (float)framerate) - elapsed_time) >= 4000.)
  {
    delayMicroseconds((uint32_t)round((1000000. / (float)framerate) - elapsed_time - 4000.));
  }
  // request vesc status
  vesc_send_status_request(0);
  vesc_send_status_request(1);
  delayMicroseconds(3800); // wait, so the esc has time to answer, even when the cycle time is too high
  cycleWait(framerate);
  t_ = micros();

  // get RC data, high level commands, motor telemetry rear motors
  cycleNode(node);

  // update motor front left information
  switch (VescUartGetValue(measuredVal_motor[FRONT_LEFT], FRONT_LEFT))
  {
  case COMM_GET_VALUES:
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_LEFT], FRONT_LEFT);
    break;
  default:
    break;
  }

  // update motor front right information
  switch (VescUartGetValue(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT))
  {
  case COMM_GET_VALUES:
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT);
    break;
  default:
    break;
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
  dynamics_control();

  // cycle publisher
  //cyclePublisherBNO(bno_data);
  cyclePublisher_Actor_Comms(actor_comms);

  // set the connected Motors
  vesc_command();
  servo_command();

  // spam Power info
  cyclePublisher_Power();

  // toggle heartbeat
  toggleHeartBeat();
  // update LED command

  process_lights();
  cyclePublisher_Drive_State(v_veh(), actor_comms.steer_angles[0], actor_comms.steer_angles[1]);
}

void process_lights() {
  // initialize offset bit and odd-parity bit
  pwm_register = B01100000;

  // set all relevant bits in the light command register
  uint8_t parity_count = 0;
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS) {
    bitSet(pwm_register, PWM_RC_LED);
    parity_count ++;
  }
  if (check_arm_state()) {
    bitSet(pwm_register, PWM_ARM);
    parity_count++;
  }
  if (NUC_drive_coms.blink == 1 || NUC_drive_coms.blink == 3) {
    bitSet(pwm_register, PWM_BLINK_LEFT);
    parity_count++;
  }
  if (NUC_drive_coms.blink == 2 || NUC_drive_coms.blink == 3) {
    bitSet(pwm_register, PWM_BLINK_RIGHT);
    parity_count++;
  }
  if (NUC_drive_coms.blink == 2 || NUC_drive_coms.blink == 3) {
    bitSet(pwm_register, PWM_BLINK_RIGHT);
    parity_count++;
  }
  if (breaking) {
    bitSet(pwm_register, PWM_BREAK);
    parity_count++;
  }

  // calculate odd-parity
  if (parity_count % 2 == 1) {
    // odd number of bits allready -> parity bit can be cleared
    bitClear(pwm_register, PWM_ODD_PARITY0);
  }
  bitClear(pwm_register, PWM_ODD_PARITY1); // allways 0 

  // write the light commands
  lights_pwm.writeMicros(pwm_register);
}

#define WHEEL_RADIUS_M 0.033
// acceleration directions in the car
float a_x_imu()
{                                    //       _____.
  return (float)bno_data.lin_acc[0]; //      /_    .
} //      |_|   ^ X
  //     |      |
float a_y_imu()
{                                    //     |      |     Y
  return (float)bno_data.lin_acc[0]; //     |    --+----->
} //     | _    |
  //      |_|   .
  //     |______.

float v_wheel(uint8_t wheelindex)
{
  float mean_rounds = (float)measuredVal_motor[wheelindex].rpm / 7; // RPM
  mean_rounds /= 60;                                                // RPS
  mean_rounds = mean_rounds * 2 * M_PI * WHEEL_RADIUS_M;            // m/s;
  return mean_rounds;
}

float v_veh()
{
  //if (a_x_imu()>=0) return (v_wheel(REAR_LEFT)+v_wheel(REAR_RIGHT)) / 2;
  //else              return (v_wheel(FRONT_LEFT)+v_wheel(FRONT_RIGHT)) / 2;
  return (v_wheel(FRONT_LEFT) + v_wheel(FRONT_RIGHT) + v_wheel(REAR_LEFT) + v_wheel(REAR_RIGHT)) / 4;
}

float a_wheel(uint8_t wheelindex)
{
  static float v = 0;
  static float prevt, t, dt, vprev;
  prevt = t;
  t = (float)micros() / 1000000.;
  dt = t - prevt;
  vprev = v;
  v = v_wheel(wheelindex);
  return (v - vprev) / dt;
}

void dynamics_control()
{

  if (check_arm_state())
  {
  #define ACC_FACTOR 0.3
    float main_amps = calc_speed_pid();
    float steer[4];
    calc_steer(steer); // 4 elements
    float tv_factor = configuration.tvFactor * constrain(RC_coms.steer_f, -1, 1);
    float acc_factor = sgn(main_amps) * ACC_FACTOR;
    actor_comms.motor_amps[FRONT_LEFT] = main_amps * (1 + tv_factor - acc_factor);
    actor_comms.motor_amps[FRONT_RIGHT] = main_amps * (1 - tv_factor - acc_factor);
    actor_comms.motor_amps[REAR_LEFT] = main_amps * (1 + tv_factor + acc_factor) / 1.7;
    actor_comms.motor_amps[REAR_RIGHT] = main_amps * (1 - tv_factor + acc_factor) / 1.7;

    actor_comms.servo_angles[FRONT_LEFT] = steer[FRONT_LEFT];
    actor_comms.servo_angles[FRONT_RIGHT] = steer[FRONT_RIGHT];
    actor_comms.servo_angles[REAR_LEFT] = steer[REAR_LEFT];
    actor_comms.servo_angles[REAR_RIGHT] = steer[REAR_RIGHT];
  }
  else
  {
    actor_comms.motor_amps[FRONT_LEFT] = 0;
    actor_comms.motor_amps[FRONT_RIGHT] = 0;
    actor_comms.motor_amps[REAR_LEFT] = 0;
    actor_comms.motor_amps[REAR_RIGHT] = 0;

    actor_comms.servo_angles[FRONT_LEFT] = 0;
    actor_comms.servo_angles[FRONT_RIGHT] = 0;
    actor_comms.servo_angles[REAR_LEFT] = 0;
    actor_comms.servo_angles[REAR_RIGHT] = 0;
    //ppm_light.write(BREAK, OFF_PPM);
  }
}

void vesc_command()
{
  #define BREAK_THRESHOLD 25
  static uint32_t break_count = 0;
  // break if speed is to reduce. otherwise set accelerating current. helps the controller not to overshoot if lifted for example
  if (abs(speed.v_sp) >= abs(speed.v_is))
  {
    VescUartSetCurrent(actor_comms.motor_amps[FRONT_LEFT], FRONT_LEFT);
    VescUartSetCurrent(actor_comms.motor_amps[FRONT_RIGHT], FRONT_RIGHT);
    if (break_count > 0) {
      break_count--;
    }
  }
  else
  {
    VescUartSetCurrentBrake(abs(actor_comms.motor_amps[FRONT_LEFT]), FRONT_LEFT);
    VescUartSetCurrentBrake(abs(actor_comms.motor_amps[FRONT_RIGHT]), FRONT_RIGHT);
    if (break_count < (BREAK_THRESHOLD * 1.5)) {
      break_count++;
    }
  }

  if (break_count > BREAK_THRESHOLD) {
    breaking = true;
  }
  else {
    breaking = false;
  }
}


void servo_command()
{
  steering_servo[FRONT_LEFT].write(steering_servo_offset[FRONT_LEFT] + actor_comms.servo_angles[FRONT_LEFT]);
  steering_servo[FRONT_RIGHT].write(steering_servo_offset[FRONT_RIGHT] + actor_comms.servo_angles[FRONT_RIGHT]);
}

void traction_control()
{
  if (true)
    ;
}

uint8_t check_arm_state()
{
  if (RC_coms.aux_mode == RemoteControl::AUX_MODE_CENTER || RC_coms.aux_mode == RemoteControl::AUX_MODE_UP)
  {
    if (!steering_servo[FRONT_LEFT].attached() || !steering_servo[FRONT_RIGHT].attached())
    {
      // setup servos for steering
      steering_servo[FRONT_LEFT].attach(steering_servo_pin[FRONT_LEFT]);
      steering_servo[FRONT_RIGHT].attach(steering_servo_pin[FRONT_RIGHT]);
    }
    return true;
  }
  else
  {
    steering_servo[FRONT_LEFT].detach();
    steering_servo[FRONT_RIGHT].detach();
    return false;
  }
}

float calc_speed_pid()
{
  static float main_amps;
  static float P_, i_, i_integral, I_, d_, D_, speed_PID, v_error, v_err_prev, derr;
  static float prevt, t, dt;
  prevt = t;
  t = (float)micros() / 1000000.;
  dt = t - prevt;

  speed.v_is = v_veh();

  //calculate Velocity error
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS)
  {
    speed.v_sp = constrain(RC_coms.thr * configuration.maxSpeed, -configuration.maxSpeed, configuration.maxSpeed);
  }
  else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS)
  {
    speed.v_sp = constrain(NUC_drive_coms.lin_vel, -configuration.maxSpeed, configuration.maxSpeed);
  }
  else
  {
    speed.v_sp = 0;
  }
  v_error = speed.v_sp - speed.v_is;
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
  if (main_amps > configuration.maxMotorAmps)
    main_amps = configuration.maxMotorAmps;
  else if (main_amps < -configuration.maxMotorAmps)
    main_amps = -configuration.maxMotorAmps;
  //Serial.print(v_veh());
  //Serial.print("\t");
  //Serial.print(P_);
  //Serial.print("\t");
  //Serial.print(D_);
  //Serial.print("\t");
  //Serial.println(main_amps);
  return main_amps;
}

void calc_steer(float *servo_angles) // servo_angles float[4]
{
  float s_sp[2] = {0, 0};
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL)
  {
    s_sp[0] = -RC_coms.steer_f * MAX_STEER_ANGLE;
    s_sp[1] = RC_coms.steer_r * MAX_STEER_ANGLE; //RC_coms.steer_r*MAX_STEER_ANGLE;
  }
  else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS)
  {
    NUC_drive_coms.steer_f = constrain(NUC_drive_coms.steer_f, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    NUC_drive_coms.steer_r = constrain(NUC_drive_coms.steer_r, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    s_sp[0] = NUC_drive_coms.steer_f;
    s_sp[1] = NUC_drive_coms.steer_r;
  }
  else
  {
    s_sp[0] = 0;
    s_sp[1] = 0;
  }
  actor_comms.steer_angles[0] = s_sp[0];
  actor_comms.steer_angles[1] = s_sp[1];
  if (s_sp[0] > 0)
  {
    servo_angles[FRONT_LEFT] = -s_sp[0] / MAX_STEER_ANGLE * MAX_STEER_SERVO_OUTER;
    servo_angles[FRONT_RIGHT] = -s_sp[0] / MAX_STEER_ANGLE * MAX_STEER_SERVO_INNER;
  }
  else
  {
    servo_angles[FRONT_LEFT] = -s_sp[0] / MAX_STEER_ANGLE * MAX_STEER_SERVO_INNER;
    servo_angles[FRONT_RIGHT] = -s_sp[0] / MAX_STEER_ANGLE * MAX_STEER_SERVO_OUTER;
  }
  if (s_sp[1] > 0)
  {
    servo_angles[REAR_LEFT] = -s_sp[1] / MAX_STEER_ANGLE * 0;  //MAX_STEER_SERVO_INNER;
    servo_angles[REAR_RIGHT] = -s_sp[1] / MAX_STEER_ANGLE * 0; //MAX_STEER_SERVO_OUTER;
  }
  else
  {
    servo_angles[REAR_LEFT] = -s_sp[1] / MAX_STEER_ANGLE * 0;  //MAX_STEER_SERVO_OUTER;
    servo_angles[REAR_RIGHT] = -s_sp[1] / MAX_STEER_ANGLE * 0; //MAX_STEER_SERVO_INNER;
  }
}
