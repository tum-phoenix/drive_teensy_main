// includes
  #include "Arduino.h"
  #include "phoenix_can_shield.h"
  #include <Adafruit_BNO055.h>
  #include <Adafruit_Sensor.h>
  #include <utility/imumaths.h>
  #include "PWMServo.h"
  #include "parameter.hpp"
  #include <math.h>
  #include "vuart.h"
  #include <Filters.h>
//

// CAN Node settings
  static constexpr uint32_t nodeID = 101;
  static constexpr uint8_t swVersion = 1;
  static constexpr uint8_t hwVersion = 1;
  static const char *nodeName = "org.phoenix.body_control";
//

// application settings
  static constexpr float framerate = 50;
//

// Driving dynamics
  #define FRONT_LEFT MotorState::POS_FRONT_LEFT
  #define FRONT_RIGHT MotorState::POS_FRONT_RIGHT
  #define REAR_LEFT MotorState::POS_REAR_LEFT
  #define REAR_RIGHT MotorState::POS_REAR_RIGHT

  #define WHEEL_RADIUS_M 0.033 // dynamic wheel radius

  #define NUC_DRIVE_COM_DEAD_TIME 250
  #define RC_COM_DEAD_TIME 250
  typedef struct
  {
    float thr = 0;
    float steer_f = 0;
    float steer_r = 0;
    float drive_state = 0;
    float drive_state_prev = 0;
    float aux_mode = 3;
    float last_update = 0;
  } drive_comm_t;
  drive_comm_t RC_coms;

  typedef struct
  {
    float lin_vel = 0; // m/s
    float steer_f = 0; // deg/s
    float steer_r = 0;
    uint8_t blink = 0; // 0 - none, 1 - left, 2 - right, 3 - both
    uint32_t last_update = 0;
  } NUC_drive_coms_t;
  NUC_drive_coms_t NUC_drive_coms;

  static struct car_vel_t
  {
    float is;
    float sp;
  } car_vel;
  double x_dist = 0;
//

// Parking
  uint8_t got_parking_command = 0;
  typedef struct
  {
    float lot_size = 0;
    float lot_dist = 0;
  } parking_t;
  parking_t parking;
  float parking_steer_f = 0;
  float parking_steer_r = 0;
//

// Vesc
  static struct actor_comm_t
  {
    float motor_amps[4] = {0, 0, 0, 0};
    float servo_angles[4] = {90, 90, 90, 90};
    float steer_angles[2] = {0, 0};
  } actor_comms;

  bldcMeasure measuredVal_motor[4];
  uint32_t last_mot_state_update[4] = {0, 0, 0, 0};
  boolean braking = false;
//

// Power
  int power_update_rate = 2;
  MonotonicTime last_power_update = MonotonicTime::fromMSec(0);
  #define CELL4_PIN A0
  #define CURR_PIN A1
  #define Cell4_FACTOR 0.0052815755  // 10k + 1k8
  #define CURR_FACTOR 0.00161133 / 4 // 0R01 + 200V/V
//

// BNO055 imu
  Adafruit_BNO055 bno055 = Adafruit_BNO055();
//

// servos for steering
  PWMServo steering_servo[2];
  float steering_servo_position[2];
  float steering_servo_offset[2] = {98, 92};
  uint8_t steering_servo_pin[2] = {10, 9};
  #define MAX_STEER_ANGLE 32 // degrees
  #define MAX_STEER_SERVO_INNER 50
  #define MAX_STEER_SERVO_OUTER 30

  // filters out changes faster that 2 Hz.
  float filterFrequency_servo = 2.0;

  // create a one pole (RC) lowpass filter
  FilterOnePole lowpassFilterServoFront(LOWPASS, filterFrequency_servo);
  FilterOnePole lowpassFilterServoRear(LOWPASS, filterFrequency_servo);
//

// lights
  #define PWM_LIGHT_PIN 5
  enum light_pwm_order
  {
    PWM_NOT_USED_0,
    PWM_NOT_USED_1,
    PWM_NOT_USED_2,
    PWM_NOT_USED_3,
    PWM_NOT_USED_4,
    PWM_OFFSET,
    PWM_O_PARITY,
    PWM_BLANK,
    PWM_BLINK_LEFT,
    PWM_BLINK_RIGHT,
    PWM_BRAKE,
    PWM_RC_LED,
    PWM_ARM
  };
  uint32_t light_com = B01100000;
  PWMServo pwm_lights;
//

// header
  float calc_speed_pid();
  void dynamics_control();
  void vesc_command();
  uint8_t check_arm_state();
  void calc_steer(float *);
  void steer_angle_distribution(float *);
  float v_veh();
  void process_lights();
  void update_lights(uint32_t command);
  bool disable_motors();
  void set_Speed();
  void drive_things();
  void parking_func();

  #include "Publisher.h"
  #include "Subscriber.h"
//

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
  pwm_lights.attach(PWM_LIGHT_PIN, 96, 8032);
  update_lights(light_com);

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

  steering_servo_offset[FRONT_LEFT] = configuration.steeringOff_FL + 90;
  steering_servo_offset[FRONT_RIGHT] = configuration.steeringOff_FR + 90;

  steering_servo[FRONT_LEFT].attach(steering_servo_pin[FRONT_LEFT]);
  steering_servo[FRONT_RIGHT].attach(steering_servo_pin[FRONT_RIGHT]);
}

imu_t bno_data;

uint32_t t_ = 0;
void loop()
{
  // wait in cycle
  uint32_t t = micros();
  float cpu_load = (float)(t - t_) / (1000000. / (float)framerate);
  //setRGBled((uint8_t)(cpu_load * 255), 255 - (uint8_t)(cpu_load * 255), 0);
  //Serial.print("CPU Load: "); Serial.print(cpu_load); Serial.println(" \%");
  cycleWait(framerate);
  t_ = micros();

  // get RC data, high level commands, motor telemetry rear motors
  cycleNode(node);

  vesc_send_status_request(0);
  vesc_send_status_request(1);
  delayMicroseconds(7000);
  // update motor front left information
  switch (VescUartGetValue(measuredVal_motor[FRONT_LEFT], FRONT_LEFT))
  {
  case COMM_GET_VALUES:
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_LEFT], FRONT_LEFT);
    last_mot_state_update[FRONT_LEFT] = millis();
    break;
  default:
    break;
  }

  // update motor front right information
  switch (VescUartGetValue(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT))
  {
  case COMM_GET_VALUES:
    cyclePublisher_Mot_State(measuredVal_motor[FRONT_RIGHT], FRONT_RIGHT);
    last_mot_state_update[FRONT_RIGHT] = millis();
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
  steering_servo[FRONT_LEFT].write(steering_servo_offset[FRONT_LEFT] + actor_comms.servo_angles[FRONT_LEFT]);
  steering_servo[FRONT_RIGHT].write(steering_servo_offset[FRONT_RIGHT] + actor_comms.servo_angles[FRONT_RIGHT]);

  // spam Power info
  cyclePublisher_Power();

  // toggle heartbeat
  toggleHeartBeat();

  // update LED command
  process_lights();

  cyclePublisher_Drive_State(v_veh(), actor_comms.steer_angles[0], actor_comms.steer_angles[1]);
  
  //parking_func();
}

/* acceleration directions in the car
 *       _____.
 *      /_    .
 *      |_|   ^ X
 *     |      |
 *     |      |     Y
 *     |    --+----->
 *     | _    |
 *      |_|   .
 *     |______.
*/

float a_x_imu()
{
  return (float)bno_data.lin_acc[0];
}
float a_y_imu()
{
  return (float)bno_data.lin_acc[1];
}

float v_wheel(uint8_t wheelindex)
{
  float mean_rounds = (float)measuredVal_motor[wheelindex].erpm / 7; // RPM
  mean_rounds /= 60;                                                 // RPS
  mean_rounds = mean_rounds * 2 * M_PI * WHEEL_RADIUS_M;             // m/s;
  return mean_rounds;
}

float v_veh()
{
  #define MOT_STATE_TIMEOUT 200 // timeout for last motor update to be valid
  uint32_t cur_time = millis();
  float sum_vel = 0;
  float devider = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    if (millis() - last_mot_state_update[i] < MOT_STATE_TIMEOUT)
    {
      sum_vel += v_wheel(i);
      devider++;
    }
  }
  if (devider)
  {
    car_vel.is = sum_vel / devider;
  }
  else
  {
    car_vel.is = 0;
  }
  return car_vel.is;
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
    float main_amps = calc_speed_pid();
    float steer[4];
    calc_steer(steer); // 4 elements
    float tv_factor = configuration.tvFactor * constrain(RC_coms.steer_f + RC_coms.steer_r, -1., 1.);
    float acc_factor = sgn(main_amps) * configuration.acFactor;
    float amp_add = 0;
    static int8_t brake_count = 0;
    if (sgn(main_amps) != sgn(v_veh()) && fabsf(main_amps) > 0.2)
    {
      if (brake_count < 6)
      {
        brake_count++;
      }
    }
    else
    {
      if (brake_count > 0)
      {
        brake_count--;
      }
    }

    if (brake_count >= 6)
    {
      braking = true;
    }
    else if (brake_count && braking)
    {
      braking = true;
    }
    else
    {
      braking = false;
    }

    /*
    if (fabsf(car_vel.sp) < fabsf(car_vel.is)){
      amp_add = 1000; // amp command set +1000 to make clear it is not a normal command, but braking in this case
    }
    */
    if (disable_motors() && RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL)
    {
      //main_amps = 0;
    }
    else
    {
      //setRGBled(0, 0, 0);
    }

    actor_comms.motor_amps[FRONT_LEFT] = (main_amps * (1 + tv_factor - acc_factor)) + amp_add;
    actor_comms.motor_amps[FRONT_RIGHT] = (main_amps * (1 - tv_factor - acc_factor)) + amp_add;
    actor_comms.motor_amps[REAR_LEFT] = (main_amps * (1 + tv_factor + acc_factor) / 1.7) + amp_add;
    actor_comms.motor_amps[REAR_RIGHT] = (main_amps * (1 - tv_factor + acc_factor) / 1.7) + amp_add;

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
  }
}

void vesc_command()
{
  if (actor_comms.motor_amps[FRONT_LEFT] > 500)
  {
    actor_comms.motor_amps[FRONT_LEFT] -= 1000;  // get the normal value back
    actor_comms.motor_amps[FRONT_RIGHT] -= 1000; // get the normal value back
    VescUartSetHandbrake(actor_comms.motor_amps[FRONT_LEFT], FRONT_LEFT);
    VescUartSetHandbrake(actor_comms.motor_amps[FRONT_RIGHT], FRONT_RIGHT);
  }
  else
  {
    VescUartSetCurrent(actor_comms.motor_amps[FRONT_LEFT], FRONT_LEFT);
    VescUartSetCurrent(actor_comms.motor_amps[FRONT_RIGHT], FRONT_RIGHT);
  }
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
    //if (!steering_servo[FRONT_LEFT].attached() || !steering_servo[FRONT_RIGHT].attached()) {
    // setup servos for steering
    //  steering_servo[FRONT_LEFT].attach(steering_servo_pin[FRONT_LEFT]);
    //  steering_servo[FRONT_RIGHT].attach(steering_servo_pin[FRONT_RIGHT]);
    //}
    return true;
  }
  else
  {
    //steering_servo[FRONT_LEFT].detach();
    //steering_servo[FRONT_RIGHT].detach();
    return false;
  }
}

float calc_speed_pid()
{
  static float main_amps, v_val;
  static float P_, i_, i_integral, I_, d_, D_, speed_PID, v_error, v_err_prev, derr;
  static float prevt, t, dt;
  prevt = t;
  t = (float)micros() / 1000000.;
  dt = t - prevt;

  set_Speed();
  v_val = v_veh();
  v_error = car_vel.sp - v_val;
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
    // apply an exponetial curve for RC steering for better feel
    s_sp[0] = -RC_coms.steer_f * MAX_STEER_ANGLE;
    s_sp[1] = RC_coms.steer_r * MAX_STEER_ANGLE;
  }
  else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS)
  {
    if (!got_parking_command)
    {
      NUC_drive_coms.steer_f = constrain(NUC_drive_coms.steer_f, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
      NUC_drive_coms.steer_r = constrain(NUC_drive_coms.steer_r, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
      s_sp[0] = NUC_drive_coms.steer_f;
      s_sp[1] = NUC_drive_coms.steer_r;
    }
    else {
       s_sp[0] = parking_steer_f;
       s_sp[1] = parking_steer_r;
    }
  }
  else
  {
    s_sp[0] = 0;
    s_sp[1] = 0;
  }
  // filter the servo signals
  //lowpassFilterServoFront.input(s_sp[0]);
  //s_sp[0] = lowpassFilterServoFront.output();
  //lowpassFilterServoRear.input(s_sp[1]);
  //s_sp[1] = lowpassFilterServoRear.output();

  // write the servo setpoints
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
    servo_angles[REAR_LEFT] = -s_sp[1] / MAX_STEER_ANGLE * MAX_STEER_SERVO_INNER;
    servo_angles[REAR_RIGHT] = -s_sp[1] / MAX_STEER_ANGLE * MAX_STEER_SERVO_OUTER;
  }
  else
  {
    servo_angles[REAR_LEFT] = -s_sp[1] / MAX_STEER_ANGLE * MAX_STEER_SERVO_OUTER;
    servo_angles[REAR_RIGHT] = -s_sp[1] / MAX_STEER_ANGLE * MAX_STEER_SERVO_INNER;
  }
}

void update_lights(uint32_t command)
{
  pwm_lights.write(map((float)command, 96., 8032., 0., 180.));
  //Serial.print(command, BIN); Serial.print("\t");
  //Serial.println(map((float)command, 96., 8032., 0., 180.));
}

void process_lights()
{
  light_com = B01100000;
  uint8_t parity_counter = 0;
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS)
  {
    bitSet(light_com, PWM_RC_LED);
    parity_counter++;
  }
  else
  {
    bitClear(light_com, PWM_RC_LED);
  }
  if (NUC_drive_coms.blink == 1 || NUC_drive_coms.blink == 3)
  {
    bitSet(light_com, PWM_BLINK_LEFT);
    parity_counter++;
  }
  else
  {
    bitClear(light_com, PWM_BLINK_LEFT);
  }
  if (NUC_drive_coms.blink == 2 || NUC_drive_coms.blink == 3)
  {
    bitSet(light_com, PWM_BLINK_RIGHT);
    parity_counter++;
  }
  else
  {
    bitClear(light_com, PWM_BLINK_RIGHT);
  }
  if (braking)
  {
    bitSet(light_com, PWM_BRAKE);
    parity_counter++;
  }
  else
  {
    bitClear(light_com, PWM_BRAKE);
  }
  if (check_arm_state())
  {
    bitSet(light_com, PWM_ARM);
    parity_counter++;
  }
  else
  {
    bitClear(light_com, PWM_ARM);
  }
  // calculate the odd parity
  if (parity_counter % 2 != 0)
  { // if we have an odd number allready, clear the parity bit
    bitClear(light_com, PWM_O_PARITY);
  }
  else
  {
    bitSet(light_com, PWM_O_PARITY);
  }

  update_lights(light_com);
}

bool disable_motors()
{
  #define MOT_DIS_THRESHOLD_V 0.001
  #define MOT_DIS_THRESHOLD_MS 1500
  static uint32_t start_millis = 0;
  static float thr_prev = 0;
  if (fabsf(RC_coms.thr) < MOT_DIS_THRESHOLD_V)
  {
    if (fabsf(thr_prev) >= MOT_DIS_THRESHOLD_V)
    {
      start_millis = millis();
      thr_prev = RC_coms.thr;
      return false;
    }
    if (millis() - start_millis >= MOT_DIS_THRESHOLD_MS)
    {
      thr_prev = RC_coms.thr;
      return true;
    }
  }
  thr_prev = RC_coms.thr;
  return false;
}

void set_Speed()
{
  //calculate Velocity error
  static float this_drive_state = 0;
  RC_coms.drive_state_prev = this_drive_state;
  this_drive_state = RC_coms.drive_state;
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL)
  {
    /*if (millis() - NUC_drive_coms.last_update < RC_COM_DEAD_TIME) // see, if updates are fresh enough
      {*/
    car_vel.sp = constrain(RC_coms.thr * configuration.maxSpeedRC, -configuration.maxSpeedRC, configuration.maxSpeedRC);
    /*}
      else 
      {
        car_vel.sp = 0;
      }*/
  }
  else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS)
  {
    /*if (millis() - NUC_drive_coms.last_update < RC_COM_DEAD_TIME) // see, if updates are fresh enough
      {*/
    car_vel.sp = constrain(RC_coms.thr * configuration.maxSpeedAuton, -configuration.maxSpeedAuton, configuration.maxSpeedAuton);
    /*}
      else 
      {
        car_vel.sp = 0;
      }*/
  }
  else if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_AUTONOMOUS)
  {
    if (!got_parking_command)
    {
      /*if (millis() - NUC_drive_coms.last_update < NUC_COM_DEAD_TIME) // see, if updates are fresh enough
        {*/
      car_vel.sp = constrain(NUC_drive_coms.lin_vel, -configuration.maxSpeedAuton, configuration.maxSpeedAuton);
      /*}
        else 
        {
          car_vel.sp = 0;
        }*/
    }
  }
  else
  {
    car_vel.sp = 0;
  }

  static bool waiting_to_stop = false;
  static uint32_t wait_to_stop_since = 0;
  if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL && (RC_coms.drive_state_prev == RemoteControl::DRIVE_MODE_AUTONOMOUS || RC_coms.drive_state_prev == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS))
  {
    waiting_to_stop = true;
    wait_to_stop_since = millis();
  }

  if (waiting_to_stop)
  {
    if (millis() - wait_to_stop_since < 800)
    {
      car_vel.sp = 0;
    }
    else
    {
      waiting_to_stop = false;
    }
  }
}

float x_veh()
{
  v_veh();
  static uint32_t lastupdate = 0;
  static uint32_t thisupdate = 0;
  lastupdate = thisupdate;
  thisupdate = micros();
  uint32_t dt = thisupdate-lastupdate; // micros
  double ds = (car_vel.is * (double)dt)/1000000.; // m
  double temp =  x_dist + ds;
  x_dist = temp;
  return x_dist;
}

void parking_func()
{
  #define STEER_X 0.25
  #define STRAIGHT_X 0.1
  #define PARK_SPEED 0.5
  uint8_t count = 0;
  float start_dist = 0;
  while (got_parking_command)
  {
    if (parking.lot_dist)
    {
      if (count == 0)
      {
        start_dist = x_veh();
        count++;
      }
      while (x_veh() - start_dist > 0.)
      {
        car_vel.sp = -PARK_SPEED;
        parking_steer_f = 0.;
        parking_steer_r = 0.;
        drive_things();
        delay(20);
      }
      float start_steer0 = x_veh();
      while (start_steer0 - x_veh() < configuration.park_s_x0)
      {
        car_vel.sp = -PARK_SPEED;
        parking_steer_f = configuration.park_steer0;
        parking_steer_r = configuration.park_steer0;
        drive_things();
        delay(20);
      }
      float start_steer1 = x_veh();
      while (start_steer1 - x_veh() < configuration.park_x0)
      {
        car_vel.sp = -PARK_SPEED;
        parking_steer_f = 0.;
        parking_steer_r = 0.;
        drive_things();
        delay(20);
      }
      float start_steer2 = x_veh();
      while (start_steer2 - x_veh() < configuration.park_s_x1)
      {
        car_vel.sp = -PARK_SPEED;
        parking_steer_f = configuration.park_steer0;
        parking_steer_r = configuration.park_steer0;
        drive_things();
        delay(20);
      }
      uint32_t begin_halt = millis();
      while (millis() - begin_halt < 2000)
      {
        car_vel.sp = 0;
        parking_steer_f = 0;
        parking_steer_r = 0;
        drive_things();
        light_com = B01100000;
        uint8_t parity_counter = 0;
        if (RC_coms.drive_state == RemoteControl::DRIVE_MODE_MANUAL || RC_coms.drive_state == RemoteControl::DRIVE_MODE_SEMI_AUTONOMOUS)
        {
          bitSet(light_com, PWM_RC_LED);
          parity_counter++;
        }
        else
        {
          bitClear(light_com, PWM_RC_LED);
        }
        bitSet(light_com, PWM_BLINK_LEFT);
        parity_counter++;
        
        bitSet(light_com, PWM_BLINK_RIGHT);
        parity_counter++;

        if (braking)
        {
          bitSet(light_com, PWM_BRAKE);
          parity_counter++;
        }
        else
        {
          bitClear(light_com, PWM_BRAKE);
        }
        if (check_arm_state())
        {
          bitSet(light_com, PWM_ARM);
          parity_counter++;
        }
        else
        {
          bitClear(light_com, PWM_ARM);
        }
        // calculate the odd parity
        if (parity_counter % 2 != 0)
        { // if we have an odd number allready, clear the parity bit
          bitClear(light_com, PWM_O_PARITY);
        }
        else
        {
          bitSet(light_com, PWM_O_PARITY);
        }

        update_lights(light_com);

        delay(20);
      }
      got_parking_command = 0;
    }
  }
}

void drive_things()
{
  dynamics_control();
  // cycle publisher
  //cyclePublisherBNO(bno_data);
  cyclePublisher_Actor_Comms(actor_comms);

  // set the connected Motors
  vesc_command();
  steering_servo[FRONT_LEFT].write(steering_servo_offset[FRONT_LEFT] + actor_comms.servo_angles[FRONT_LEFT]);
  steering_servo[FRONT_RIGHT].write(steering_servo_offset[FRONT_RIGHT] + actor_comms.servo_angles[FRONT_RIGHT]);

}