/** 
 * BLDC Motor Controller using SimpleFOC
 */
#include <Arduino.h>
#include "SimpleFOC.h"
#include "interval_timer.hpp"

/*********************************************************************/

#define VOLTAGE_POWER_SUPPLY        (12)

#define POLE_PAIR                   (1)
#define PHASE_RESISTANCE            (2.2)

#define PIN_PHASE_UH                (A_PHASE_UH)
#define PIN_PHASE_UL                (A_PHASE_UL)
#define PIN_PHASE_VH                (A_PHASE_VH)
#define PIN_PHASE_VL                (A_PHASE_VL)
#define PIN_PHASE_WH                (A_PHASE_WH)
#define PIN_PHASE_WL                (A_PHASE_WL)

#define LOWSIDE_CURSENSE_SHUNT                  (0.003)
#define LOWSIDE_CURSENSE_GAIN                   (-64.0/7.0)
#define LOWSIDE_CURSENSE_PIN_A_PHASE_ADC        (A_OP1_OUT)
#define LOWSIDE_CURSENSE_PIN_B_PHASE_ADC        (A_OP2_OUT)
#define LOWSIDE_CURSENSE_PIN_C_PHASE_ADC        (A_OP3_OUT)

// Debug options
#define ENABLE_FIND_POLE_PAIR                   (0)
#define  POLE_PAIRE_SEARCH_VOLTAGE              (0.5)
#define ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER     (0)
#define ENABLE_ROT_SENSOR_MONITOR               (0)

/*********************************************************************/

#ifndef COMMANDER_SERIAL
#define COMMANDER_SERIAL      Serial
#endif

#ifndef MONITOR_SERIAL
#define MONITOR_SERIAL        Serial
#endif

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL          Serial
#endif

/*********************************************************************/

static void init_serial(void);
static void init_wire(void);
static void init_rot_sensor(void);

static void init_motor_config(void);
static void do_system_reset(void);

static void find_pole_pair(void);
static void rot_sensor_pulse_len_checker(void);
static void rot_sensor_monitor(void);

static void encoder_monitor(void);

static void find_zero_position(void);

static void demo_move(void);
static void demo_move2(void);

/*********************************************************************/

BLDCMotor motor = BLDCMotor(
  #if !ENABLE_FIND_POLE_PAIR
    POLE_PAIR
    #ifdef PHASE_RESISTANCE
    , PHASE_RESISTANCE
    #endif
  #else
    1
  #endif
);

BLDCDriver6PWM driver = BLDCDriver6PWM(
  PIN_PHASE_UH,
  PIN_PHASE_UL,
  PIN_PHASE_VH,
  PIN_PHASE_VL,
  PIN_PHASE_WH,
  PIN_PHASE_WL
  );

LowsideCurrentSense currentSense = LowsideCurrentSense(
  LOWSIDE_CURSENSE_SHUNT,
  LOWSIDE_CURSENSE_GAIN,
  LOWSIDE_CURSENSE_PIN_A_PHASE_ADC,
  LOWSIDE_CURSENSE_PIN_B_PHASE_ADC,
  LOWSIDE_CURSENSE_PIN_C_PHASE_ADC
  );

MagneticSensorI2C rotSense = MagneticSensorI2C(
  // AS5600_I2C
  AS5048_I2C
  );

/*********************************************************************/

// Encoder monitor inverval
// < 0 : disable
// 0 < : monitor interval ms
int encoderMonitorIntervalMs_ = -1;
int preEncoderMonitorTimeMs_ = 0;

// SimpleFOC monitor enable
// == 0 : disable
// != 0 : enable
int enableSimpleFOCMonitor_ = 0;

// Demo mode enable flag
// == 0 : disable
// != 0 : enable
int enableDemo_ = 0;

// Find zero position flag
// == 0 : disable
// != 0 : enable
int enableFindZeroPosition_ = 0;

// instantiate the commander
Commander command = Commander(COMMANDER_SERIAL);
void doTarget(char* cmd) { command.motion(&motor, cmd); }
void doCommand(char* cmd) { command.motor(&motor, cmd); }
void doEncoder(char* cmd) { encoderMonitorIntervalMs_ = atoi(cmd); }
void doMonitor(char* cmd) { enableSimpleFOCMonitor_ = atoi(cmd); }
void doFindZeroPos(char* cmd) { enableFindZeroPosition_ = atoi(cmd); }
void doDemo(char* cmd) { enableDemo_ = atoi(cmd); }
void doReset(char* cmd) { do_system_reset(); }

// Zero position offset angle
float offsetZeroAngle_ = 0;
float offsetMaxAngle_ = 0;

/*********************************************************************/

void setup()
{
  init_serial();

  // wait for host ready
  //delay(3000);
  //Serial.printf("Start Program\n");

  init_wire();

  #if ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER
  rot_sensor_pulse_len_checker();
  while (1) {;}
  #endif

  #if ENABLE_ROT_SENSOR_MONITOR
  rot_sensor_monitor();
  while (1) {;}
  #endif

  #if ENABLE_FIND_POLE_PAIR
  find_pole_pair();
  while (1) {;}
  #endif

  init_rot_sensor();

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  init_current_sensor();

  // general settings
  // motor phase resistance
  motor.phase_resistance = PHASE_RESISTANCE;

  init_motor_config();

  //SimpleFOCDebug::enable();
  //SimpleFOCDebug::enable(&DEBUG_SERIAL);
  command.verbose = VerboseMode::user_friendly;

  // use monitoring with serial
  motor.useMonitoring(MONITOR_SERIAL);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, (char*)"target angle");
  command.add('M', doCommand, (char*)"motor command");
  command.add('E', doEncoder, (char*)"encoder monitor");
  command.add('S', doEncoder, (char*)"simplefoc monitor");
  command.add('F', doFindZeroPos, (char*)"find zero pos");
  command.add('D', doDemo, (char*)"demo");
  command.add('R', doReset, (char*)"system reset");

  DEBUG_SERIAL.println(F("Motor ready."));
  DEBUG_SERIAL.println(F("Set the target angle using serial terminal:"));
  _delay(1000);

  preEncoderMonitorTimeMs_ = millis();
  
  enableFindZeroPosition_ = 1;
}

void loop()
{
  if (enableDemo_) {
    // Demo move
    if (enableDemo_ == 1) {
      demo_move();
    } else if (enableDemo_ == 2) {
      demo_move2();
    }
  } else if (enableFindZeroPosition_) {
    find_zero_position();
  } else {
    // iterative setting FOC phase voltage
    motor.loopFOC();

    // Motion control function
    motor.move();
  }

  if (enableSimpleFOCMonitor_) {
    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
    motor.monitor();
  }

  // user communication
  command.run();

  // Encoder monitor
  if (encoderMonitorIntervalMs_ >= 0) {
    encoder_monitor();
  }
}

/*********************************************************************/

void init_serial(void)
{
  Serial.begin(115200);
}

void init_wire(void)
{
  Wire.setSDA(PinName::PB_7);
  Wire.setSCL(PinName::PB_8);
  // @note Wire.begin() will be called in sensor.init()
}

void init_rot_sensor(void)
{
  rotSense.init(&Wire);
  // @note If run setClock() before sensor.init(), the system will hang.
  // STM32-DUINO only problem?
  Wire.setClock(400000);
  motor.linkSensor(&rotSense);
}

void init_current_sensor(void)
{
  currentSense.linkDriver(&driver);
  currentSense.init();
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);
}

/*********************************************************************/

void init_motor_config(void)
{
  extern BLDCMotor motor;
  
  // aligning voltage [V]
  motor.voltage_sensor_align = 1;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // TorqueControlType::voltage
  // TorqueControlType::dc_current
  // TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::voltage;

  // MotionControlType::torque
  // MotionControlType::velocity
  // MotionControlType::angle
  // MotionControlType::velocity_openloop
  // MotionControlType::angle_openloop
  motor.controller = MotionControlType::angle;

  motor.motion_downsample = DEF_MOTION_DOWNSMAPLE;

  // velocity loop PID
  motor.PID_velocity.P = 0.015;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000.0;
  //motor.PID_velocity.limit = 10.0;
  motor.LPF_velocity.Tf = 0.01;

  // angle loop PID
  motor.P_angle.P = 20.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  //motor.P_angle.output_ramp = 0.0;
  //motor.P_angle.limit = 1000.0;
  motor.LPF_angle.Tf = 0.001;
  
  // current q loop PID
  motor.PID_current_q.P = 5.0;
  motor.PID_current_q.I = 10.0;
  motor.PID_current_q.D = 1.0;
  //motor.PID_current_q.output_ramp = 0.0;
  //motor.PID_current_q.limit = 0.0;
  //motor.LPF_current_q.Tf = 0.0;
  
  // current d loop PID
  motor.PID_current_d.P = 20.0;
  motor.PID_current_d.I = 10.0;
  motor.PID_current_d.D = 0.0;
  //motor.PID_current_d.output_ramp = 0.0;
  //motor.PID_current_d.limit = 0.0;
  //motor.LPF_current_d.Tf = 0.0;
  
  // Limits 
  motor.current_limit = 6.0;
  motor.voltage_limit = 5.0;
  motor.velocity_limit = 200.0;
  
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
  
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 5.15;
}

void do_system_reset(void)
{
  HAL_NVIC_SystemReset();
}

/*********************************************************************/

#if ENABLE_FIND_POLE_PAIR
void find_pole_pair(void)
{
  init_rot_sensor();

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.init();

  DEBUG_SERIAL.begin(115200);

  // pole pairs calculation routine
  DEBUG_SERIAL.println("Pole pairs (PP) estimator");
  DEBUG_SERIAL.println("-\n");
  _delay(1000);
  DEBUG_SERIAL.println("-5\n");
  _delay(1000);
  DEBUG_SERIAL.println("-4\n");
  _delay(1000);
  DEBUG_SERIAL.println("-3\n");
  _delay(1000);
  DEBUG_SERIAL.println("-2\n");
  _delay(1000);
  DEBUG_SERIAL.println("-1\n");
  _delay(1000);

  float pp_search_voltage = POLE_PAIRE_SEARCH_VOLTAGE; // maximum power_supply_voltage/2
  float pp_search_angle = 1*M_PI; // search electrical angle to turn

  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = pp_search_voltage;
  
  motor.move(0);
  _delay(1000);

  // read the encoder angle
  rotSense.update(); 
  float angle_begin = rotSense.getAngle();
  _delay(50);

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.001f;
    motor.move(motor_angle);
    _delay(1);
  }
  _delay(1000);
  
  // read the encoder value for 180
  rotSense.update(); 
  float angle_end = rotSense.getAngle();
  _delay(50);

  // turn off the motor
  motor.move(0);
  _delay(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  DEBUG_SERIAL.print(F("Estimated PP : "));
  DEBUG_SERIAL.println(pp);
  DEBUG_SERIAL.println(F("PP = Electrical angle / Encoder angle "));
  DEBUG_SERIAL.print(pp_search_angle*180/M_PI);
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print((angle_end-angle_begin)*180/M_PI);
  DEBUG_SERIAL.print(" = ");
  DEBUG_SERIAL.println((pp_search_angle)/(angle_end-angle_begin));
  DEBUG_SERIAL.println();

  // a bit of monitoring the result
  if(pp <= 0 ){
    DEBUG_SERIAL.println(F("PP number cannot be negative"));
    DEBUG_SERIAL.println(F(" - Try changing the search_voltage value or motor/encoder configuration."));
    return;
  }else if(pp > 30){
    DEBUG_SERIAL.println(F("PP number very high, possible error."));
  }else{
    DEBUG_SERIAL.println(F("If PP is estimated well your motor should turn now!"));
    DEBUG_SERIAL.println(F(" - If it is not moving try to relaunch the program!"));
    DEBUG_SERIAL.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }
}
#endif

#if ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER
void rot_sensor_pulse_len_checker(void)
{
  while (1) {
    DEBUG_SERIAL.println("Not supported configuration");
    delay(1000);
  }
}
#endif

#if ENABLE_ROT_SENSOR_MONITOR
void rot_sensor_monitor(void)
{
  init_rot_sensor();

  while(1) {
    rotSense.update();

    DEBUG_SERIAL.print("angle:");
    DEBUG_SERIAL.print(rotSense.getAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("sensor angle:");
    DEBUG_SERIAL.print(rotSense.getSensorAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("mech angle:");
    DEBUG_SERIAL.print(rotSense.getMechanicalAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("prec angle:");
    DEBUG_SERIAL.print(rotSense.getPreciseAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("full rot:");
    DEBUG_SERIAL.print(rotSense.getFullRotations());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("vel:");
    DEBUG_SERIAL.println(rotSense.getVelocity());
  }
}
#endif

/*********************************************************************/

void encoder_monitor(void)
{
  int curEncoderMonitorTimeMs = millis();
  if (curEncoderMonitorTimeMs - preEncoderMonitorTimeMs_ >= encoderMonitorIntervalMs_) {
    preEncoderMonitorTimeMs_ = curEncoderMonitorTimeMs;
    COMMANDER_SERIAL.print("E ");
    COMMANDER_SERIAL.print(rotSense.getAngle());
    COMMANDER_SERIAL.print(" ");
    COMMANDER_SERIAL.print(rotSense.getVelocity());
    #if 0
    COMMANDER_SERIAL.print(" ");
    COMMANDER_SERIAL.print(100);
    COMMANDER_SERIAL.print(" ");
    COMMANDER_SERIAL.print(-100);
    #endif
    COMMANDER_SERIAL.println();
  }
}

/*********************************************************************/

void find_zero_position(void)
{
  const float targetVel = 0.005f;
  const float settleThreshold = 0.1f;

  static float target = 0.0f;
  static int state = 0;
  static float lastAngle = 0.0f;
  static IntervalTimer timer;

  switch(state) {
  case 0:
  {
    DEBUG_SERIAL.println(F("Start search zero position..."));
    timer.setIntervalMs(800);

    timer.reset();
    lastAngle = rotSense.getAngle();
    state++;
  } break;

  case 1:
  {
    if (timer.check()) {
      float angle = rotSense.getAngle();
      float delta = fabsf(angle - lastAngle);
      lastAngle = angle;
      if (delta < settleThreshold) {
        DEBUG_SERIAL.print(F("max angle = "));
        DEBUG_SERIAL.print(angle);
        DEBUG_SERIAL.println();
        offsetMaxAngle_ = angle;
        state++;
      }
    }

    motor.loopFOC();
    motor.move(target);
    target += targetVel;
  } break;

  case 2:
  {
    timer.reset();
    lastAngle = rotSense.getAngle();
    target = offsetMaxAngle_;
    state++;
  } break;
  
  case 3:
  {
    if (timer.check()) {
      float angle = rotSense.getAngle();
      float delta = fabsf(angle - lastAngle);
      lastAngle = angle;
      if (delta < settleThreshold) {
        DEBUG_SERIAL.print(F("zero angle = "));
        DEBUG_SERIAL.print(angle);
        DEBUG_SERIAL.println();
        offsetZeroAngle_ = angle;
        state++;
      }
    }

    motor.loopFOC();
    motor.move(target);
    target -= targetVel;
  } break;

  case 4:
  {
    DEBUG_SERIAL.println(F("Finish search zero position."));
    motor.move(offsetZeroAngle_);
    enableFindZeroPosition_ = 0;
    enableDemo_ = 1;
    state = 0;
  } break;

  default:
  {
    state = 0;
  } break;
  }
}

/*********************************************************************/

void demo_move(void)
{
  static float offset = offsetZeroAngle_ + ((offsetMaxAngle_ - offsetZeroAngle_) / 2);
  static float target = 0.0;
  static int state = 0;
  static int state2 = 0;
  static int pretime = 0;

  int curtime = millis();

  switch(state) {
  case 0:
    pretime = millis();
    state++;
    break;
  case 1:
    switch (state2)
    {
    case  0: target =  ((_PI / 2.0) * 2); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  1: target =  ((_PI / 2.0) * 4); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  2: target =  ((_PI / 2.0) * 6); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  3: target =  ((_PI / 2.0) * 8); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  4: target =  ((_PI / 2.0) * 6); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  5: target =  ((_PI / 2.0) * 4); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  6: target =  ((_PI / 2.0) * 2); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  7: target =  ((_PI / 2.0) * 0); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  8: target = -((_PI / 2.0) * 2); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case  9: target = -((_PI / 2.0) * 4); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 10: target = -((_PI / 2.0) * 6); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 11: target = -((_PI / 2.0) * 8); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 12: target = -((_PI / 2.0) * 6); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 13: target = -((_PI / 2.0) * 4); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 14: target = -((_PI / 2.0) * 2); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 15: target = -((_PI / 2.0) * 0); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 16: state2 = 0; state++; break;
    default: state++;
    }
    break;
  case 2:
    switch (state2)
    {
    case  0: target =  ((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  1: target = -((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  2: target =  ((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  3: target = -((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  4: target =  ((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  5: target = -((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  6: target =  ((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  7: target = -((_PI / 2.0) * 8); if (curtime - pretime >= 800) { pretime = curtime; state2++; } break;
    case  8: state2 = 0; state++; break;
    default: state++;
    }
    break;
  case 3:
  {
    static float pow = 0.08;
    static float dir = pow;
    static int pretime2;
    int curtime = millis();
    switch (state2)
    {
    case  0: {
      pretime  = curtime;
      pretime2 = curtime;
      state2++;
    } break;
    case  1: {
      if (curtime - pretime > 10) {
        target += dir;
        if (target > ((_PI / 2.0) * 8)) {
          dir = -pow;
        } else if (target < -((_PI / 2.0) * 8)){
          dir = pow;
        }
        pretime = curtime;
      }
      if (curtime - pretime2 > 10000) {
        state2++;
      }
    } break;
    case  2: state2 = 0; state++; break;
    default: state++;
    }
  } break;

  default:
    state = 0;
    break;
  }
  
  motor.loopFOC();
  motor.move(target + offset);
}

void demo_move2(void)
{
  static float offset = offsetZeroAngle_ + ((offsetMaxAngle_ - offsetZeroAngle_) / 2);
  static float target = 0.0;
  static float pow = 0.08;
  static float dir = pow;
  static int pretime = 0;

  int curtime = millis();

  if (curtime - pretime > 10) {
    target += dir;

    if (target > ((_PI / 2.0) * 8)) {
      dir = -pow;
    } else if (target < -((_PI / 2.0) * 8)){
      dir = pow;
    }

    pretime = curtime;
  }
  
  motor.loopFOC();
  motor.move(target + offset);
}
