/**********************************************************************/
/**
 * @brief  Hall Sensor to I2C Controller
 * @author naoa
 */
/**********************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Includes
 *----------------------------------------------------------------------
 */
#include <math.h>
#include <Wire.h>

#include "led.hpp"
#include "interval_timer.hpp"

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

#define PIN_I2C_SDA       20
#define PIN_I2C_SCL       21
#define PIN_HAL_A         26
#define PIN_HAL_B         27
#define PIN_HAL_C         28

#define ENABLE_SETUP_SERIAL_HOST_WAIT   (1)
#define SETUP_SERIAL_HOST_WAIT_MS       (1000) // startup wait after serial begin for host pc connection
#define USB_SERIAL                      Serial

#define ENABLE_IIR_FILTER         (1) 
#define ALPHA_IIR                 (0.1)   // Alpha of IIR filter
#define ENABLE_MOVING_AVE_FILTER  (1)
#define MA_LEN                    (2)     // Samples of moving average

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

static void i2c_on_write(int len); 
static void i2c_on_read(void); 

static double iir_filter(double raw, double prevFiltered, double alpha);
static double moving_average_filter(double raw, double *buffer, int *index);
static double calc_position(double A, double B, double C);
static void   main_process(void);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

static LED onBoardLed_(25, 500);
static IntervalTimer debugTimer_;

static uint16_t angle_ = 0;

static double iirA_ = 0, iirB_ = 0, iirC_ = 0;
static double maBufferA_[MA_LEN] = {0}, maBufferB_[MA_LEN] = {0}, maBufferC_[MA_LEN] = {0};
static int idxA_ = 0, idxB_ = 0, idxC_ = 0;

static double anglePrev_ = 0;
static int turnCount_ = 0;

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Core0
 *----------------------------------------------------------------------
 */

void setup()
{
  //////////////////////////
  // Setup SysClock

  //set_sys_clock_khz(144000, true);

  //////////////////////////
  // Setup debug serial
  USB_SERIAL.begin(115200);
  #if ENABLE_SETUP_SERIAL_HOST_WAIT
  delay(SETUP_SERIAL_HOST_WAIT_MS);
  #endif
  USB_SERIAL.printf("START SETUP\n");
  
  //////////////////////////
  // Setup modules
  
  debugTimer_.setIntervalMs(1000);

  pinMode(PIN_HAL_A, INPUT);
  pinMode(PIN_HAL_B, INPUT);
  pinMode(PIN_HAL_C, INPUT);

  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin(0x40);

  Wire.onReceive(i2c_on_write);
  Wire.onRequest(i2c_on_read);
}

void loop()
{
  main_process();

  //delay(10);
  
  onBoardLed_.loop();

#if 0
  static int fps = 0;
  fps++;
  if (debugTimer_.check()) {
    USB_SERIAL.printf("fps = %d\n", fps);
    fps = 0;
  }
#endif
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Core1
 *----------------------------------------------------------------------
 */

void setup1()
{
#if ENABLE_SETUP_SERIAL_HOST_WAIT
  delay(SETUP_SERIAL_HOST_WAIT_MS);
#endif
}

void loop1()
{

}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions
 *----------------------------------------------------------------------
 */

static double iir_filter(double raw, double prevFiltered, double alpha)
{
  return alpha * raw + (1.0 - alpha) * prevFiltered;
}

static double moving_average_filter(double raw, double *buffer, int *index)
{
  buffer[*index] = raw;
  *index = (*index + 1) % MA_LEN;
  double sum = 0;
  for(int i = 0; i < MA_LEN; i++) sum += buffer[i];
  return sum / MA_LEN;
}

static double calc_position(double A, double B, double C)
{
  // Clarke conversion

  double offset = (A + B + C) / 3.0;
  double A2 = A - offset;
  double B2 = B - offset;
  double C2 = C - offset;

  double alpha = A2;
  double beta  = (B2 - C2) / 1.73205080757; // (B2 - C2) / sqrt(3.0)

  double angle = atan2(beta, alpha);

  if(angle < 0) angle += 2*M_PI;
  angle = angle * 180.0 / M_PI;

  return angle;
}

static void main_process(void)
{
  int rawA = analogRead(PIN_HAL_A);
  int rawB = analogRead(PIN_HAL_B);
  int rawC = analogRead(PIN_HAL_C);

  #if ENABLE_IIR_FILTER
  iirA_ = iir_filter(rawA, iirA_, ALPHA_IIR);
  iirB_ = iir_filter(rawB, iirB_, ALPHA_IIR);
  iirC_ = iir_filter(rawC, iirC_, ALPHA_IIR);
  #else
  iirA_ = rawA;
  iirB_ = rawB;
  iirC_ = rawC;
  #endif

  #if ENABLE_MOVING_AVE_FILTER
  double filteredA = moving_average_filter(iirA_, maBufferA_, &idxA_);
  double filteredB = moving_average_filter(iirB_, maBufferB_, &idxB_);
  double filteredC = moving_average_filter(iirC_, maBufferC_, &idxC_);
  #else
  double filteredA = iirA_;
  double filteredB = iirB_;
  double filteredC = iirC_;
  #endif

  // Calcurate raw angle from linear hall sensors value.
  double angle_raw = calc_position(filteredA, filteredB, filteredC);

  // Convert normalized angle to 0–16384 integer: (angle_raw / 360) * 16384
  angle_ = (uint16_t)(angle_raw * 45.511111) & 0x3FFF;
  //USB_SERIAL.printf("0, 16384, %f, %f, %f, %.2f, %d\n", filteredA, filteredB, filteredC, angle_raw, angle_);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions
 *----------------------------------------------------------------------
 */

static void i2c_on_write(int len)
{
  //Wire1.read()
  //0xFE / 0xFF = angle register
}

static void i2c_on_read(void)
{
  uint8_t buff[2];
  uint16_t data = angle_;
  buff[0] = (data >> 6) & 0xFF;
  buff[1] = (data >> 0) & 0x3F;
  Wire.write(buff, 2);
}
