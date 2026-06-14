#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define LOOP_RATE_HZ    1000
#define MPU_RATE        1000
#define COMPASS_RATE    100
#define BARO_RATE_HZ    50
#define HC12_COMM_RATE  20
#define BARO_POLL_US    18000

// --- ODR Setting Constants ---
#define BMP3_ODR_50_HZ     0x02  // 20ms (Balanced Drone Setting)
#define BMP3_ODR_25_HZ     0x03  // 40ms 
#define BMP3_ODR_12_5_HZ   0x04  // 80ms (Max Accuracy)

#define MPU_INTERVAL_US     (1000000UL / MPU_RATE)
#define COMPASS_INTERVAL_US (1000000UL / COMPASS_RATE)
#define HC12_INTERVAL_US    (1000000UL / HC12_COMM_RATE)

// EEPROM
#define EEPROM_CALIB_ADDR 0
#define CALIB_SIGNATURE 0x7A

// Motor pins (DShot600)
#define MOTOR1 9
#define MOTOR2 10
#define MOTOR3 11
#define MOTOR4 12
#define CPU_MHZ 600
#define DSHOT600_BIT_US 1.67f

// Peripherals 
#define SERVO_PIN 15

#define HC12_SERIAL Serial1 // (Hardware Serial 1: Pins 0 and 1)
#define HC12_BAUD 9600
#define HC12_SET_PIN 2

#define VBAT_PIN 27 // A13
const float DIVIDER_RATIO = 10000.0 / (50000.0 + 10000.0);

// Failsafe
#define FAILSAFE_TIMEOUT_MS 1000
#define RADIO_SIGNAL_TIMEOUT_MS 500

// Timing 
uint32_t lastMPUTime = 0, lastBaroTime = 0, lastCompassTime = 0;
uint32_t lastHC12Time = 0, lastPrintTime = 0, prevTime = 0;

const uint8_t MPU_ADDR = 0x68; // I2C address for MPU6050
const uint8_t MAG_ADDR = 0x2C; // I2C address for the hidden QMC5883P!

float rollPIDOutput = 0.0; 
float pitchPIDOutput = 0.0; 
float yawPIDOutput = 0.0; 
float altPIDOutput = 0.0; 

float throttle_in = 0.0;
float roll_setpoint = 0.0;
float pitch_setpoint = 0.0;
float yaw_rate_setpoint = 0.0;
float yaw_angle_setpoint = 0.0; 
float vert_vel_setpoint = 0.0; 
float alt_setpoint = 0.0; 

// Madgwick Instances
float B_madgwick = 0.041; // Madgwick filter parameter
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float rollMADG = 0.0, pitchMADG = 0.0, yawMADG = 0.0, yawKalman = 0.0;

float accZ_earth_ms2 = 0.0; 
float accEarthAlpha = 0.5;

float finalVertVel = 0.0; 
float finalAlt = 0.0;

float initialAlt_setpoint = 1.0f; // Initial altitude setpoint in meters

float vel_x_setpoint = 0.0; 
float vel_y_setpoint = 0.0;

float throttle_Int_db = 0.20f; // Won't fly till like 40 - 50 % throttle

// Motor outputs (0-2047) for DShot
uint16_t motor_output[4] = {0, 0, 0, 0}; 

enum FlightMode {
  MODE_DISARMED = 0, 
  MODE_MANUAL = 1, 
  MODE_STABILIZE = 2, // Altitude hold
  MODE_AUTO = 3, 
  MODE_FAILSAFE = 4
}; 

// Flight state
FlightMode flight_mode = MODE_DISARMED; 
bool armed = false;
bool payload_released = false; 
uint32_t arm_time = 0; 

// Magnetometer calibration 
// Hard Iron Bias (b)
float mag_bias[3] = { -4.563036637e+01, 1.722627070e+02, 2.417504545e+02 };
// Soft Iron Matrix (A)
float mag_scale[3][3] = {
  { 2.143687975e-03, -4.932252201e-05, -1.333228547e-05 },
  { -4.932252201e-05, 2.168749356e-03, -9.061763534e-07 },
  { -1.333228547e-05, -9.061763534e-07, 2.274640713e-03 }
};

struct AsyncDFBMP388 {
  // Public variables 
  bool newDataAvailable = false;
  float pressure_pa = 0.0f;
  float temp_c = 0.0f;
  
  // Easily editable polling interval from the main loop
  uint32_t pollIntervalUs = 80000; 

  const uint8_t BMP_ADDR = 0x76; 
  uint32_t lastPollTime = 0;

  float parT1, parT2, parT3;
  float parP1, parP2, parP3, parP4, parP5, parP6, parP7, parP8, parP9, parP10, parP11;
  float tempLin; 

  // --- NEW SETUP LOGIC ---
  // Defaults to 50Hz and an 18,000 microsecond polling interval
  void init(uint8_t odr_setting = 0x02, uint32_t refresh_rate_us = 18000) {
    Wire2.setSCL(24);
    Wire2.setSDA(25);
    Wire2.begin();
    Wire2.setClock(400000); 

    Serial.println("Initializing BMP388 in BALANCED DRONE Mode...");
    
    if (readReg(0x00) != 0x50) {
      Serial.println("ERROR: BMP388 not found! Check wiring.");
      while (1);
    }

    readAndQuantizeCalibration();

    pollIntervalUs = refresh_rate_us;

    // --- HARDWARE CONFIGURATION FOR DRONE BALANCE ---
    
    // OSR (0x1C): Press x8 (0x03), Temp x1 (0x00) 
    writeReg(0x1C, 0x03); 
    delay(5);
    
    // ODR (0x1D): Use the variable passed from the main loop
    writeReg(0x1D, odr_setting); 
    delay(5);
    
    // IIR (0x1F): Coef 3 (Good balance of prop-wash rejection and speed)
    writeReg(0x1F, 0x04); 
    delay(5);
    
    // PWR_CTRL (0x1B): Press ON, Temp ON, Normal Mode
    writeReg(0x1B, 0x33); 
    delay(50); // Allow sensor time to enter Normal Mode

    Serial.println("BMP388 Configured Successfully.");
  }

  void update(uint32_t now_us) {
    // Dynamic polling based on the variable
    if (now_us - lastPollTime >= pollIntervalUs) {
      lastPollTime = now_us;

      uint8_t status = readReg(0x03);

      if (status & 0x20) {
        Wire2.beginTransmission(BMP_ADDR);
        Wire2.write(0x04);
        
        // CRITICAL FIX: Use 'false' for a Repeated Start to prevent bus release
        if (Wire2.endTransmission(false) == 0) { 
          
          Wire2.requestFrom(BMP_ADDR, (uint8_t)6);
          if (Wire2.available() == 6) {
            
            uint8_t buf[6];
            for (int i = 0; i < 6; i++) { buf[i] = Wire2.read(); }

            uint32_t uncompPress = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
            uint32_t uncompTemp  = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

            temp_c = calibTemperatureC(uncompTemp);
            pressure_pa = calibPressurePa(uncompPress);
            
            newDataAvailable = true;
          }
        }
      }
    }
  }

  float getPressure() { return pressure_pa; }
  float getTemperature() { return temp_c; }

private:
  uint8_t readReg(uint8_t reg) {
    Wire2.beginTransmission(BMP_ADDR);
    Wire2.write(reg);
    Wire2.endTransmission(); 
    Wire2.requestFrom(BMP_ADDR, (uint8_t)1);
    return Wire2.available() ? Wire2.read() : 0;
  }

  void writeReg(uint8_t reg, uint8_t value) {
    Wire2.beginTransmission(BMP_ADDR);
    Wire2.write(reg);
    Wire2.write(value);
    Wire2.endTransmission();
  }

  void readAndQuantizeCalibration() {
    uint8_t cal[21];
    Wire2.beginTransmission(BMP_ADDR);
    Wire2.write(0x31);
    Wire2.endTransmission(); 
    Wire2.requestFrom(BMP_ADDR, (uint8_t)21);
    for (int i = 0; i < 21; i++) { cal[i] = Wire2.read(); }

    uint16_t reg_T1 = ((uint16_t)cal[1] << 8) | (uint16_t)cal[0];
    uint16_t reg_T2 = ((uint16_t)cal[3] << 8) | (uint16_t)cal[2];
    int8_t   reg_T3 = (int8_t)cal[4];

    int16_t  reg_P1 = (int16_t)(((uint16_t)cal[6] << 8) | (uint16_t)cal[5]);
    int16_t  reg_P2 = (int16_t)(((uint16_t)cal[8] << 8) | (uint16_t)cal[7]);
    int8_t   reg_P3 = (int8_t)cal[9];
    int8_t   reg_P4 = (int8_t)cal[10];
    uint16_t reg_P5 = ((uint16_t)cal[12] << 8) | (uint16_t)cal[11];
    uint16_t reg_P6 = ((uint16_t)cal[14] << 8) | (uint16_t)cal[13];
    int8_t   reg_P7 = (int8_t)cal[15];
    int8_t   reg_P8 = (int8_t)cal[16];
    int16_t  reg_P9 = (int16_t)(((uint16_t)cal[18] << 8) | (uint16_t)cal[17]);
    int8_t   reg_P10 = (int8_t)cal[19];
    int8_t   reg_P11 = (int8_t)cal[20];

    parT1 = (float)reg_T1 / pow(2, -8);
    parT2 = (float)reg_T2 / pow(2, 30);
    parT3 = (float)reg_T3 / pow(2, 48);

    parP1 = (float)(reg_P1 - 16384) / pow(2, 20);
    parP2 = (float)(reg_P2 - 16384) / pow(2, 29);
    parP3 = (float)reg_P3 / pow(2, 32);
    parP4 = (float)reg_P4 / pow(2, 37);

    parP5 = (float)reg_P5 / pow(2, -3);
    parP6 = (float)reg_P6 / pow(2, 6);
    parP7 = (float)reg_P7 / pow(2, 8);
    parP8 = (float)reg_P8 / pow(2, 15);
    parP9 = (float)reg_P9 / pow(2, 48);
    parP10 = (float)reg_P10 / pow(2, 48);
    parP11 = (float)reg_P11 / pow(2, 65);
  }

  float calibTemperatureC(uint32_t uncompTemp) {
    float partialData1 = (float)(uncompTemp - parT1);
    float partialData2 = partialData1 * parT2;
    tempLin = partialData2 + pow(partialData1, 2) * parT3;
    return tempLin;
  }

  float calibPressurePa(uint32_t uncompPress) {
    float partialData1, partialData2, partialData3, partialData4;
    float partialOut1, partialOut2, compPress;

    partialData1 = parP6 * tempLin;
    partialData2 = parP7 * pow(tempLin, 2);
    partialData3 = parP8 * pow(tempLin, 3);
    partialOut1 = parP5 + partialData1 + partialData2 + partialData3;

    partialData1 = parP2 * tempLin;
    partialData2 = parP3 * pow(tempLin, 2);
    partialData3 = parP4 * pow(tempLin, 3);
    partialOut2 = (float)uncompPress * (parP1 + partialData1 + partialData2 + partialData3);

    partialData1 = (float)uncompPress * (float)uncompPress;
    partialData2 = parP9 + parP10 * tempLin;
    partialData3 = partialData1 * partialData2;
    partialData4 = partialData3 + pow((float)uncompPress, 3) * parP11;
    
    compPress = partialOut1 + partialOut2 + partialData4;
    return compPress;
  }
};

// Hardware instances
AsyncDFBMP388 Barometer;
Servo payloadServo; 

// Structs 
struct IMUData {
  float acc[3]; 
  float gyro[3]; 
  float mag[3]; 
  float baroP;
  float baroT; 
  float baroH; 
  float mpuT;
  float accBias[3]; 
  float gyroBias[3]; 
} rawImuData; 

struct CalibrationStore {
  uint8_t signature;
  float accBias[3];
  float gyroBias[3];
} savedCalib;

struct FilteredIMUData {
  float acc_filt[3];
  float gyro_filt[3];
  float mag_filt[3];
  float baroP_filt; 
  float baroH_filt; 
  float acc_alpha; 
  float gyro_alpha;
  float mag_alpha;
  float baro_alpha; 
} filteredImuData;

struct RadioData {
  uint16_t roll;      // Ch1
  uint16_t pitch;     // Ch2
  uint16_t throttle;  // Ch3
  uint16_t yaw;       // Ch4
  uint16_t arm_cmd;   // Ch5
  uint16_t mode;      // Ch6
  uint32_t last_update;
  bool signal_valid;

  // Radio utils 
  uint8_t ibusIndex;
  uint8_t ibusBuffer[32];

  void init(){
    ibusIndex = 0; 
  }

  void readIBUS() {
    while (Serial7.available() > 0) {
      uint8_t val = Serial7.read();
      if (ibusIndex == 0 && val != 0x20) continue; // Keep looking for 0x20
      if (ibusIndex == 1 && val != 0x40) { 
        ibusIndex = 0; // False alarm, reset
        continue; 
      }
      ibusBuffer[ibusIndex++] = val;
      if (ibusIndex == 32) {
        ibusIndex = 0; // Reset for the next packet
        uint16_t calculated_checksum = 0xFFFF;
        for (int i = 0; i < 30; i++) {
          calculated_checksum -= ibusBuffer[i];
        }
        // Read the checksum sent by the receiver
        uint16_t received_checksum = ibusBuffer[30] | (ibusBuffer[31] << 8);

        if (calculated_checksum == received_checksum) {
          roll     = ibusBuffer[2] | (ibusBuffer[3] << 8);
          pitch    = ibusBuffer[4] | (ibusBuffer[5] << 8);
          throttle = ibusBuffer[6] | (ibusBuffer[7] << 8);
          yaw      = ibusBuffer[8] | (ibusBuffer[9] << 8);
          arm_cmd  = ibusBuffer[10] | (ibusBuffer[11] << 8); 
          mode     = ibusBuffer[12] | (ibusBuffer[13] << 8); 
          // Update failsafe tracking
          last_update = millis();
          signal_valid = true;
        }
      }
    }
  }
} radio;

struct PIDController {
  float Kp_rate;
  float Ki_rate;
  float Kd_rate;
  float Kp_pos;

  float prevErrorRate;
  float integralRate;
  float derivativeRate;
  float LPF_beta;
  float MAX_I_OUTPUT; 
  float output_multiplier;

  void reset(){
    prevErrorRate = 0; 
    integralRate = 0;
    derivativeRate = 0; 
  } 

  float update(float dt, float setpoint, float pos_meas, float rate_meas){
    float error_pos = setpoint - pos_meas; 
    float target_rate = Kp_pos * error_pos; 

    float error_rate = target_rate - rate_meas; 
    float P_out = Kp_rate * error_rate; 

    integralRate += error_rate * dt;
    float I_out = 0.0f; 
    if (Ki_rate > 0.0001f && throttle_in > throttle_Int_db){
      float current_acc_limit = MAX_I_OUTPUT / Ki_rate;
      integralRate = constrain(integralRate, -current_acc_limit, current_acc_limit); 
      I_out = Ki_rate * integralRate;
    } else {
      integralRate = 0.0f;
    }

    derivativeRate = derivativeRate * LPF_beta + (1 - LPF_beta) * (error_rate - prevErrorRate) / dt;
    float D_out = Kd_rate * derivativeRate; 
    prevErrorRate = error_rate; 

    float pid_output = (P_out + I_out + D_out) * output_multiplier; 
    return pid_output; 
  }

  float updateRate(float dt, float rate_setpoint, float rate_meas){
    float error_rate = rate_setpoint - rate_meas; 
    float P_out = Kp_rate * error_rate; 

    integralRate += error_rate * dt;
    float I_out = 0.0f; 
    if (Ki_rate > 0.0001f && throttle_in > throttle_Int_db){
      float current_acc_limit = MAX_I_OUTPUT / Ki_rate;
      integralRate = constrain(integralRate, -current_acc_limit, current_acc_limit); 
      I_out = Ki_rate * integralRate;
    } else {
      integralRate = 0.0f;
    }

    derivativeRate = derivativeRate * LPF_beta + (1 - LPF_beta) * (error_rate - prevErrorRate) / dt;
    float D_out = Kd_rate * derivativeRate; 
    prevErrorRate = error_rate; 

    float pid_output = (P_out + I_out + D_out) * output_multiplier; 
    return pid_output; 
  }
} pidRoll, pidPitch, pidYaw; 

struct StandardPID {
  float Kp_pos;
  float Ki_pos;
  float Kd_pos;
  float LPF_beta;
  float MAX_I_OUTPUT; 

  float prevErrorPos;
  float integralPos;
  float derivativePos;

  void reset(){
    prevErrorPos = 0; 
    integralPos = 0;
    derivativePos = 0; 
  }

  void init(float Kp, float Ki, float Kd, float lpf_beta, float max_i_output){
    Kp_pos = Kp;
    Ki_pos = Ki;
    Kd_pos = Kd;
    LPF_beta = lpf_beta;
    MAX_I_OUTPUT = max_i_output;
  }

  float update(float dt, float pos_setpoint, float meas_pos){
    float error_pos = pos_setpoint - meas_pos;
    float P_out = Kp_pos * error_pos; 

    integralPos += error_pos * dt; 
    float I_out = 0.0f; 

    if (Ki_pos > 0.0001f && throttle_in > throttle_Int_db){
      float current_acc_limit = MAX_I_OUTPUT / Ki_pos;
      integralPos = constrain(integralPos, -current_acc_limit, current_acc_limit); 
      I_out = Ki_pos * integralPos;
    } else {
      integralPos = 0.0f;
    }

    derivativePos = derivativePos * LPF_beta + (1 - LPF_beta) * (error_pos - prevErrorPos) / dt;
    float D_out = Kd_pos * derivativePos;
    prevErrorPos = error_pos;

    float pid_output = P_out + I_out + D_out;
    return pid_output;
  }
} pidAlt; 

struct AltitudeLuenberger{
  float home_altitude; 
  float alt_est;
  float vel_est; 
  float acc_trim; 

  // Tuning gains 
  float Kp_alt; float Kp_vel; float Ki_acc;

  float last_baro_alt;

    
  void calibrateBaroHome() {
    float alt_sum = 0.0f; 
    int valid_samples = 0; 

    while (valid_samples < 50) {
      uint32_t now = micros();
      Barometer.update(now);
      if (Barometer.newDataAvailable) {
        float p = Barometer.getPressure(); 
        float h = 44330.0f * (1.0f - pow(p / 101325.0f, 0.1903f));
        alt_sum += h;
        valid_samples++;
        // if (valid_samples % 10 == 0) Serial.print("."); 
        Barometer.newDataAvailable = false;
      }
      delay(1); 
    }

    home_altitude = alt_sum / 50.0f; 
    filteredImuData.baroH_filt = home_altitude; 
  }

  void init(){
    alt_est = home_altitude;
    last_baro_alt = home_altitude;
    vel_est = 0.0f;
    acc_trim = 0.0f;

    // Tuned gains optimized for Pos and Vel
    Kp_alt = 1.064f;
    Kp_vel = 0.746f;
    Ki_acc = 0.119f;
  }

  void update(float dt, float accZ_earth, float baro_meas, bool new_baro_data) {
    if (new_baro_data){
      last_baro_alt = baro_meas;
    }
    float error = last_baro_alt - alt_est;
    acc_trim += Ki_acc * error * dt; 

    float acc_clean = accZ_earth + acc_trim; 

    alt_est += (vel_est + Kp_alt * error) * dt; 
    vel_est += (acc_clean + Kp_vel * error) * dt; 
  }
} altitude_luenberger; 

struct AttitudeKalman {
  float angle; // The estimated angle
  float bias;  // The estimated gyro bias
  // Covariance Matrix
  float P[2][2];
  // Tuning Parameters
  float Q_angle;   // Process noise variance for the accelerometer
  float Q_bias;    // Process noise variance for the gyro bias
  float R_measure; // Measurement noise variance
  float yaw_alpha;  

  void init() {
    angle = 0.0f;
    bias = 0.0f;
    P[0][0] = 3.0f*3.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 3.0f*3.0f;
    // Tuning values for a 2000Hz loop
    Q_angle = 4.0f*4.0f;
    Q_bias = 4.0f;
    R_measure = 3.0f*3.0f; 
    yaw_alpha = 0.05;
  }
  
  float update(float dt, float gyro_rate, float measured_angle) {
    float rate = gyro_rate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Calculate difference
    float y = measured_angle - angle;

    // --- CRITICAL YAW WRAP-AROUND FIX ---
    while (y > 180.0f)  y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    // ------------------------------------

    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    angle += K0 * y;
    bias += K1 * y;

    // Keep the internal Kalman angle constrained to -180 to 180
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
  }
} kalmanYaw;

struct AngleKalman1D{
  float angle;
  float uncertaintyAngle;
  float Q_std;
  float R_std;

  void init(){
    angle = 0.0f;
    uncertaintyAngle = 4.0f; // High initial uncertainty
    Q_std = 4.0f; // Process noise standard deviation
    R_std = 3.0f; // Measurement noise standard deviation
  }

  void update(float dt, float input, float measurement){
    angle += input * dt;
    uncertaintyAngle += dt * dt * Q_std * Q_std;
    float KalmanGain = uncertaintyAngle / (uncertaintyAngle + R_std * R_std);
    angle += KalmanGain * (measurement - angle);
    uncertaintyAngle *= (1 - KalmanGain);
  }
} kalmanRoll, kalmanPitch; 

struct PositionEstimator {
  float posX_est, posX_L;
  float posY_est, posY_L;
  float velX_est, velX_L;
  float velY_est, velY_L;
  float accX_earth; 
  float accY_earth;

  float Kp_pos;
  float Kp_vel;
  float decay_rate; 

  float angle_deadband; 
  float accEarthAlpha;

  void init(){
    posX_est = 0.0f; posY_est = 0.0f; posX_L = 0.0f; posY_L = 0.0f;
    velX_est = 0.0f; velY_est = 0.0f; velX_L = 0.0f; velY_L = 0.0f;
    accX_earth = 0.0f; accY_earth = 0.0f;

    angle_deadband = 1.0f;
    accEarthAlpha = 0.2f;

    Kp_pos = 1.0f;
    Kp_vel = 1.0f;
    decay_rate = 0.8f;
  }

  void update(float dt, float roll_deg, float pitch_deg, float yaw_deg, FilteredIMUData filteredImuData_){
    // Check for actual motion 
    if (abs(roll_deg) < angle_deadband) roll_deg = 0.0f;
    if (abs(pitch_deg) < angle_deadband) pitch_deg = 0.0f;

    // Grab all 3 axes (You need Z to properly rotate X and Y!)
    float accX_body = filteredImuData_.acc_filt[0];
    float accY_body = filteredImuData_.acc_filt[1];
    float accZ_body = filteredImuData_.acc_filt[2];

    // Convert angles to radians 
    float roll_rad = roll_deg * 0.0174532925f;
    float pitch_rad = pitch_deg * 0.0174532925f;
    float yaw_rad = yaw_deg * 0.0174532925f;

    // Pre-compute trig functions
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    // --- 1. ROTATE TO EARTH FRAME ---
    // Standard DCM Euler projection from Body to Earth
    float aX_earth_raw = (cp * cy) * accX_body 
                       + (sr * sp * cy - cr * sy) * accY_body 
                       + (cr * sp * cy + sr * sy) * accZ_body;

    float aY_earth_raw = (cp * sy) * accX_body 
                       + (sr * sp * sy + cr * cy) * accY_body 
                       + (cr * sp * sy - sr * cy) * accZ_body;

    // --- 2. GRAVITY IS ALREADY REMOVED ---
    // Note: Because we are rotating the entire 3D vector, the 1g gravity component
    // is mathematically isolated entirely into the Z-axis of the Earth frame. 
    // The X and Y axes are naturally gravity-free after this rotation!

    // --- 3. FILTER AND SCALE TO m/s^2 ---
    accX_earth = accEarthAlpha * accX_earth + (1.0f - accEarthAlpha) * (aX_earth_raw * 9.80665f);
    accY_earth = accEarthAlpha * accY_earth + (1.0f - accEarthAlpha) * (aY_earth_raw * 9.80665f);

    // --- 4. Estimate velocity and position ---
    // If there is no actual motion 
    if (abs(roll_deg) < angle_deadband && abs(pitch_deg) < angle_deadband){
      // Bring the velocity back to zero
      velX_est *= (1 - decay_rate); 
      velY_est *= (1 - decay_rate); 
    } else {
      // If there is motion, integrate acceleration to velocity and position
      velX_est += accX_earth * dt;
      velY_est += accY_earth * dt;
    }

    // Integrate velocity to get intermediate position
    posX_est += velX_est * dt;
    posY_est += velY_est * dt;

    // Now use Luenberger-style corrections based on the current position and velocity estimates
    float errorX = posX_est - posX_L; // measurement - estimate 
    float errorY = posY_est - posY_L;

    posX_L += (velX_est + Kp_pos * errorX) * dt;
    posY_L += (velY_est + Kp_pos * errorY) * dt;

    velX_L += (accX_earth + Kp_vel * errorX) * dt;
    velY_L += (accY_earth + Kp_vel * errorY) * dt;
  }
} positionEstimator; 

struct PositionHoldVel {
  float Kp_vel; 

  void init(){
    Kp_vel = 10.0f; 
  }

  float get_angle_setpoint(float vel_error){
    return Kp_vel * vel_error;
  }
} positionHoldVel;

// Functions

void loopRate(int freq){
  uint32_t period = 1000000 / freq; 
  while (micros() - prevTime < period) {
    // Wait
  }
}

void setupIMU() {
  // --- MPU6050 INITIALIZATION ---
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up
  byte status = Wire.endTransmission();

  if (status != 0){
    Serial.println("MPU6050 init failed!");
    while(1){ delay(10); } // Halt on failure
  }

  // ENABLE HARDWARE DLPF (Digital Low Pass Filter)
  // 0x00 = 260Hz, 0x01 = 184Hz, 0x02 = 94Hz, 0x03 = 44Hz, 0x04 = 21Hz
  // 0x03 (44Hz) or 0x04 (21Hz) are standard for quadcopters
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // Set DLPF to ~44Hz
  Wire.endTransmission();

  // Set Gyro to +/- 250 degrees/second (Max Resolution)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();
  // Set Accel to +/- 2g (Max Resolution)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();
  delay(10); 

  // --- I2C BYPASS SETUP ---
  // Disable MPU6050 Master Mode
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6A); Wire.write(0x00); Wire.endTransmission(); delay(10);
  // Open Bypass Gate
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission(); delay(50); 
  
  // --- QMC5883P CONFIGURATION ---
  // Control Register 1 (0x0A): 0x1D = Continuous Mode, 200Hz ODR, 8G Range
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x0A); Wire.write(0x1D); Wire.endTransmission(); delay(10);
  // Configuration Register (0x0B): Set/Reset Period = 0x01
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission(); delay(10);

  // --- BMP388 CONFIGURATION ---
  Barometer.init(BMP3_ODR_50_HZ, BARO_POLL_US);
}

void readRawIMU(uint32_t now, IMUData &rawImuData) {  
  // --- CRITICAL FIX: TICK THE BAROMETER STATE MACHINE ---
  // This MUST run every single loop to track the fast microsecond timers
  Barometer.update(now);

  // --- BMP388 BAROMETER FETCH ---
  if (Barometer.newDataAvailable) {
      float pressure = Barometer.getPressure();
      float altitude = 44330.0f * (1.0f - pow(pressure / 101325.0f, 0.1903f));

      rawImuData.baroP = pressure;
      rawImuData.baroH = altitude;

      // Barometer.newDataAvailable = false; // This is reset in the main loop after all calculations have been done. 
  }

  // --- QMC5883P COMPASS ---
  if (now - lastCompassTime >= COMPASS_INTERVAL_US){
    lastCompassTime += COMPASS_INTERVAL_US; 

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x01); // CRITICAL: QMC5883P data starts at 0x01
    Wire.endTransmission(); 
    
    Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)6);
    
    // Read Little-Endian (LSB first, MSB second)
    // int16_t mx = (Wire.read() | (Wire.read() << 8));
    // int16_t my = (Wire.read() | (Wire.read() << 8));
    // int16_t mz = (Wire.read() | (Wire.read() << 8));

    // Little-Endian (LSB first, MSB second)
    int16_t mx = Wire.read(); mx |= (Wire.read() << 8);
    int16_t my = Wire.read(); my |= (Wire.read() << 8);
    int16_t mz = Wire.read(); mz |= (Wire.read() << 8);

    // Leaving as raw integer values since you are applying calibration offsets manually
    rawImuData.mag[0] = (float)mx;
    rawImuData.mag[1] = (float)my;
    rawImuData.mag[2] = (float)mz; 
  }
  
  // --- MPU6050 ---
  if (now - lastMPUTime >= MPU_INTERVAL_US){
    lastMPUTime += MPU_INTERVAL_US;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false); 
    
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14); 
    
    // Read Big-Endian (MSB first, LSB second)
    // int16_t ax = (Wire.read() << 8 | Wire.read());
    // int16_t ay = (Wire.read() << 8 | Wire.read());
    // int16_t az = (Wire.read() << 8 | Wire.read());
    // int16_t temp = (Wire.read() << 8 | Wire.read()); 
    // int16_t gx = (Wire.read() << 8 | Wire.read());
    // int16_t gy = (Wire.read() << 8 | Wire.read());
    // int16_t gz = (Wire.read() << 8 | Wire.read());

    // Big-Endian (MSB first, LSB second)
    int16_t ax = Wire.read() << 8; ax |= Wire.read();
    int16_t ay = Wire.read() << 8; ay |= Wire.read();
    int16_t az = Wire.read() << 8; az |= Wire.read();
    int16_t temp = Wire.read() << 8; temp |= Wire.read(); 
    int16_t gx = Wire.read() << 8; gx |= Wire.read();
    int16_t gy = Wire.read() << 8; gy |= Wire.read();
    int16_t gz = Wire.read() << 8; gz |= Wire.read();

    // Scale to G's and Deg/s
    rawImuData.acc[0] = (float)ax / 16384.0; 
    rawImuData.acc[1] = (float)ay / 16384.0;
    rawImuData.acc[2] = (float)az / 16384.0;
    
    rawImuData.gyro[0] = (float)gx / 131.0; 
    rawImuData.gyro[1] = (float)gy / 131.0;
    rawImuData.gyro[2] = (float)gz / 131.0;

    rawImuData.mpuT = (float)temp;
  }
}

void filterIMUData(){
  for (int i = 0; i < 3; i++){
    // LPF Acc
    float curr_acc = rawImuData.acc[i] - rawImuData.accBias[i]; 
    filteredImuData.acc_filt[i] = filteredImuData.acc_alpha * curr_acc + (1.0f - filteredImuData.acc_alpha) * filteredImuData.acc_filt[i];
    // LPF Gyro
    float curr_gyro = rawImuData.gyro[i] - rawImuData.gyroBias[i];
    filteredImuData.gyro_filt[i] = filteredImuData.gyro_alpha * curr_gyro + (1.0f - filteredImuData.gyro_alpha) * filteredImuData.gyro_filt[i];
  }
  
  // Apply Mag calibration and LPF
  float temp_x = rawImuData.mag[0] - mag_bias[0];
  float temp_y = rawImuData.mag[1] - mag_bias[1];
  float temp_z = rawImuData.mag[2] - mag_bias[2];

  float mag_corrected[3]; 
  mag_corrected[0] = mag_scale[0][0]*temp_x + mag_scale[0][1]*temp_y + mag_scale[0][2]*temp_z;
  mag_corrected[1] = mag_scale[1][0]*temp_x + mag_scale[1][1]*temp_y + mag_scale[1][2]*temp_z;
  mag_corrected[2] = mag_scale[2][0]*temp_x + mag_scale[2][1]*temp_y + mag_scale[2][2]*temp_z;

  for (int i = 0; i < 3; i++){
    filteredImuData.mag_filt[i] = filteredImuData.mag_alpha * mag_corrected[i] + (1.0f - filteredImuData.mag_alpha) * filteredImuData.mag_filt[i]; 
  }
  
  // LPF Baro
  filteredImuData.baroP_filt = filteredImuData.baro_alpha * rawImuData.baroP + (1.0f - filteredImuData.baro_alpha) * filteredImuData.baroP_filt;
}

void calibrateAccGyro(){  
  const int numSamples = 2000; 
  float accBias[3] = {0, 0, 0};
  float gyroBias[3] = {0, 0, 0}; 

  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false); 
    
    // FIX 2: Cast address to uint8_t
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true); 
    
    // int16_t ax = (Wire.read() << 8 | Wire.read());
    // int16_t ay = (Wire.read() << 8 | Wire.read());
    // int16_t az = (Wire.read() << 8 | Wire.read());
    
    // // FIX 3: Explicitly read the temp bytes to clear the buffer, 
    // // but cast to void to tell the compiler to throw the data away.
    // (void)(Wire.read() << 8 | Wire.read()); 
    
    // int16_t gx = (Wire.read() << 8 | Wire.read());
    // int16_t gy = (Wire.read() << 8 | Wire.read());
    // int16_t gz = (Wire.read() << 8 | Wire.read());

    int16_t ax = Wire.read() << 8; ax |= Wire.read();
    int16_t ay = Wire.read() << 8; ay |= Wire.read();
    int16_t az = Wire.read() << 8; az |= Wire.read();
    
    // Clear temp buffer safely
    int16_t temp = Wire.read() << 8; temp |= Wire.read(); 
    (void)temp; // Cast to void to ignore unused variable warning
    
    int16_t gx = Wire.read() << 8; gx |= Wire.read();
    int16_t gy = Wire.read() << 8; gy |= Wire.read();
    int16_t gz = Wire.read() << 8; gz |= Wire.read();
    
    rawImuData.acc[0] = (float)ax / 16384.0; 
    rawImuData.acc[1] = (float)ay / 16384.0;
    rawImuData.acc[2] = (float)az / 16384.0;
    
    rawImuData.gyro[0] = (float)gx / 131.0; 
    rawImuData.gyro[1] = (float)gy / 131.0;
    rawImuData.gyro[2] = (float)gz / 131.0;
    
    for (int j = 0; j < 3; j++){
      accBias[j] += rawImuData.acc[j]; 
      gyroBias[j] += rawImuData.gyro[j]; 
    }
    delay(1); 
  }

  for (int j = 0; j < 3; j++){
    accBias[j] /= numSamples; 
    gyroBias[j] /= numSamples; 
    rawImuData.accBias[j] = accBias[j];
    rawImuData.gyroBias[j] = gyroBias[j];
  }

  // Remove 1g from accZ (Important)
  rawImuData.accBias[2] -= 1.0;

  // Save calibration data 
  savedCalib.signature = CALIB_SIGNATURE;
  
  for (int i = 0; i < 3; i++) {
    savedCalib.accBias[i] = rawImuData.accBias[i];
    savedCalib.gyroBias[i] = rawImuData.gyroBias[i];
  }

  EEPROM.put(EEPROM_CALIB_ADDR, savedCalib);
}

bool loadCalibrationData(){
  CalibrationStore tempCalib;
  EEPROM.get(EEPROM_CALIB_ADDR, tempCalib);
  if (tempCalib.signature == CALIB_SIGNATURE) {
    for (int i = 0; i < 3; i++) {
      rawImuData.accBias[i] = tempCalib.accBias[i];
      rawImuData.gyroBias[i] = tempCalib.gyroBias[i];
    }
    return true; 
  } else {
    for (int i = 0; i < 3; i++) {
      rawImuData.accBias[i] = 0.0f;
      rawImuData.gyroBias[i] = 0.0f;
    }
    return false;
  }
}

float invSqrt(float x) {
  return 1.0/sqrtf(x); //Teensy is fast enough
}

// Madgwick AHRS
// Compromize - Use 6DOF to estimate roll and pitch and avoid corruption from wrong mag. readings. 
// Yaw is estimated in a separate Kalman filter
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  
  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174532925f; 
  gy *= 0.0174532925f;
  gz *= 0.0174532925f;
  
  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  
  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    
    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }
  
  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;
  
  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  // --- COMPUTE ANGLES (PURE NED FRAME) ---
  rollMADG  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.29577951f;
  pitchMADG = asinf(constrain(2.0f * (q0*q2 - q3*q1), -0.999999f, 0.999999f)) * 57.29577951f;
  yawMADG   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.29577951f;
  if (yawMADG < 0.0f) yawMADG += 360.0f; 
}

void getYawKalman(float rollValDeg, float pitchValDeg, float Mx, float My, float Mz, float Gz, float dt){
  float roll_rad  = rollValDeg * 0.0174532925f;
  float pitch_rad = pitchValDeg * 0.0174532925f;

  float cr = cosf(roll_rad);
  float sr = sinf(roll_rad);
  float cp = cosf(pitch_rad);
  float sp = sinf(pitch_rad);

  // Tilt Compensation Math 
  float Xh = Mx * cp + My * sr * sp + Mz * cr * sp;
  float Yh = My * cr - Mz * sr;
  float measured_yaw = atan2f(Yh, Xh) * 57.29577951f; 

  // Update the Yaw Kalman Filter! And Low Pass Filter it 
  yawKalman = kalmanYaw.update(dt, Gz, measured_yaw) * kalmanYaw.yaw_alpha + (1.0f-kalmanYaw.yaw_alpha)*yawKalman;
}

void computeDesiredSetpoint(){
  // Assume radio has been updated
  // Code for only manual mode and stabilize now

  // --- FAILSAFE ENFORCEMENT ---
  if (millis() - radio.last_update > RADIO_SIGNAL_TIMEOUT_MS) {
    armed = false; 
    flight_mode = MODE_FAILSAFE;
    throttle_in = 0.0f;
    roll_setpoint = 0.0f;
    pitch_setpoint = 0.0f;
    yaw_rate_setpoint = 0.0f;
    return; // Abort processing old radio data
  }

  // Use radio commands to alter status 
  if (radio.arm_cmd < 1500){
    armed = false; 
  } else {
    armed = true; 
  }

  if (radio.mode < 1300) {
    flight_mode = MODE_MANUAL; 
  } else if (radio.mode > 1300 && radio.mode < 1700){
    flight_mode = MODE_STABILIZE; 
  } else {
    flight_mode = MODE_AUTO; 
  }

  if (!armed){
    throttle_in = 0.0; 
    roll_setpoint = 0.0; 
    pitch_setpoint = 0.0;
    yaw_rate_setpoint = 0.0; 
    return; 
  }

  if (flight_mode == MODE_MANUAL || flight_mode == MODE_STABILIZE || flight_mode == MODE_AUTO){
    // Calculate throttle_in 
    if (flight_mode == MODE_MANUAL){
      throttle_in = constrain((radio.throttle - 1000.0f) / 1000.0f, 0.0f, 1.0f);

      // Apply RC Deadband (e.g., +/- 8 from center)
      float roll_diff = radio.roll - 1500.0f;
      if (abs(roll_diff) < 8.0f) roll_diff = 0.0f;
      
      float pitch_diff = radio.pitch - 1500.0f;
      if (abs(pitch_diff) < 8.0f) pitch_diff = 0.0f;

      roll_setpoint = roll_diff / 500.0f * 30.0f; 
      pitch_setpoint = pitch_diff / 500.0f * 30.0f;

    } else if (flight_mode == MODE_STABILIZE){
      // Map throttle to vertical position - This is intended to ensure max and min altitude limits 
      float throttle_deadband = 100.0f; // Always start with Manual mode to bring throttle to 1500
      float difference = radio.throttle - 1500.0f;
      if (difference > throttle_deadband){
        difference -= throttle_deadband;
        alt_setpoint = initialAlt_setpoint + 0.0015f * difference; 
      } else if (difference < -throttle_deadband){
        difference += throttle_deadband;
        alt_setpoint = initialAlt_setpoint + 0.000625f * difference; 
      } else {
        alt_setpoint = initialAlt_setpoint; 
      }

      // Apply RC Deadband (e.g., +/- 8 from center)
      float roll_diff = radio.roll - 1500.0f;
      if (abs(roll_diff) < 8.0f) roll_diff = 0.0f;
      
      float pitch_diff = radio.pitch - 1500.0f;
      if (abs(pitch_diff) < 8.0f) pitch_diff = 0.0f;

      roll_setpoint = roll_diff / 500.0f * 30.0f; 
      pitch_setpoint = pitch_diff / 500.0f * 30.0f;

      // Check if we are in MODE_AUTO
      if (flight_mode == MODE_AUTO){
        // In AUTO, we will override the roll and pitch setpoints with position hold corrections
        // Use a velocity setpoint approach 
        // Map roll and pitch stick to velocities in the X and Y directions (Body frame)
        float vel_deadband = 10.0f;
        if (abs(radio.pitch - 1500.0f) < vel_deadband) {
          vel_x_setpoint = 0.0f;
        } else {
            vel_x_setpoint = (radio.pitch - 1500.0f) / 500.0f * 2.0f; // Max 2 m/s forward/backward
        }  
        
        if (abs(radio.roll - 1500.0f) < vel_deadband) {
          vel_y_setpoint = 0.0f;
        } else {
            vel_y_setpoint = (radio.roll - 1500.0f) / 500.0f * 2.0f; // Max 2 m/s left/right
        }

        float vel_x_error = vel_x_setpoint - positionEstimator.velX_est; 
        float vel_y_error = vel_y_setpoint - positionEstimator.velY_est;

        roll_setpoint = positionHoldVel.get_angle_setpoint(vel_y_error);
        pitch_setpoint = -positionHoldVel.get_angle_setpoint(vel_x_error); // Negate because forward stick should pitch down

        roll_setpoint = constrain(roll_setpoint, -20.0f, 20.0f);
        pitch_setpoint = constrain(pitch_setpoint, -20.0f, 20.0f);
      }
    } 

    // Yaw: Needs careful treatment - Heading hold!
    // Approach - Controlled w/ rate mode but held w/ angle mode

    float yaw_deadband = 20.0f;
    if (abs(radio.yaw - 1500) > yaw_deadband) {
      // --- TURNING MODE (Rate Control) ---
      yaw_rate_setpoint = map(radio.yaw, 1000, 2000, -100.0f, 100.0f);
      yaw_angle_setpoint = yawKalman;
    } else {
        // --- HEADING HOLD MODE (Angle Control) ---
        float yaw_angle_error = yaw_angle_setpoint - yawKalman;
        // Handle the 0/360 degree compass wrap-around
        if (yaw_angle_error > 180.0f)  yaw_angle_error -= 360.0f;
        if (yaw_angle_error < -180.0f) yaw_angle_error += 360.0f;
        // The Angle PID looks at the degree error and outputs the required turning speed
        // Simple P-controller on the outer loop
        yaw_rate_setpoint = pidYaw.Kp_pos * yaw_angle_error;
    }
  }
}

// Motor DShot600 Implementation
inline void wait_cycles(uint32_t cycles) {
  uint32_t start = ARM_DWT_CYCCNT;
  while (ARM_DWT_CYCCNT - start < cycles);
}

uint16_t createDshotPacket(uint16_t throttle, bool telemetry) {
  throttle = constrain(throttle, 0, 2047);
  throttle = throttle << 1;
  throttle |= telemetry;
  uint16_t csum = 0;
  uint16_t csum_data = throttle;
  for (int i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }
  csum &= 0xF;
  return (throttle << 4) | csum;
}

inline void sendBit(int pin, bool bitVal) {
  uint32_t totalCycles = CPU_MHZ * DSHOT600_BIT_US;
  uint32_t highCycles = bitVal ? totalCycles * 0.75 : totalCycles * 0.375;
  digitalWriteFast(pin, HIGH);
  wait_cycles(highCycles);
  digitalWriteFast(pin, LOW);
  wait_cycles(totalCycles - highCycles);
}

void sendDshotFrame(int pin, uint16_t throttle) {
  uint16_t packet = createDshotPacket(throttle, false);
  noInterrupts();
  for (int i = 15; i >= 0; i--) {
    sendBit(pin, packet & (1 << i));
  }
  interrupts();
}

void updateMotors() {
  float throttle_dshot = 48.0f; // Minimum spinning idle

  if (flight_mode == MODE_MANUAL){
    // Scale 0.0 - 1.0 directly to the 2000-point DShot range
    throttle_dshot = 48.0f + (throttle_in * 1999.0f); 
  } else if (flight_mode == MODE_STABILIZE || flight_mode == MODE_AUTO){
    // Feed-forward hover base (approx 50% throttle for a 2:1 TWR drone) + Alt PID correction
    float hover_base = 0.5f; 
    float alt_adjusted_throttle = constrain(hover_base + altPIDOutput, 0.0f, 1.0f);
    throttle_dshot = 48.0f + (alt_adjusted_throttle * 1999.0f);
  }

  // Mix motors
  // motor_output[0] = constrain(throttle_dshot - rollPIDOutput + pitchPIDOutput - yawPIDOutput, 48, 2047); 
  // motor_output[1] = constrain(throttle_dshot + rollPIDOutput + pitchPIDOutput + yawPIDOutput, 48, 2047); 
  // motor_output[2] = constrain(throttle_dshot - rollPIDOutput - pitchPIDOutput + yawPIDOutput, 48, 2047); 
  // motor_output[3] = constrain(throttle_dshot + rollPIDOutput - pitchPIDOutput - yawPIDOutput, 48, 2047); 

  // New motor mapping 
  motor_output[0] = constrain(throttle_dshot - rollPIDOutput - pitchPIDOutput + yawPIDOutput, 48, 2047); 
  motor_output[1] = constrain(throttle_dshot - rollPIDOutput + pitchPIDOutput - yawPIDOutput, 48, 2047); 
  motor_output[2] = constrain(throttle_dshot + rollPIDOutput - pitchPIDOutput - yawPIDOutput, 48, 2047); 
  motor_output[3] = constrain(throttle_dshot + rollPIDOutput + pitchPIDOutput + yawPIDOutput, 48, 2047); 
  
  if (!armed || flight_mode == MODE_DISARMED || flight_mode == MODE_FAILSAFE){
    motor_output[0] = 0; // 0 is the DShot disarm command
    motor_output[1] = 0;
    motor_output[2] = 0;
    motor_output[3] = 0;

    pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); pidAlt.reset(); 
  }

  sendDshotFrame(MOTOR1, motor_output[0]);
  sendDshotFrame(MOTOR2, motor_output[1]);
  sendDshotFrame(MOTOR3, motor_output[2]);
  sendDshotFrame(MOTOR4, motor_output[3]);
}

void setupMotors() {
  motor_output[0] = 0; motor_output[1] = 0; motor_output[2] = 0; motor_output[3] = 0;
  armed = false; 
  unsigned long bootStart = millis();
  while (millis() - bootStart < 4000) {
    sendDshotFrame(MOTOR1, motor_output[0]);
    sendDshotFrame(MOTOR2, motor_output[1]);
    sendDshotFrame(MOTOR3, motor_output[2]);
    sendDshotFrame(MOTOR4, motor_output[3]);
    delay(1);       
  }
}

void printRadio(){
  Serial.print("Roll:"); Serial.print(radio.roll); Serial.print(","); 
  Serial.print("Pitch:"); Serial.print(radio.pitch); Serial.print(","); 
  Serial.print("Throttle:"); Serial.print(radio.throttle); Serial.print(","); 
  Serial.print("Yaw:"); Serial.print(radio.yaw); Serial.print(","); 
  Serial.print("Arm_CMD:"); Serial.print(radio.arm_cmd); Serial.print(","); 
  Serial.print("Mode:"); Serial.print(radio.mode); // Serial.print(","); 
}

void setup() {
  Serial.begin(115200); 
  uint32_t t_start = millis();
  while (!Serial && (millis() - t_start < 3000));
  Wire.begin();
  analogReadResolution(12);
  setupIMU(); 

  bool isCalibAvail = loadCalibrationData(); 
  if (!isCalibAvail){
    calibrateAccGyro(); 
  }
  // calibrateAccGyro(); 
  // Reload 
  loadCalibrationData(); 

  // Initialize filtered IMU data 
  // filteredImuData.acc_alpha = 0.03; // Tuned
  // filteredImuData.gyro_alpha = 0.10; // Tuned
  // filteredImuData.mag_alpha = 0.05; // Tuned
  // filteredImuData.baro_alpha = 0.0025; // Tuned

  // --- RECOMMENDED FILTER SETTINGS ---
  filteredImuData.acc_alpha = 0.086;  // Target: 15Hz (Clean Accel data)
  filteredImuData.gyro_alpha = 0.334; // Target: 80Hz (Crisp PID response)
  filteredImuData.mag_alpha = 0.059; // Target: 10 Hz
  filteredImuData.baro_alpha = 0.012; // Target: 2 Hz
  

  for (int i = 0; i < 3; i++){
    filteredImuData.acc_filt[i] = 0.0; 
    filteredImuData.gyro_filt[i] = 0.0; 
    filteredImuData.mag_filt[i] = 0.0; 
  }
  filteredImuData.acc_filt[2] = 1.0; // 1g normalized

  // Initialize Roll and Pitch Kalman filters 
  kalmanRoll.init();
  kalmanPitch.init();
  
  // Initialize Yaw Kalman filter
  kalmanYaw.init();
  
  // Home calibration comes first for new attitude observer 
  altitude_luenberger.calibrateBaroHome(); 
  altitude_luenberger.init();

  // Initialize position estimators 
  positionEstimator.init();

  // Initialize PID controllers 
  pidRoll.Kp_rate = 2.5;
  pidRoll.Ki_rate = 0.02;
  pidRoll.Kd_rate = 0.035; 
  pidRoll.Kp_pos = 8.0;
  pidRoll.prevErrorRate = 0.0;
  pidRoll.integralRate = 0.0;
  pidRoll.derivativeRate = 0.0;
  pidRoll.LPF_beta = 0.5;
  pidRoll.MAX_I_OUTPUT = 250.0; 
  pidRoll.output_multiplier = 1.0f;

  pidPitch.Kp_rate = 2.5;
  pidPitch.Ki_rate = 0.02;
  pidPitch.Kd_rate = 0.035;
  pidPitch.Kp_pos = 8.0;
  pidPitch.prevErrorRate = 0.0;
  pidPitch.integralRate = 0.0;
  pidPitch.derivativeRate = 0.0;
  pidPitch.LPF_beta = 0.5;
  pidPitch.MAX_I_OUTPUT = 250.0; 
  pidPitch.output_multiplier = 1.0f;

  pidYaw.Kp_rate = 1.5;
  pidYaw.Ki_rate = 0.02;
  pidYaw.Kd_rate = 0.035;
  pidYaw.Kp_pos = 3.0;
  pidYaw.prevErrorRate = 0.0;
  pidYaw.integralRate = 0.0;
  pidYaw.derivativeRate = 0.0;
  pidYaw.LPF_beta = 0.3;
  pidYaw.MAX_I_OUTPUT = 200.0;
  pidYaw.output_multiplier = 1.0f;

  // Kp = 0.3, Ki = 0.001, Kd = 0.2, LPF_beta = 0.75, MAX_I = 0.5
  pidAlt.init(0.3f, 0.001f, 0.2f, 0.75f, 0.5f);

  // Initialize position hold 
  positionHoldVel.init(); 

  // Radio setup 
  // Initialize iBUS Serial (Hardware Serial 7 on Teensy Pin 28)
  Serial7.begin(115200);
  radio.init(); 

  // Initialize DShot timers 
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  pinMode(MOTOR1, OUTPUT); pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT); pinMode(MOTOR4, OUTPUT);

  // ESC Setup (Sending DShot '0' for 4 seconds)
  setupMotors(); 

  // Apply the 400kHz clock here so no libraries can override it!
  Wire.setClock(400000);
  // 
  prevTime = micros();
}

void loop() {
  uint32_t now = micros(); 
  // CRITICAL SAFETY CATCH: Prevent divide-by-zero on the first loop
  if (now == prevTime) return;

  float dt = (now - prevTime) / 1000000.0; 
  prevTime = now; 

  readRawIMU(now, rawImuData); 
  filterIMUData(); 

  // Remap angles to NED frame
  // float Ax_NED = filteredImuData.acc_filt[0], Ay_NED = -filteredImuData.acc_filt[1], Az_NED = -filteredImuData.acc_filt[2]; 
  // float Gx_NED = filteredImuData.gyro_filt[0], Gy_NED = -filteredImuData.gyro_filt[1], Gz_NED = -filteredImuData.gyro_filt[2]; 
  float Mx_NED = filteredImuData.mag_filt[0], My_NED = -filteredImuData.mag_filt[1], Mz_NED = -filteredImuData.mag_filt[2];

  // Compute attitude with Madgwick
  // Madgwick6DOF(Gx_NED, Gy_NED, Gz_NED, Ax_NED, Ay_NED, Az_NED, dt);

  // Feed Madgwick the native MPU6050 axes (Forward, Left, Up)
  float Ax_FLU = filteredImuData.acc_filt[0]; 
  float Ay_FLU = filteredImuData.acc_filt[1]; 
  float Az_FLU = filteredImuData.acc_filt[2]; // Z is now positive!

  float Gx_FLU = filteredImuData.gyro_filt[0]; 
  float Gy_FLU = filteredImuData.gyro_filt[1]; 
  float Gz_FLU = filteredImuData.gyro_filt[2]; 

  // Compute attitude with Madgwick
  Madgwick6DOF(Gx_FLU, Gy_FLU, Gz_FLU, Ax_FLU, Ay_FLU, Az_FLU, dt);

  // Estimate roll and pitch from acc data 
  float accRoll = atan2(Ay_FLU, sqrtf(Ax_FLU * Ax_FLU + Az_FLU * Az_FLU)) * 180.0f / PI;
  float accPitch = atan2(-Ax_FLU, sqrtf(Ay_FLU * Ay_FLU + Az_FLU * Az_FLU)) * 180.0f / PI;

  // TEST: Use 1D Kalman for roll and pitch
  kalmanRoll.update(dt, Gx_FLU, accRoll);
  kalmanPitch.update(dt, Gy_FLU, accPitch);

  // Get yawKalman
  // getYawKalman(rollMADG, pitchMADG, Mx_NED, My_NED, Mz_NED, -Gz_FLU, dt); 
  getYawKalman(kalmanRoll.angle, kalmanPitch.angle, Mx_NED, My_NED, Mz_NED, -Gz_FLU, dt); 

  // --------------------------------  CORRECT HERE ---------------------------------// 
  // Rotate Body-Frame Acc to Earth-Frame Acc in Z-axis using 6DOF Quaternions
  // The Yaw drift in q0-q3 does not affect vertical acceleration
  // float accZ_earth_Gs = 2.0f * (q1*q3 - q0*q2) * Ax_FLU 
  //                     + 2.0f * (q2*q3 + q0*q1) * Ay_FLU 
  //                     + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * Az_FLU;
                        
  // Another method to find accZ_earth_Gs
  float roll_rad  = kalmanRoll.angle * 0.0174532925f;
  float pitch_rad = kalmanPitch.angle * 0.0174532925f;
  float cr = cosf(roll_rad);
  float sr = sinf(roll_rad);
  float cp = cosf(pitch_rad);
  float sp = sinf(pitch_rad);
  float accZ_earth_Gs = (-sp * Ax_FLU) 
                      + (sr * cp * Ay_FLU) 
                      + (cr * cp * Az_FLU);

  accZ_earth_Gs -= 1.0f; // remove gravity => linear acc.
  accZ_earth_ms2 = accEarthAlpha * accZ_earth_ms2 + (1 - accEarthAlpha) * accZ_earth_Gs * 9.80665f;

  // Use raw baroH for better response (but more noise) since Luenberger can handle it
  float rawBaroH = 44330.0f * (1.0f - pow(rawImuData.baroP / 101325.0f, 0.1903f));
  float relative_altitude = rawBaroH - altitude_luenberger.home_altitude; 

  // Update altitude observer 
  altitude_luenberger.update(dt, accZ_earth_ms2, relative_altitude, Barometer.newDataAvailable);

  // Update altitude and velocity in the global scope 
  float lpf_alt = 0.90f; 
  finalAlt = finalAlt * lpf_alt + altitude_luenberger.alt_est * (1 - lpf_alt);
  finalVertVel = altitude_luenberger.vel_est;

  // Estimate position in X and Y 
  positionEstimator.update(dt, kalmanRoll.angle, kalmanPitch.angle, yawKalman, filteredImuData); 
  
  // Read radio
  radio.readIBUS(); 

  // Update desired state 
  computeDesiredSetpoint(); 

  // Update PIDs 
  // With Madgwick angles -
  // rollPIDOutput = pidRoll.update(dt, roll_setpoint, rollMADG, Gx_FLU); // Dogru
  // pitchPIDOutput = pidPitch.update(dt, pitch_setpoint, pitchMADG, Gy_FLU); // Dogru

  // With Kalman angles -
  rollPIDOutput = pidRoll.update(dt, roll_setpoint, kalmanRoll.angle, Gx_FLU); // Dogru
  pitchPIDOutput = pidPitch.update(dt, pitch_setpoint, kalmanPitch.angle, Gy_FLU); // Dogru

  yawPIDOutput = pidYaw.updateRate(dt, yaw_rate_setpoint, -Gz_FLU); // With negative => Dogru
  altPIDOutput = pidAlt.update(dt, alt_setpoint, finalAlt);

  // --- PID SUM LIMITS (Safety Clamps) ---
  rollPIDOutput  = constrain(rollPIDOutput,  -400.0f, 400.0f);
  pitchPIDOutput = constrain(pitchPIDOutput, -400.0f, 400.0f);
  yawPIDOutput   = constrain(yawPIDOutput,   -300.0f, 300.0f);
  altPIDOutput   = constrain(altPIDOutput,   -0.30f, 0.30f);
  // --------------------------------------

  // Command motors
  updateMotors();

  // // Print at a lower rate
  // if (millis() - lastPrintTime > 20) {
  //   Serial.print(">");  

  //   Serial.print("FinalAlt:"); Serial.print(finalAlt); Serial.print(",");

  //   // Serial.print("posX:"); Serial.print(positionEstimator.posX_L); Serial.print(",");
  //   // // Serial.print("posY:"); Serial.print(positionEstimator.posY_L); Serial.print(",");
  //   // Serial.print("velX:"); Serial.print(positionEstimator.velX_est); Serial.print(",");
  //   // Serial.print("velY:"); Serial.print(positionEstimator.velY_est); Serial.print(",");
  //   // Serial.print("rollKalman:"); Serial.print(kalmanRoll.angle); Serial.print(",");
  //   // Serial.print("pitchKalman:"); Serial.print(kalmanPitch.angle); Serial.print(","); 
  //   // Serial.print("relAlt:"); Serial.print(relative_altitude); Serial.print(",");
  //   // Serial.print("FinalAlt:"); Serial.print(finalAlt); Serial.print(",");
  //   // Serial.print("FinalVertVel:"); Serial.print(finalVertVel); Serial.print(",");

  //   // Serial.print("Az: "); Serial.print(rawImuData.acc[2]); Serial.print(",");
  //   // Serial.print("Az_filt: "); Serial.print(filteredImuData.acc_filt[2]); 

  //   // Serial.print("Gx: "); Serial.print(rawImuData.gyro[0]); Serial.print(",");
  //   // Serial.print("Gx_filt: "); Serial.print(filteredImuData.gyro_filt[0]);

  //   // Serial.print("Mx: "); Serial.print(rawImuData.mag[0]); Serial.print(",");
  //   // Serial.print("Mx_filt: "); Serial.print(filteredImuData.mag_filt[0]); // Serial.print(",");
  //   // Serial.print("My: "); Serial.print(rawImuData.mag[1]); Serial.print(",");
  //   // Serial.print("Mz: "); Serial.print(rawImuData.mag[2]); // Serial.print(",");
  //   // Serial.print("Mx_filt: "); Serial.print(filteredImuData.mag_filt[0]); Serial.print(",");
  //   // Serial.print("My_filt: "); Serial.print(filteredImuData.mag_filt[1]); Serial.print(",");
  //   // Serial.print("My: "); Serial.print(rawImuData.mag[1]); Serial.print(",");
  //   // Serial.print("Mz: "); Serial.print(rawImuData.mag[2]); // Serial.print(",");

  //   // Serial.print("BaroP:"); Serial.print(rawImuData.baroP); Serial.print(",");
  //   // Serial.print("BaroP_filt:"); Serial.print(filteredImuData.baroP_filt); Serial.print(",");
  //   // Serial.print("BaroP2P_filt:"); Serial.print(filteredImuData.baroP2P_filt); Serial.print(",");

  //   // Serial.print("BaroH:"); Serial.print(rawImuData.baroH); Serial.print(",");
  //   // Serial.print("BaroH_filt:"); Serial.print(filteredImuData.baroH_filt);

  //   // Serial.print("rollMADG:"); Serial.print(rollMADG); Serial.print(",");
  //   // Serial.print("pitchMADG:"); Serial.print(pitchMADG); Serial.print(",");
  //   // Serial.print("rollKalman:"); Serial.print(kalmanRoll.angle); Serial.print(",");
  //   // Serial.print("pitchKalman:"); Serial.print(kalmanPitch.angle); Serial.print(","); 

  //   // Serial.print("Gx_FLU:"); Serial.print(Gx_FLU); Serial.print(","); 
  //   // Serial.print("Gy_FLU:"); Serial.print(Gy_FLU); Serial.print(",");  
  //   // Serial.print("yawKalman:"); Serial.print(yawKalman); Serial.print(",");
  //   // Serial.print("Gz_FLU:"); Serial.print(-Gz_FLU); Serial.print(","); // with negative

  //   // Serial.print("accZEarth:"); Serial.print(accZ_earth_ms2); Serial.print(","); 
  //   // Serial.print("relAlt:"); Serial.print(relative_altitude); Serial.print(",");
  //   // Serial.print("FinalAlt:"); Serial.print(finalAlt); Serial.print(",");
  //   // Serial.print("FinalVertVel:"); Serial.print(finalVertVel); // Serial.print(",");

  //   // printRadio();

  //   Serial.println(); 
  //   lastPrintTime = millis(); // Reset the print timer
  // }

  // Place this at the absolute bottom of loop() before loopRate()
  if (Barometer.newDataAvailable) {
    Barometer.newDataAvailable = false;
  }

  loopRate(LOOP_RATE_HZ); 
}