#include "main.h"
#include "venturi.h"
#include "fan.h"
#include "PidController.h"
#include <pid.h>
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>
#include <SensirionCore.h>
#include <SensirionI2CSdp.h>
#include <Adafruit_SH110X.h>
#include <Fonts/Picopixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Smoothed.h>

// Pin definitions
const int FAN_ON_PIN  = 32;
const int FAN_PULL_PWM_PIN = 27;
const int FAN_PUSH_PWM_PIN = 14;
const int FAN_PULL_TACHO_PIN = 34;
const int FAN_PUSH_TACHO_PIN = 35;

//I2C pin definitions
const int I2C_SDA0_PIN = 21;
const int I2C_SCL0_PIN = 22;
const int I2C_SDA1_PIN = 25;
const int I2C_SCL1_PIN = 26;

// Display
const int I2C_ADDRESS_DISPLAY = 0x3C;
const int I2C_ADDRESS_BME280  = 0x76;
const int I2C_ADDRESS_SDP810  = 0x25;

// Rotary encoder
const int ROTARY_ENCODER_A_PIN = 18;
const int ROTARY_ENCODER_B_PIN = 19;
const int ROTARY_ENCODER_BUTTON_PIN = 5;
const int ROTARY_ENCODER_STEPS = 4;

// PWM settings
const int PWM_PULL_FAN_CHAN = LEDC_CHANNEL_0;
const int PWM_PUSH_FAN_CHAN = LEDC_CHANNEL_1;
const int PWM_FREQ = 25000;    // 25 kHz frequency for computer fans
const int PWM_RESOLUTION = 10; // 10-bit resolution (0-1023)
const int PWM_DITHER_RESOLUTION = 2; // 2-bit resolution for dithering (0-3)

// tachometer
volatile uint32_t tachoFanPulseCycles = 300;
volatile uint32_t tachoPushFanPulseCount = 0;
volatile uint32_t tachoPullFanPulseCount = 0;
volatile uint32_t tachoPullFanRPM = 0;
volatile uint32_t tachoPushFanRPM = 0;

// Venturi
Venturi venturi;

Fan pullFan(FAN_PULL_PWM_PIN, PWM_PULL_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION, PWM_DITHER_RESOLUTION);
Fan pushFan(FAN_PUSH_PWM_PIN, PWM_PUSH_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION, PWM_DITHER_RESOLUTION);

//paramaters for button
const unsigned long shortPressAfterMiliseconds = 50;   // how long short press shoud be.
                                                       // Do not set too low to avoid bouncing (false press events).
const unsigned long longPressAfterMiliseconds = 1000;  // how long long press shoud be.

AiEsp32RotaryEncoder *rotaryEncoder = new AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS, false);
AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();

Preferences preferences;

Smoothed<float> venturiPressure;
Smoothed<float> balancePressure;
Smoothed<float> pressureAbsolute;
Smoothed<float> temperatureAmbient;
Smoothed<float> humidityAmbient;
Smoothed<float> calibration;

// Twee hardware I2C bussen
TwoWire I2C_A = TwoWire(0);
TwoWire I2C_B = TwoWire(1);

SensirionI2CSdp sdpVenturi;
SensirionI2CSdp sdpBalance;

// Initialize the OLED display using Wire library
// ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically
// based on your board's pins_arduino.h 
// e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SH1106Wire      display(I2C_ADDRESS_DISPLAY, I2C_SDA0_PIN, I2C_SCL0_PIN, GEOMETRY_128_64);
Adafruit_SH1106G display(128, 64, &I2C_A);
Adafruit_BME280  bme280;     // I2C

hw_timer_t *timer0 = nullptr;
volatile bool ms10_passed = false;

FollowFanType followFanType = FOLLOW_PULL_FAN;
ModeType modeType = MT_OPERATION;
PidTuneType pidTuneType = PID_TUNE_NONE;
OperatingMode operatingMode = OM_SPEED_PULL_FAN;
VenturiConstantsType venturiConstantsType = VENTURI_CONSTANTS_SET_NONE;

double outputMasterFan = 0.0;
double KpFlow=2, KiFlow=0, KdFlow=0;
double flowAlpha=0.2, flowBeta=0.02, flowDamping=0.95, flowTrendLimit=5;
PidController pidFlow(KpFlow, KiFlow, KdFlow, 0.01, 0.0, 100.0);
double outputPidBalanceFan = 0.0;
double KpBalance=6, KiBalance=3, KdBalance=0;
double balanceAlpha=0.2, balanceBeta=0.02, balanceDamping=0.95, balanceTrendLimit=5;
PidController pidBalance(KpBalance, KiBalance, KdBalance, 0.01, 0.0, 100.0);

float offsetVenturiPressure = 0.0;
float offsetBalancePressure = 0.0;
float lastEncoderOperationValue = 0.0;

static void  setupTachometers();
static void  setupTimer0();
static void  readPressureSensors();
static void  loadPreferences();
static void  setupRotaryEncoder();
static void  setMasterFan(FollowFanType type);
static void  setMasterFanSpeedPercent(float speed);
static void  setSlaveFanSpeedPercent(float speed);
static void  setOperationMode(OperatingMode mode);
static void  initDisplay(void);
static void  loopRotaryEncoder();
static void  displayMeasurements();
static void  displaySelectMode(ModeType);
static void  displaySelectOperationMode(OperatingMode mode);
static void  displayTextNumber(const char *txt, float);
static void  displayAdjustSensorOffsets(const char *sensor);
static void  displayAdjustSensorOffsetsProgress(int16_t progress);
static void  displaySelectTunePID(PidTuneType type);
static void  displaySelectVenturiConstants(VenturiConstantsType type);
static void  initBME280();
static void  initSDP(SensirionI2CSdp&, TwoWire&);
static void  drawString(int16_t x, int16_t y, const String &text);
//////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);

  Serial.println("\ntestbench_airflowmeter is starting up...");

  // prefent fan from starting up
  pinMode(FAN_ON_PIN, OUTPUT);
  digitalWrite(FAN_ON_PIN, LOW);

  setupTachometers();

  // load preferences (calibration and PID parameters)
  loadPreferences();

  // setup RotaryEncoder
  setupRotaryEncoder();

  // configure PWM
  Serial.println("Setup PWM");
  pullFan.begin();
  pushFan.begin();

  // set initial fan speeds to zero
  setMasterFanSpeedPercent(0);
  setSlaveFanSpeedPercent(0);
  delay(1000);

  // Start both busses
  I2C_A.begin(I2C_SDA0_PIN, I2C_SCL0_PIN, 700000);   // SDA, SCL
  I2C_B.begin(I2C_SDA1_PIN, I2C_SCL1_PIN, 100000);

  Serial.println("Setup Display");
  initDisplay();
  delay(500);
  Serial.println("Setup BME280");
  initBME280();
  delay(500);
  Serial.println("Setup SDP810 Venturi");
  initSDP(sdpVenturi, I2C_A);
  delay(500);
  Serial.println("Setup SDP810 Balance");
  initSDP(sdpBalance, I2C_B);
  delay(500);

  Serial.println("setup smoothed average.");
  balancePressure.begin(SMOOTHED_EXPONENTIAL, 10);
  venturiPressure.begin(SMOOTHED_AVERAGE, 10);
  calibration.begin(SMOOTHED_AVERAGE, 200);
  pressureAbsolute.begin(SMOOTHED_AVERAGE, 40);
  humidityAmbient.begin(SMOOTHED_AVERAGE, 40);
  temperatureAmbient.begin(SMOOTHED_AVERAGE, 40);
  delay(500);

  // initialize the Flow PID variables
  Serial.println("turn the PID Flow on.");
  pidFlow.setTunings(KpFlow, KiFlow, KdFlow);
  pidFlow.setOutputLimits(0, 100.0); // Output will be in percentage of max PWM (0-100%)
  pidFlow.setSampleTime(0.01); // 10 ms sample time
  pidFlow.setAlpha(flowAlpha);
  pidFlow.setBeta(flowBeta);
  pidFlow.setDamping(flowDamping);
  pidFlow.setTrendLimit(flowTrendLimit);
  delay(500);

  // initialize the Balance PID variables
  Serial.println("turn the PID Balance on.");
  pidBalance.setTunings(KpBalance, KiBalance, KdBalance);
  pidBalance.setOutputLimits(0, 100);
  pidBalance.setSampleTime(0.01);
  pidBalance.setAlpha(balanceAlpha);
  pidBalance.setBeta(balanceBeta);
  pidBalance.setDamping(balanceDamping);
  pidBalance.setTrendLimit(balanceTrendLimit);
  delay(500);

  Serial.print("Setup timer interrupt\n");
  setupTimer0();
  delay(500);

  bme280.takeForcedMeasurement();
  venturi.setRho(bme280.readPressure(), bme280.readTemperature(), bme280.readHumidity());
  
  Serial.print("\n Setup done...\n\n");
  delay(500);
}
//////////////////////////////////////////////////////////////////////////

void loop() {
  static uint32_t loopcnt = 0;
  static uint32_t last_millis = 0;
  uint32_t now = millis();
  
  readPressureSensors();

  loopRotaryEncoder();

  // every 100ms
  if (now >= last_millis + 100) {
    last_millis = now;

    readPressureSensors();

    venturi.loop(venturiPressure.get());

    readPressureSensors();
    
    // every second
    if (loopcnt % 10 == 0) {

      // Only needed in forced mode! In normal mode, you can remove the next line.
      bme280.takeForcedMeasurement(); // has no effect in normal mode
      readPressureSensors();

      // get the measurements from the BME280
      pressureAbsolute.add(bme280.readPressure()); // Pa
      readPressureSensors();
      temperatureAmbient.add(bme280.readTemperature()); // °C
      readPressureSensors();
      humidityAmbient.add(bme280.readHumidity());
      readPressureSensors();

      Serial.printf("Flow: %.1f m3/s\n", venturi.getFlow());
      Serial.printf("Dutycycle Masterfan: %.1f%%\n", outputMasterFan);
      Serial.printf("Balance Pressure: %.1f Pa\n", balancePressure.get());
      Serial.printf("Dutycycle Slavefan: %.1f%%\n", outputPidBalanceFan);

      readPressureSensors();
    }

    // every minute
    if (loopcnt % 600 == 0) {
       venturi.setRho(pressureAbsolute.get(), temperatureAmbient.get(), humidityAmbient.get());
       readPressureSensors();
    }
    
    // every 2 seconds
    if (loopcnt % 20 == 0) {
      if (modeType == MT_OPERATION 
          || modeType == MT_CAL_FLOW
          || pidTuneType != PID_TUNE_NONE
      ) {
        displayMeasurements(); // 42ms
      }
    }

    ++loopcnt;
  }
}
//////////////////////////////////////////////////////////////////////////

// Interrupt Service Routine (ISR)
// IRAM_ATTR places the function in RAM for faster execution
void IRAM_ATTR Timer0_ISR() {
  ms10_passed = true;
  if (--tachoFanPulseCycles == 0) {
    tachoPullFanRPM = tachoPullFanPulseCount * 10;
    tachoPullFanPulseCount = 0;
    tachoPushFanRPM = tachoPushFanPulseCount * 10;
    tachoPushFanPulseCount = 0;
    tachoFanPulseCycles = 300;
  }

  if (tachoFanPulseCycles % 2 == 0) { // every 20ms
    pushFan.handleDither();
  } else { // every 20ms
    pullFan.handleDither();
  }
}
//////////////////////////////////////////////////////////////////////////

void IRAM_ATTR readEncoder_ISR() {
    rotaryEncoder->readEncoder_ISR();
}
//////////////////////////////////////////////////////////////////////////

void IRAM_ATTR readButton_ISR() {
    rotaryEncoder->readButton_ISR();
}
//////////////////////////////////////////////////////////////////////////

void IRAM_ATTR tachoPullFanPulseCount_ISR() {
  tachoPullFanPulseCount++;
}
//////////////////////////////////////////////////////////////////////////

void IRAM_ATTR tachoPushFanPulseCount_ISR() {
  tachoPushFanPulseCount++;
}
//////////////////////////////////////////////////////////////////////////

static void setupTachometers() {
  pinMode(FAN_PULL_TACHO_PIN, INPUT_PULLUP); // GPIO 34
  attachInterrupt(digitalPinToInterrupt(FAN_PULL_TACHO_PIN), tachoPullFanPulseCount_ISR, FALLING);
  pinMode(FAN_PUSH_TACHO_PIN, INPUT_PULLUP); // GPIO 35
  attachInterrupt(digitalPinToInterrupt(FAN_PUSH_TACHO_PIN), tachoPushFanPulseCount_ISR, FALLING);
}
//////////////////////////////////////////////////////////////////////////

static void setupTimer0() {
  // 1. Initialize timer
  //timerBegin(timer_number, divider, countUp)
  // Prescaler 80: 80MHz / 80 = 1MHz (1 microsecond per tick)
  timer0 = timerBegin(0, 80, true);

  // 2. Attach the ISR function
  timerAttachInterrupt(timer0, &Timer0_ISR, true);

  // 3. Set alarm to fire every 10000 ticks (10000 us = 10ms)
  // timerAlarmWrite(timer, microseconds, autoreload)
  timerAlarmWrite(timer0, 10000, true);

  // 4. Enable the alarm
  timerAlarmEnable(timer0);
}
//////////////////////////////////////////////////////////////////////////

static void readPressureSensors() {
  
  if (ms10_passed == true) {
    ms10_passed = false;
    float differentialPressure, temperature;

    // 2 ms time to read
    uint16_t error = sdpVenturi.readMeasurement(differentialPressure, temperature);
    if (error) {
      Serial.print("Error trying to execute readMeasurement from Venturi Pressure sensor");
      venturiPressure.add(0.0);
    } else {
      differentialPressure -= offsetVenturiPressure;
      venturiPressure.add(differentialPressure);
      if (modeType != MT_ADJUST_OFFSETS) {
        if (operatingMode == OM_FLOW_PULL_FAN || operatingMode == OM_FLOW_PUSH_FAN) {
          outputMasterFan = pidFlow.update(differentialPressure);
          setMasterFanSpeedPercent(outputMasterFan);
        }
      }
    }

    error = sdpBalance.readMeasurement(differentialPressure, temperature);
    if (error) {
      Serial.print("Error trying to execute readMeasurement from Balance Pressure sensor");
      balancePressure.add(0.0);
    } else {
      differentialPressure -= offsetBalancePressure;
      balancePressure.add(differentialPressure);
      setSlaveFanSpeedPercent(pidBalance.update(differentialPressure)); 
    }
  }
}
//////////////////////////////////////////////////////////////////////////

static float getFloat(const char* key, float value = NAN) {
  float f;
  preferences.begin("testbench", false);
  f = preferences.getFloat(key, value);
  preferences.end();
  return f == NAN ? 0.0 : f;
}
//////////////////////////////////////////////////////////////////////////

static void saveFloat(const char* key, float value) {
  preferences.begin("testbench", false);
  preferences.putFloat(key, value);
  preferences.end();
}
//////////////////////////////////////////////////////////////////////////

static void loadPreferences() {
  preferences.begin("testbench", true);

  float inletDiameter = preferences.getFloat(KEY_VENTURI_INLET_DIAMETER, 0.116);
  float throatDiameter = preferences.getFloat(KEY_VENTURI_THROAT_DIAMETER, 0.087);
  float dischargeCoefficient = preferences.getFloat(KEY_VENTURI_CD, 0.975);
  venturi.begin(inletDiameter, throatDiameter, dischargeCoefficient);
  venturi.setSmoothedFlowFactor(preferences.getFloat(KEY_VENTURI_SMOOTHING_FACTOR, 0.02));

  offsetBalancePressure = preferences.getFloat(KEY_OFFSET_BALANCE_PRESSURE_SENSOR, 0.0);
  offsetVenturiPressure = preferences.getFloat(KEY_OFFSET_VENTURI_PRESSURE_SENSOR, 0.0);
  KpFlow =  preferences.getFloat(KEY_KP_FLOW, KpFlow);
  KiFlow =  preferences.getFloat(KEY_KI_FLOW, KiFlow);
  KdFlow =  preferences.getFloat(KEY_KD_FLOW, KdFlow);
  flowAlpha = preferences.getFloat(KEY_ALPHA_FLOW, flowAlpha);
  flowBeta = preferences.getFloat(KEY_BETA_FLOW, flowBeta);
  flowDamping = preferences.getFloat(KEY_DAMPING_FLOW, flowDamping);
  flowTrendLimit = preferences.getFloat(KEY_TREND_LIMIT_FLOW, flowTrendLimit);
  KpBalance = preferences.getFloat(KEY_KP_BALANCE, KpBalance);
  KiBalance = preferences.getFloat(KEY_KI_BALANCE, KiBalance);
  KdBalance = preferences.getFloat(KEY_KD_BALANCE, KdBalance);
  balanceAlpha = preferences.getFloat(KEY_BALANCE_ALPHA, balanceAlpha);
  balanceBeta = preferences.getFloat(KEY_BALANCE_BETA, balanceBeta);
  balanceDamping = preferences.getFloat(KEY_BALANCE_DAMPING, balanceDamping);
  balanceTrendLimit = preferences.getFloat(KEY_BALANCE_TREND_LIMIT, balanceTrendLimit);

  preferences.end();
}
//////////////////////////////////////////////////////////////////////////

static void setupRotaryEncoder() {
  rotaryEncoder->begin();
  rotaryEncoder->setup(&readEncoder_ISR, &readButton_ISR);
  numberSelector.attachEncoder(rotaryEncoder);

  // numberSelector.setRange parameters:
  float minValue = 0.0;        // set minimum value for example -12.0
  float maxValue = 100.0;      // set maxinum value for example 12.0
  float step = 0.1;            // set step increment, default 1, can be smaller steps like 0.5 or 10
  bool cycleValues = false;    // set true only if you want going to miminum value after maximum 
  unsigned int decimals = 1;   // set precision - how many decimal places you want, default is 0
  numberSelector.setRange(minValue, maxValue,  step, cycleValues, decimals);
  numberSelector.setValue(0);  // sets initial value
}
//////////////////////////////////////////////////////////////////////////

static void setPullFanSpeed(uint32_t speed) {
  static uint32_t lastspeed = 0xFFFFFFFF;

  if (lastspeed != speed) {
    pullFan.setSpeed(speed);
    if (followFanType == FOLLOW_PULL_FAN) {
      if (speed == 0) {
        digitalWrite(FAN_ON_PIN, LOW);
      } else if (lastspeed == 0) {
        digitalWrite(FAN_ON_PIN, HIGH);
      }
      lastspeed = speed;
    } else {
      lastspeed = 0xFFFFFFFF;
    }
  }
} 
//////////////////////////////////////////////////////////////////////////

static void setPullFanSpeedPercent(float percent) {
  uint16_t speed = (uint16_t)(percent * pullFan.getMaxSpeed() / 100.0);
  setPullFanSpeed(speed);
}
//////////////////////////////////////////////////////////////////////////

static void setPushFanSpeed(uint32_t speed) {
  static uint32_t lastspeed = 0xFFFFFFFF;

  if (lastspeed != speed) {
    pushFan.setSpeed(speed);
    if (followFanType == FOLLOW_PUSH_FAN) {
      if (speed == 0) {
        digitalWrite(FAN_ON_PIN, LOW);
      } else if (lastspeed == 0) {
        digitalWrite(FAN_ON_PIN, HIGH);
      }
      lastspeed = speed;
    } else {
      lastspeed = 0xFFFFFFFF;
    }
 }
} 
//////////////////////////////////////////////////////////////////////////

static void setPushFanSpeedPercent(float percent) {
  uint16_t speed = (uint16_t)(percent * pushFan.getMaxSpeed() / 100.0);
  setPushFanSpeed(speed);
}
//////////////////////////////////////////////////////////////////////////

static void setMasterFanSpeed(uint32_t speed) {
  if (followFanType == FOLLOW_PULL_FAN) {
    setPullFanSpeed(speed);
  } else {
    setPushFanSpeed(speed);
  }
}
//////////////////////////////////////////////////////////////////////////

static void setMasterFanSpeedPercent(float percent) {
  if (followFanType == FOLLOW_PULL_FAN) {
    setPullFanSpeedPercent(percent);
  } else {
    setPushFanSpeedPercent(percent);
  }
}
//////////////////////////////////////////////////////////////////////////

static void setSlaveFanSpeed(uint32_t speed) {
  if (followFanType == FOLLOW_PULL_FAN) {
    setPushFanSpeed(speed);
  } else {
    setPullFanSpeed(speed);
  }
}
//////////////////////////////////////////////////////////////////////////

static void setSlaveFanSpeedPercent(float percent) {
  if (followFanType == FOLLOW_PULL_FAN) {
    setPushFanSpeedPercent(percent);
  } else {
    setPullFanSpeedPercent(percent);
  }
}
//////////////////////////////////////////////////////////////////////////

static void setMasterFan(FollowFanType type) {
  if (followFanType != type) {
    followFanType = type;
    pidBalance.setControllerDirection(type == FOLLOW_PULL_FAN ? PidController::DIRECT : PidController::REVERSE);
  }
}
//////////////////////////////////////////////////////////////////////////

static Fan& getMasterFan() {
  return followFanType == FOLLOW_PULL_FAN ? pullFan : pushFan;
}
//////////////////////////////////////////////////////////////////////////

static Fan& getSlaveFan() {
  return followFanType == FOLLOW_PULL_FAN ? pushFan : pullFan;
}
//////////////////////////////////////////////////////////////////////////

static void setOperationMode(OperatingMode mode) {
  if (operatingMode != mode) {
    operatingMode = mode;
    switch (mode) {
      case OM_SPEED_PULL_FAN:
      case OM_FLOW_PULL_FAN:
        setMasterFan(FOLLOW_PULL_FAN);
        break;

      case OM_SPEED_PUSH_FAN:
      case OM_FLOW_PUSH_FAN:
        setMasterFan(FOLLOW_PUSH_FAN);
        break;
    }

  }  
  switch (mode) {
    case OM_SPEED_PULL_FAN:
    case OM_SPEED_PUSH_FAN:
      numberSelector.setRange(0.0, 100.0, 0.1, false, 1); // 0 - 100%
      numberSelector.setValue(lastEncoderOperationValue >= 0.0
        && lastEncoderOperationValue <= 100.0
        ? lastEncoderOperationValue : 10.0);
      setMasterFanSpeedPercent(lastEncoderOperationValue);
      break;

    case OM_FLOW_PULL_FAN:
    case OM_FLOW_PUSH_FAN:
      numberSelector.setRange(10.0, 200.0, 0.1, false, 1); // 0 - 200 (m3/h)
      double setpointPidFlow = lastEncoderOperationValue >= 10.0 
        && lastEncoderOperationValue <= 200.0
        ? lastEncoderOperationValue : 10.0;
      pidFlow.setSetpoint(setpointPidFlow);  
      numberSelector.setValue(setpointPidFlow);
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

static void adjustSensorOffsetVenturiPressure() {
  Serial.println("Adjusting offset venturi pressure sensor."); 
  for (size_t i = 0; i < 200; i++) {
    float differentialPressure;
    float temperature;
    uint16_t error = sdpVenturi.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from flowsensor");
       break;
    }
    calibration.add(differentialPressure);
    if (i % 12 == 0) {
      displayAdjustSensorOffsetsProgress(i * 120 / 200);
    }
    delay(15);
  }

  offsetVenturiPressure = calibration.get();
  saveFloat(KEY_OFFSET_VENTURI_PRESSURE_SENSOR, offsetVenturiPressure);
  Serial.printf("offset venturi %f\n", offsetVenturiPressure);
  delay(500); 
}
//////////////////////////////////////////////////////////////////////////

static void adjustSensorOffsetBalancePressure() {
  Serial.println("Adjusting offset balance pressure sensor."); 
  for (size_t i = 0; i < 200; i++) {
    float differentialPressure;
    float temperature;
    uint16_t error = sdpBalance.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from zeropressuresensor");
       break;
    }
    calibration.add(differentialPressure);
    if (i % 12 == 0) {
      displayAdjustSensorOffsetsProgress(i * 120 / 200);
    }
    delay(15);
  }
  
  offsetBalancePressure = calibration.get();
  saveFloat(KEY_OFFSET_BALANCE_PRESSURE_SENSOR, offsetBalancePressure);
  Serial.printf("offset balance %f\n", offsetBalancePressure);
  delay(500); 
}
//////////////////////////////////////////////////////////////////////////

static void adjustSensorOffsets() {
  displayAdjustSensorOffsets("Venturi");
  adjustSensorOffsetVenturiPressure();
  displayAdjustSensorOffsets("Balance");
  adjustSensorOffsetBalancePressure();
}
//////////////////////////////////////////////////////////////////////////

static void initNextMode(ModeType type) {
  modeType = type;

  switch (type) {
   
    case MT_SELECT:
      numberSelector.setRange(1, 7,  1, true, 0);
      numberSelector.setValue(MT_OPERATION); // sets initial value
      displaySelectMode(MT_OPERATION);
      break;

    case MT_OPERATION:
      setOperationMode(operatingMode);
      displayMeasurements();
      break;

    case MT_SELECT_OPERATION:
      numberSelector.setRange(0, 3, 1, true, 0);
      numberSelector.setValue(operatingMode);
      displaySelectOperationMode(operatingMode);
      break;
    
    case MT_CAL_FLOW:
      numberSelector.setRange(0.9, 1.0, 0.001, false, 3);
      numberSelector.setValue(venturi.getDischargeCoefficient());
      break;

    case MT_TUNE_PID_FLOW:
      pidTuneType = PID_TUNE_NONE;
      numberSelector.setRange(0, 7, 1, true, 0);
      numberSelector.setValue(PID_TUNE_NONE);
      displaySelectTunePID(PID_TUNE_NONE);
      break;

    case MT_TUNE_PID_BALANCE:
      pidTuneType = PID_TUNE_NONE;
      numberSelector.setRange(0, 7, 1, true, 0);
      numberSelector.setValue(PID_TUNE_NONE);
      displaySelectTunePID(PID_TUNE_NONE);
      break;

    case MT_ADJUST_OFFSETS:
      adjustSensorOffsets();
      initNextMode(MT_OPERATION);
      break;

    case MT_SET_VENTURI_CONSTANTS:
      venturiConstantsType = VENTURI_CONSTANTS_SET_NONE;
      numberSelector.setRange(0, 3, 1, true, 0);
      numberSelector.setValue(VENTURI_CONSTANTS_SET_NONE);
      displaySelectVenturiConstants(VENTURI_CONSTANTS_SET_NONE);
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

static void on_button_short_click() {
  // pidSetpoint = compensationFactorB = numberSelector.getValue();
  // compensationFactorA = flow.get();
  switch (modeType) {
    case MT_SELECT:
      initNextMode((ModeType)numberSelector.getValue()); 
      break;
    case MT_OPERATION:
      initNextMode(MT_SELECT_OPERATION);
      break;  
    case MT_SELECT_OPERATION:
      setOperationMode((OperatingMode)numberSelector.getValue());
      initNextMode(MT_OPERATION);
      break;
    case MT_CAL_FLOW:
      venturi.setDischargeCoefficient(numberSelector.getValue());
      saveFloat(KEY_VENTURI_CD, venturi.getDischargeCoefficient());
      initNextMode(MT_OPERATION);
      break;
    case MT_TUNE_PID_FLOW:
      switch (pidTuneType) {
        case PID_TUNE_NONE:
          switch ((PidTuneType)(uint8_t)numberSelector.getValue()) {
            case PID_TUNE_NONE:
              initNextMode(MT_SELECT);
              break;
            case PID_TUNE_P:
              pidTuneType = PID_TUNE_P;
              numberSelector.setRange(PID_KP_MIN, PID_KP_MAX, PID_KP_STEP, false, 1);
              numberSelector.setValue(KpFlow);
              displayMeasurements();
              break;
            case PID_TUNE_I:
              pidTuneType = PID_TUNE_I;
              numberSelector.setRange(PID_KI_MIN, PID_KI_MAX, PID_KI_STEP, false, PID_KI_DECIMALS);
              numberSelector.setValue(KiFlow);
              displayMeasurements();
              break;
            case PID_TUNE_D:
              pidTuneType = PID_TUNE_D;
              numberSelector.setRange(PID_KD_MIN, PID_KD_MAX, PID_KD_STEP, false, PID_KD_DECIMALS);
              numberSelector.setValue(KdFlow);
              displayMeasurements();
              break;
            case PID_TUNE_ALPHA:
              pidTuneType = PID_TUNE_ALPHA;
              numberSelector.setRange(ALPHA_MIN, ALPHA_MAX, ALPHA_STEP, false, ALPHA_DECIMALS);
              numberSelector.setValue(pidFlow.getAlpha());
              displayMeasurements();
              break;
            case PID_TUNE_BETA:
              pidTuneType = PID_TUNE_BETA;
              numberSelector.setRange(BETA_MIN, BETA_MAX, BETA_STEP, false, BETA_DECIMALS);
              numberSelector.setValue(pidFlow.getBeta());
              displayMeasurements();
              break;
            case PID_TUNE_DAMPING:
              pidTuneType = PID_TUNE_DAMPING;
              numberSelector.setRange(DAMPING_MIN, DAMPING_MAX, DAMPING_STEP, false, DAMPING_DECIMALS);
              numberSelector.setValue(pidFlow.getDamping());
              displayMeasurements();
              break;
            case PID_TUNE_TREND_LIMIT:  
              pidTuneType = PID_TUNE_TREND_LIMIT;
              numberSelector.setRange(TREND_LIMIT_MIN, TREND_LIMIT_MAX, TREND_LIMIT_STEP, false, TREND_LIMIT_DECIMALS);
              numberSelector.setValue(pidFlow.getTrendLimit());
              displayMeasurements();
              break;
          }
          break;
        case PID_TUNE_P:
          saveFloat(KEY_KP_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;
        case PID_TUNE_I:
          saveFloat(KEY_KI_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;  
        case PID_TUNE_D:
          saveFloat(KEY_KD_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;
        case PID_TUNE_ALPHA:
         saveFloat(KEY_ALPHA_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;
        case PID_TUNE_BETA:
          saveFloat(KEY_BETA_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;
        case PID_TUNE_DAMPING:
          saveFloat(KEY_DAMPING_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;
        case PID_TUNE_TREND_LIMIT:
          saveFloat(KEY_TREND_LIMIT_FLOW, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_FLOW);
          break;
      }
      break;
  
    case MT_TUNE_PID_BALANCE:
      switch (pidTuneType) {
        case PID_TUNE_NONE:
          switch((PidTuneType)(uint8_t)numberSelector.getValue()) {
            case PID_TUNE_NONE:
              initNextMode(MT_SELECT);
              break;
            case PID_TUNE_P:
              pidTuneType = PID_TUNE_P;
              numberSelector.setRange(PID_KP_MIN, PID_KP_MAX, PID_KP_STEP, false, PID_KP_DECIMALS);
              numberSelector.setValue(KpBalance);
              displayMeasurements();
              break;
            case PID_TUNE_I:
              pidTuneType = PID_TUNE_I;
              numberSelector.setRange(PID_KI_MIN, PID_KI_MAX, PID_KI_STEP, false, PID_KI_DECIMALS);
              numberSelector.setValue(KiBalance);
              displayMeasurements();
              break;
            case PID_TUNE_D:
              pidTuneType = PID_TUNE_D;
              numberSelector.setRange(PID_KD_MIN, PID_KD_MAX, PID_KD_STEP, false, PID_KD_DECIMALS);
              numberSelector.setValue(KdBalance);
              displayMeasurements();
              break;
            case PID_TUNE_ALPHA:
              pidTuneType = PID_TUNE_ALPHA;
              numberSelector.setRange(ALPHA_MIN, ALPHA_MAX, ALPHA_STEP, false, ALPHA_DECIMALS);
              numberSelector.setValue(pidBalance.getAlpha());
              displayMeasurements();
              break;
            case PID_TUNE_BETA:
              pidTuneType = PID_TUNE_BETA;
              numberSelector.setRange(BETA_MIN, BETA_MAX, BETA_STEP, false, BETA_DECIMALS);
              numberSelector.setValue(pidBalance.getBeta());
              displayMeasurements();
              break;
            case PID_TUNE_DAMPING:
              pidTuneType = PID_TUNE_DAMPING;
              numberSelector.setRange(DAMPING_MIN, DAMPING_MAX, DAMPING_STEP, false, DAMPING_DECIMALS);
              numberSelector.setValue(pidBalance.getDamping());
              displayMeasurements();
              break;
            case PID_TUNE_TREND_LIMIT:
              pidTuneType = PID_TUNE_TREND_LIMIT;
              numberSelector.setRange(TREND_LIMIT_MIN, TREND_LIMIT_MAX, TREND_LIMIT_STEP, false, TREND_LIMIT_DECIMALS);
              numberSelector.setValue(pidBalance.getTrendLimit());
              displayMeasurements();
              break; 
          }
          break;
        case PID_TUNE_P:
          saveFloat(KEY_KP_BALANCE, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break;
        case PID_TUNE_I:
          saveFloat(KEY_KI_BALANCE, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break; 
        case PID_TUNE_D:
          saveFloat(KEY_KD_BALANCE, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break;
        case PID_TUNE_ALPHA:
          saveFloat(KEY_BALANCE_ALPHA, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break;
        case PID_TUNE_BETA:
          saveFloat(KEY_BALANCE_BETA, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break;
        case PID_TUNE_DAMPING:
          saveFloat(KEY_BALANCE_DAMPING, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break;
        case PID_TUNE_TREND_LIMIT:
          saveFloat(KEY_BALANCE_TREND_LIMIT, numberSelector.getValue());
          initNextMode(MT_TUNE_PID_BALANCE);
          break;
      }
      break;
  
    case MT_SET_VENTURI_CONSTANTS:
      switch (venturiConstantsType) {
        case VENTURI_CONSTANTS_SET_NONE:
          switch ((VenturiConstantsType)(uint8_t)numberSelector.getValue()) {
            case VENTURI_CONSTANTS_SET_NONE:
              initNextMode(MT_SELECT);
              break;
            case VENTURI_CONSTANTS_SET_DIA_INLET:
              venturiConstantsType = VENTURI_CONSTANTS_SET_DIA_INLET;
              numberSelector.setRange(0.01, 0.3, 0.001, false, 3);
              numberSelector.setValue(venturi.getInletDiameter());
              displayMeasurements();
              break;
            case VENTURI_CONSTANTS_SET_DIA_THROAT:
              venturiConstantsType = VENTURI_CONSTANTS_SET_DIA_THROAT;
              numberSelector.setRange(0.01, 0.25, 0.001, false, 3);
              numberSelector.setValue(venturi.getThroatDiameter());
              displayMeasurements();
             break;
            case VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR:
              venturiConstantsType = VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR;
              numberSelector.setRange(0.01, 1.0, 0.01, false, 2);
              numberSelector.setValue(venturi.getSmoothedFlowFactor());
              displayMeasurements();
              break;
          }
          break;
        case VENTURI_CONSTANTS_SET_DIA_INLET:
          venturi.setInletDiameter(numberSelector.getValue());
          saveFloat(KEY_VENTURI_INLET_DIAMETER, venturi.getInletDiameter());
          initNextMode(MT_SET_VENTURI_CONSTANTS);
          break;
        case VENTURI_CONSTANTS_SET_DIA_THROAT:
          venturi.setThroatDiameter(numberSelector.getValue());
          saveFloat(KEY_VENTURI_THROAT_DIAMETER, venturi.getThroatDiameter());
          initNextMode(MT_SET_VENTURI_CONSTANTS);
          break;
        case VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR:
          venturi.setSmoothedFlowFactor(numberSelector.getValue());
          saveFloat(KEY_VENTURI_SMOOTHING_FACTOR, venturi.getSmoothedFlowFactor());
          initNextMode(MT_SET_VENTURI_CONSTANTS);
          break;
      }
      break;  
  }
}
//////////////////////////////////////////////////////////////////////////

static void on_button_long_click() {
  
  switch (modeType) {
    case MT_OPERATION:
      initNextMode(MT_SELECT);
      break;
    default:
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

static void handle_rotary_button() {
  static unsigned long lastTimeButtonDown = 0;
  static bool wasButtonDown = false;

  bool isEncoderButtonDown = rotaryEncoder->isEncoderButtonDown();
  
  if (isEncoderButtonDown) {
    if (!wasButtonDown) {
      wasButtonDown = true;
      // start measuring
      lastTimeButtonDown = millis();
    }
  // button is up
  } else if (wasButtonDown) {
    wasButtonDown = false;
    // click happened, lets see if it was short click, long click or just too short
    if (millis() - lastTimeButtonDown >= longPressAfterMiliseconds) {
      on_button_long_click();
    } else if (millis() - lastTimeButtonDown >= shortPressAfterMiliseconds) {
      on_button_short_click();
    }
  } 
}
//////////////////////////////////////////////////////////////////////////

static void loopRotaryEncoder() {
  
  if (rotaryEncoder->encoderChanged()) {
    switch (modeType) {
      
      case MT_SELECT:
        displaySelectMode((ModeType)numberSelector.getValue());
        break;

      case MT_OPERATION:
        lastEncoderOperationValue = numberSelector.getValue();
        switch (operatingMode) {
          case OM_SPEED_PULL_FAN:
          case OM_SPEED_PUSH_FAN:
            outputMasterFan = lastEncoderOperationValue;
            setMasterFanSpeedPercent(outputMasterFan);
            displayTextNumber("s %.1f%%", outputMasterFan);
            break;
          case OM_FLOW_PULL_FAN:
          case OM_FLOW_PUSH_FAN:
            double setpointPidFlow = lastEncoderOperationValue;
            pidFlow.setSetpoint(setpointPidFlow);
            displayTextNumber("f %.1fm3/s", setpointPidFlow);
            break;
        }
        break;  

      case MT_SELECT_OPERATION:
        displaySelectOperationMode((OperatingMode)numberSelector.getValue());
        break;
      
      case MT_CAL_FLOW:
        venturi.setDischargeCoefficient(numberSelector.getValue());
        displayTextNumber("Cd %.3f", venturi.getDischargeCoefficient());
        break;

      case MT_TUNE_PID_FLOW:
        switch (pidTuneType) {
          case PID_TUNE_NONE:
            displaySelectTunePID((PidTuneType)(uint8_t)numberSelector.getValue());
            break;
          case PID_TUNE_P:
            KpFlow = numberSelector.getValue();
            pidFlow.setTunings(KpFlow, KiFlow, KdFlow);
            displayTextNumber("Kp flow: %.2f", KpFlow);
            break;
          case PID_TUNE_I:
            KiFlow = numberSelector.getValue();
            pidFlow.setTunings(KpFlow, KiFlow, KdFlow);
            displayTextNumber("Ki flow: %.2f", KiFlow);
            break;
          case PID_TUNE_D:
            KdFlow = numberSelector.getValue();
            pidFlow.setTunings(KpFlow, KiFlow, KdFlow);
            displayTextNumber("Kd flow: %.2f", KdFlow);
            break;

          case PID_TUNE_ALPHA:
            flowAlpha = numberSelector.getValue();
            pidFlow.setAlpha(flowAlpha);
            displayTextNumber("alpha flow: %.2f", flowAlpha);
            break;
          
          case PID_TUNE_BETA:
            flowBeta = numberSelector.getValue();
            pidFlow.setBeta(flowBeta);
            displayTextNumber("beta flow: %.2f", flowBeta);
            break;
            
          case PID_TUNE_DAMPING:
            flowDamping = numberSelector.getValue();
            pidFlow.setDamping(flowDamping);
            displayTextNumber("damping flow: %.2f", flowDamping);
            break;
            
          case PID_TUNE_TREND_LIMIT:
            flowTrendLimit = numberSelector.getValue();
            pidFlow.setTrendLimit(flowTrendLimit);
            displayTextNumber("trendlimit flow: %.1f", flowTrendLimit);
            break;
        } 
        break;

      case MT_TUNE_PID_BALANCE:
        switch (pidTuneType) {
          case PID_TUNE_NONE:
            displaySelectTunePID((PidTuneType)(uint8_t)numberSelector.getValue());
            break;
          case PID_TUNE_P:
            KpBalance = numberSelector.getValue();
            pidBalance.setTunings(KpBalance, KiBalance, KdBalance);
            displayTextNumber("Kp balance: %.2f", KpBalance);
            break;
          case PID_TUNE_I:
            KiBalance = numberSelector.getValue();
            pidBalance.setTunings(KpBalance, KiBalance, KdBalance);
            displayTextNumber("Ki balance: %.2f", KiBalance);
            break;
          case PID_TUNE_D:
            KdBalance = numberSelector.getValue();
            pidBalance.setTunings(KpBalance, KiBalance, KdBalance);
            displayTextNumber("Kd balance: %.2f", KdBalance);
            break;
          case PID_TUNE_ALPHA:
            balanceAlpha = numberSelector.getValue();
            pidBalance.setAlpha(balanceAlpha);
            displayTextNumber("alpha balance: %.2f", balanceAlpha);
            break;
          case PID_TUNE_BETA:
            balanceBeta = numberSelector.getValue();
            pidBalance.setBeta(balanceBeta);
            displayTextNumber("beta balance: %.2f", balanceBeta);
            break;
          case PID_TUNE_DAMPING:
            balanceDamping = numberSelector.getValue();
            pidBalance.setDamping(balanceDamping);
            displayTextNumber("damping bal.: %.2f", balanceDamping);
            break;
          case PID_TUNE_TREND_LIMIT:
            balanceTrendLimit = numberSelector.getValue();
            pidBalance.setTrendLimit(balanceTrendLimit);
            displayTextNumber("trendlimit bal.: %.1f", balanceTrendLimit);
            break;
        } 
        break;
      
      case MT_ADJUST_OFFSETS:
        // do nothing on encoder changes, only show progress during calibration
        break;

      case MT_SET_VENTURI_CONSTANTS:
        switch (venturiConstantsType) {
          case VENTURI_CONSTANTS_SET_NONE:
            displaySelectVenturiConstants((VenturiConstantsType)(uint8_t)numberSelector.getValue());
            break;
          case VENTURI_CONSTANTS_SET_DIA_INLET:
            displayTextNumber("Inlet dia: %.3fm", numberSelector.getValue());
            break;
          case VENTURI_CONSTANTS_SET_DIA_THROAT:
            displayTextNumber("Throat dia: %.3fm", numberSelector.getValue());
            break;
          case VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR:
            venturi.setSmoothedFlowFactor(numberSelector.getValue());
            displayTextNumber("Smooth factor: %.2f", venturi.getSmoothedFlowFactor());
            break;
        }
        break;  
    }
  } 
  handle_rotary_button();
}
//////////////////////////////////////////////////////////////////////////

static void initBME280(void) {
  
  if (bme280.begin(I2C_ADDRESS_BME280, &I2C_A)) {

    //weather monitoring scenario
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
                    
    drawString(0, 12, "Setup BME280 ");                
    delay(1000);

  } else {
    char message[32];
    drawString(5, 10, "No valid BME280");
    delay(5000);
    snprintf(message, sizeof(message), "SensorID: 0x%02X", bme280.sensorID());
    drawString(5, 10, message);
    delay(5000);
  }
}
//////////////////////////////////////////////////////////////////////////

static void initSDP(SensirionI2CSdp& sdp, TwoWire& wire) {
  drawString(0, 0, "Setup SDP810 ");                
  delay(1000);
  
  sdp.begin(wire, SDP8XX_I2C_ADDRESS_0);

  uint16_t error;
  char errorMessage[256];

  uint32_t productNumber;
  uint8_t serialNumber[8];
  uint8_t serialNumberSize = 8;

  sdp.stopContinuousMeasurement();

  error = sdp.readProductIdentifier(productNumber, serialNumber,
    serialNumberSize);
  if (error) {
    drawString(0, 12, "Error ");                
    Serial.print("Error: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    delay(5000);
  } else {
    Serial.print("ProductNumber:");
    Serial.print(productNumber);
    Serial.print("\t");
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
            Serial.print(serialNumber[i], HEX);
     }
     Serial.println();
  }

  //error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  error = sdp.startContinuousMeasurementWithDiffPressureTComp();

  if (error) {
    Serial.print(
      "Error trying to execute "
      "startContinuousMeasurementWithDiffPressureTComp(): "
    );
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    delay(500);
  }
}
//////////////////////////////////////////////////////////////////////////

static const char* getOperatingModeText(OperatingMode mode) {
  switch (mode) {
    case OM_SPEED_PULL_FAN:
      return "SPEED_PULL";
    case OM_SPEED_PUSH_FAN:
      return "SPEED_PUSH";
    case OM_FLOW_PULL_FAN:
      return "FLOW_PULL";
    case OM_FLOW_PUSH_FAN:
      return "FLOW_PUSH";
  }
      return "          ";
}
//////////////////////////////////////////////////////////////////////////

static void setOledContrast(uint8_t contrast, uint8_t precharge, uint8_t comdetect) {
  display.oled_command(SH110X_SETPRECHARGE); //0xD9
  display.oled_command(precharge); //0xF1 default, to lower the contrast, put 1-1F
  display.oled_command(SH110X_SETCONTRAST);
  display.oled_command(contrast);  // 0-255
  display.oled_command(SH110X_SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
  display.oled_command(comdetect); //0x40 default, to lower the contrast, put 0
  display.oled_command(SH110X_DISPLAYALLON_RESUME);
  display.oled_command(SH110X_NORMALDISPLAY);
  display.oled_command(SH110X_DISPLAYON);
}
//////////////////////////////////////////////////////////////////////////

static void setOledBrightness(uint8_t brightness) {
  uint8_t contrast = brightness;
  if (brightness < 128) {
    // Magic values to get a smooth/ step-free transition
    contrast = brightness * 1.171;
  } else {
    contrast = brightness * 1.171 - 43;
  }

  uint8_t precharge = 241;
  if (brightness == 0) {
    precharge = 0;
  }
  uint8_t comdetect = brightness / 8;

  setOledContrast(contrast, precharge, comdetect);
}
//////////////////////////////////////////////////////////////////////////

static void initDisplay(void) {
  display.begin(0x3C, true);
  setOledBrightness(0x60);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setTextSize(1);          // schaalfactor
  display.setCursor(0,24);
  display.print("Display ready");
  display.display();
  delay(1000);
}
//////////////////////////////////////////////////////////////////////////

static void printAlignCenter(const char *text, int16_t x, int16_t y) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor(x-(w>>1), y);
  display.print(text);
}
//////////////////////////////////////////////////////////////////////////

static void printAlignRight(const char *text, int16_t x, int16_t y) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor(x-w, y);
  display.print(text);
}
//////////////////////////////////////////////////////////////////////////

// Draws a string at the given location
static void drawString(int16_t x, int16_t y, const String &text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(x, y);
  display.print(text);
  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displayTextNumber(const char *txt, float number) {
  // display setpoint zero pressure
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("          ");
  readPressureSensors();
  display.setCursor(0, 0);
  display.printf(txt, number);
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayAdjustSensorOffsets(const char *sensor) {
  display.clearDisplay();
  readPressureSensors();
  display.setTextSize(1);
  printAlignCenter("Adjusting Offsets", 64, 0);
  readPressureSensors();
  display.setCursor(0, 28);
  display.printf("Offset %s...", sensor);
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayAdjustSensorOffsetsProgress(int16_t progress) {
  display.fillRect(0, 40, progress, 10, SH110X_WHITE);
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayMeasurements() {
  char message[32];
  
  display.clearDisplay();

  readPressureSensors();
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  if (modeType == MT_OPERATION) {
    switch (operatingMode) {
      case OM_SPEED_PULL_FAN:
      case OM_SPEED_PUSH_FAN:
        // display setpoint speed fan
        display.printf("s %.1f%%", numberSelector.getValue());
        break;
      case OM_FLOW_PULL_FAN:
      case OM_FLOW_PUSH_FAN:
        // display setpoint flow
        display.printf("f %.1fm3/h", numberSelector.getValue());
        break;
    }
    printAlignRight(getOperatingModeText(operatingMode), 127, 0);
  } else if (modeType == MT_CAL_FLOW) {
    // display discharge coefficient venturi
    display.printf("Cd: %.3f", numberSelector.getValue());
  } else if (modeType == MT_TUNE_PID_FLOW) {
    if (pidTuneType == PID_TUNE_P) {
      // display proportional gain PID controller
      display.printf("Kp flow: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_I) {
      // display integral gain PID controller
      display.printf("Ki flow: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_D) {
      // display derivative gain PID controller
      display.printf("Kd flow: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_ALPHA) {
      // display alpha PID controller
      display.printf("alpha flow: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_BETA) {
      // display beta PID controller
      display.printf("beta flow: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_DAMPING) {
      // display damping PID controller
      display.printf("damping flow: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_TREND_LIMIT) {
      // display trend limit PID controller
      display.printf("trendlimit flow: %.1f", numberSelector.getValue());
    }
  
  } else if (modeType == MT_TUNE_PID_BALANCE) {
    if (pidTuneType == PID_TUNE_P) {
      // display proportional gain PID controller
      display.printf("Kp balance: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_I) {
      // display integral gain PID controller
      display.printf("Ki balance: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_D) {
      // display derivative gain PID controller
      display.printf("Kd balance: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_ALPHA) {
      // display alpha PID controller
      display.printf("alpha balance: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_BETA) {
      // display beta PID controller
      display.printf("beta balance: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_DAMPING) {
      // display damping PID controller
      display.printf("damping bal.: %.2f", numberSelector.getValue());
    } else if (pidTuneType == PID_TUNE_TREND_LIMIT) {
      // display trend limit PID controller
      display.printf("trendlimit bal.: %.1f", numberSelector.getValue());
    }

  } else if (modeType == MT_SET_VENTURI_CONSTANTS) {
    if (venturiConstantsType == VENTURI_CONSTANTS_SET_DIA_INLET) {
      // display inlet diameter venturi
      display.printf("Inlet dia: %.3fm", numberSelector.getValue());
    } else if (venturiConstantsType == VENTURI_CONSTANTS_SET_DIA_THROAT) {
      // display throat diameter venturi
      display.printf("Throat dia: %.3fm", numberSelector.getValue());
    } else if (venturiConstantsType == VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR) {
      // display smoothing factor venturi
      display.printf("Smooth factor: %.2f", numberSelector.getValue());
    } 
  }
  readPressureSensors();

  // display flow
  snprintf(message, sizeof(message),"%.1f", venturi.getSmoothedFlow());
  display.setFont(); // standaard font
  display.setTextSize(2);
  printAlignCenter(message, 63, 18);
  readPressureSensors();
  display.setTextSize(1);
  display.setCursor(0, 21);
  display.print("Flow");
  readPressureSensors();
  printAlignRight("m3/h", 127, 21);
  readPressureSensors();

  // display balance pressure
  display.setCursor(0, 41);
  display.printf("Pb %.2f Pa", balancePressure.get());
  readPressureSensors();

  // display temperature
  snprintf(message, sizeof(message),"%.1f C", temperatureAmbient.get());
  printAlignRight(message, 127, 41);
  readPressureSensors();

  // display absolute pressure
  display.setCursor(0, 57);
  display.printf("%.1f hPa", pressureAbsolute.get() / 100.0);
  readPressureSensors();

  // display humidity
  snprintf(message, sizeof(message),"%.1f %%RH", humidityAmbient.get());
  printAlignRight(message, 127, 57);
  readPressureSensors();

  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectMode(ModeType mtype) {
  
  display.clearDisplay();
  display.setTextSize(1);

  if (mtype == MT_OPERATION)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Back");
  if (mtype == MT_OPERATION) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_SELECT_OPERATION)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Select operating mode");
  if (mtype == MT_SELECT_OPERATION) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_CAL_FLOW) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Calibrate flow");
  if (mtype == MT_CAL_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_TUNE_PID_FLOW) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Tune PID flow");
  if (mtype == MT_TUNE_PID_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_TUNE_PID_BALANCE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 37);
  display.print("Tune PID balance");
  if (mtype == MT_TUNE_PID_BALANCE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_ADJUST_OFFSETS) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 46);
  display.print("Adjust sensor offsets");
  if (mtype == MT_ADJUST_OFFSETS) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_SET_VENTURI_CONSTANTS) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 55);
  display.print("Set venturi constants");
  if (mtype == MT_SET_VENTURI_CONSTANTS) display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectOperationMode(OperatingMode mode) {
  display.clearDisplay();
  display.setTextSize(1);

  if (mode == OM_SPEED_PULL_FAN) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Speed Pull Fan");
  if (mode == OM_SPEED_PULL_FAN) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mode == OM_SPEED_PUSH_FAN)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 8);
  display.print("Speed Push Fan");
  if (mode == OM_SPEED_PUSH_FAN) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mode == OM_FLOW_PULL_FAN) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 16);
  display.print("Flow Pull Fan");
  if (mode == OM_FLOW_PULL_FAN) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mode == OM_FLOW_PUSH_FAN) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 24);
  display.print("Flow Push Fan");
  if (mode == OM_FLOW_PUSH_FAN) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectTunePID(PidTuneType type) {
  display.clearDisplay();
  display.setTextSize(1);

  if (type == PID_TUNE_NONE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Back");
  if (type == PID_TUNE_NONE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_P) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Tune Kp");
  if (type == PID_TUNE_P) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_I)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Tune Ki");
  if (type == PID_TUNE_I) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_D) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Tune Kd");
  if (type == PID_TUNE_D) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_ALPHA) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 37);
  display.print("Tune alpha");
  if (type == PID_TUNE_ALPHA) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_BETA) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(64, 37);
  display.print("Tune beta");
  if (type == PID_TUNE_BETA) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_DAMPING) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 46);
  display.print("Tune damping");
  if (type == PID_TUNE_DAMPING) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_TREND_LIMIT) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 55);
  display.print("Tune trend limit");
  if (type == PID_TUNE_TREND_LIMIT) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectVenturiConstants(VenturiConstantsType type) {
  
  display.clearDisplay();
  display.setTextSize(1);

  if (type == VENTURI_CONSTANTS_SET_NONE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Back");
  if (type == VENTURI_CONSTANTS_SET_NONE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == VENTURI_CONSTANTS_SET_DIA_INLET) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Set Inlet Dia");
  if (type == VENTURI_CONSTANTS_SET_DIA_INLET) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == VENTURI_CONSTANTS_SET_DIA_THROAT)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Set Throat Dia");
  if (type == VENTURI_CONSTANTS_SET_DIA_THROAT) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Set Smooth Factor");
  if (type == VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();
}
//////////////////////////////////////////////////////////////////////////
