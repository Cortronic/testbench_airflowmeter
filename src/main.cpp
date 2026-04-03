#include <Arduino.h>

#include <driver/ledc.h>
#include <driver/dac.h>
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
#include <PID_v1.h>
#include "main.h"

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

// tachometer
volatile uint32_t tachoFanPulseCycles = 300;
volatile uint32_t tachoPushFanPulseCount = 0;
volatile uint32_t tachoPullFanPulseCount = 0;
volatile uint32_t tachoPullFanRPM = 0;
volatile uint32_t tachoPushFanRPM = 0;

// Venturi Constants
VenturiConstants venturi;

//paramaters for button
const unsigned long shortPressAfterMiliseconds = 50;   // how long short press shoud be.
                                                       // Do not set too low to avoid bouncing (false press events).
const unsigned long longPressAfterMiliseconds = 1000;  // how long long press shoud be.

AiEsp32RotaryEncoder *rotaryEncoder = new AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS, false);
AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();

Preferences preferences;

Smoothed<float> flow;
Smoothed<float> venturiPressure;
Smoothed<float> balancePressure;
Smoothed<float> calibration;
Smoothed<float> pressureAbsolute;
Smoothed<float> temperatureAmbient;
Smoothed<float> humidityAmbient;

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
OperationMode operationMode = OM_SPEED_PULL_FAN;
SetDiameterType setDiameterType = SET_DIA_NONE;

// setup PID controllers
// Define Variables we'll be connecting to
double setpointPidFlow, inputPidFlow, outputPidFlow;
// Specify the links and initial tuning parameters
double KpFlow=6, KiFlow=3, KdFlow=0;
PID pidFlow(&inputPidFlow, &outputPidFlow, &setpointPidFlow, KpFlow, KiFlow, KdFlow, DIRECT);
double setpointPidBalance, inputPidBalance, outputPidBalance;
double KpBalance=6, KiBalance=3, KdBalance=0;
PID pidBalance(&inputPidBalance, &outputPidBalance, &setpointPidBalance, KpBalance, KiBalance, KdBalance, DIRECT);


float rho = 1.2; // densitity of air at 1013,25 hPa, 20 °C, and 60 %Rh (Kg/m3)

float offsetVenturiPressure = 0.0;
float offsetBalancePressure = 0.0;
float lastEncoderOperationValue = 0.0;

static void  loadPreferences();
static void  setupRotaryEncoder();
static void  setPullFanSpeed(uint32_t speed);
static void  setPushFanSpeed(uint32_t speed);
static void  setMasterFan(FollowFanType type);
static void  setMasterFanSpeed(uint32_t speed);
static void  setSlaveFanSpeed(uint32_t speed);
static void  setOperationMode(OperationMode mode);
static void  setupTachometers();
static void  setupTimer0();
static void  initDisplay(void);
static void  readPressureSensors();
static void  loopRotaryEncoder();
static void  displayMeasurements();
static void  displaySelectMode(ModeType);
static void  displaySelectOperationMode(OperationMode mode);
static void  displayTextNumber(const char *txt, float);
static void  displayAdjustSensorOffsets(const char *sensor);
static void  displayAdjustSensorOffsetsProgress(int16_t progress);
static void  displaySelectTunePID(PidTuneType type);
static void  displaySelectSetDiameter(SetDiameterType type);
static void  initBME280();
static void  initSDP(SensirionI2CSdp&, TwoWire&);
static float calculateFlowCompensated(float dP);
static void  drawString(int16_t x, int16_t y, const String &text);
static float getFlow(float dP);
static void  setRho(float tempC, float absPressurePa, float humidityPct);
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
  ledcSetup(PWM_PULL_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PULL_PWM_PIN, PWM_PULL_FAN_CHAN);
  ledcSetup(PWM_PUSH_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PUSH_PWM_PIN, PWM_PUSH_FAN_CHAN);
 
  // set initial fan speeds to zero
  setMasterFanSpeed(0);
  setSlaveFanSpeed(0);
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
  flow.begin(SMOOTHED_AVERAGE, 50);
  balancePressure.begin(SMOOTHED_EXPONENTIAL, 2);
  venturiPressure.begin(SMOOTHED_AVERAGE, 100);
  calibration.begin(SMOOTHED_AVERAGE, 200);
  pressureAbsolute.begin(SMOOTHED_AVERAGE, 40);
  humidityAmbient.begin(SMOOTHED_AVERAGE, 40);
  temperatureAmbient.begin(SMOOTHED_AVERAGE, 40);
  delay(500);

  // initialize the Flow PID variables
  pidFlow.SetTunings(KpFlow, KiFlow, KdFlow);
  setpointPidFlow = 0.0;
  inputPidFlow = 0.0;
  Serial.println("turn the PID Flow on.");
  pidFlow.SetMode(AUTOMATIC);
  pidFlow.SetOutputLimits(0, 1023);
  pidFlow.SetControllerDirection(DIRECT);
  delay(500);

  // initialize the Balance PID variables
  pidBalance.SetTunings(KpBalance, KiBalance, KdBalance);
  setpointPidBalance = 0.0;
  inputPidBalance = 0.0;
  Serial.println("turn the PID Balance on.");
  pidBalance.SetMode(AUTOMATIC);
  pidBalance.SetOutputLimits(0, 1023);
  pidBalance.SetControllerDirection(DIRECT);
  delay(500);

  Serial.print("Setup timer interrupt\n");
  setupTimer0();
  delay(500);

  bme280.takeForcedMeasurement();
  setRho(bme280.readTemperature(), bme280.readPressure(), bme280.readHumidity());
  
  Serial.print("\n Setup done...\n\n");
  delay(500);
}
//////////////////////////////////////////////////////////////////////////

void loop() {
  static uint32_t loopcnt = 0;
  static uint32_t last_millis = 0;
  uint32_t ms = millis();
  
  readPressureSensors();

  loopRotaryEncoder();

  // every 100ms
  if (ms >= last_millis + 100) {
    last_millis = ms;

    readPressureSensors();

    flow.add(getFlow(venturiPressure.get()));

    // Compute PID for balance pressure
    inputPidBalance = balancePressure.get();    
    pidBalance.Compute();
    setSlaveFanSpeed(outputPidBalance); // Apply PWM to fan

    if (modeType != MT_ADJUST_OFFSETS) {
      if (operationMode == OM_FLOW_PULL_FAN || operationMode == OM_FLOW_PUSH_FAN) {
        inputPidFlow = flow.get();
        pidFlow.Compute();
        setMasterFanSpeed(outputPidFlow);
      }
    }
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

      Serial.printf("Flow pressure: %.3f Pa\n", venturiPressure.get());
      Serial.printf("PWM fan dutycycle: %.1f%%\n", outputPidFlow / 10.23);
    }

    // every minute
    if (loopcnt % 600 == 0) {
       setRho(temperatureAmbient.get(), pressureAbsolute.get(), humidityAmbient.get());
    }
    
    // every 2 seconds
    if (loopcnt % 20 == 0) {
      if (modeType == MT_OPERATION 
          || modeType == MT_CAL_FLOW
          || pidTuneType == PID_TUNE_P
          || pidTuneType == PID_TUNE_I
          || pidTuneType == PID_TUNE_D) {
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
  pinMode(FAN_PULL_TACHO_PIN, INPUT_PULLUP); // GPIO 12
  attachInterrupt(digitalPinToInterrupt(FAN_PULL_TACHO_PIN), tachoPullFanPulseCount_ISR, FALLING);
  pinMode(FAN_PUSH_TACHO_PIN, INPUT_PULLUP); // GPIO 13
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

  venturi.inletDiameter = preferences.getFloat(KEY_VENTURI_INLET_DIAMETER, 0.116);
  venturi.throatDiameter = preferences.getFloat(KEY_VENTURI_THROAT_DIAMETER, 0.087);
  venturi.areaInlet = M_PI * pow(venturi.inletDiameter / 2.0, 2);
  venturi.areaThroat = M_PI * pow(venturi.throatDiameter / 2.0, 2);
  venturi.betaRatio = venturi.throatDiameter / venturi.inletDiameter;
  venturi.betaCoefficient = 1.0 - pow(venturi.betaRatio, 4);
  venturi.dischargeCoefficient = preferences.getFloat(KEY_VENTURI_CD, 0.975);

  offsetBalancePressure = preferences.getFloat(KEY_OFFSET_BALANCE_PRESSURE_SENSOR, 0.0);
  offsetVenturiPressure = preferences.getFloat(KEY_OFFSET_VENTURI_PRESSURE_SENSOR, 0.0);
  KpFlow =  preferences.getFloat(KEY_KP_FLOW, KpFlow);
  KiFlow =  preferences.getFloat(KEY_KI_FLOW, KiFlow);
  KdFlow =  preferences.getFloat(KEY_KD_FLOW, KdFlow);
  KpBalance = preferences.getFloat(KEY_KP_BALANCE, KpBalance);
  KiBalance = preferences.getFloat(KEY_KI_BALANCE, KiBalance);
  KdBalance = preferences.getFloat(KEY_KD_BALANCE, KdBalance);

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
    ledcWrite(PWM_PULL_FAN_CHAN, speed);
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

static void setPushFanSpeed(uint32_t speed) {
  static uint32_t lastspeed = 0xFFFFFFFF;

  if (lastspeed != speed) {
    ledcWrite(PWM_PUSH_FAN_CHAN, speed);
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

static void setMasterFanSpeed(uint32_t speed) {
  if (followFanType == FOLLOW_PULL_FAN) {
    setPullFanSpeed(speed);
  } else {
    setPushFanSpeed(speed);
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

static void  setMasterFan(FollowFanType type) {
  if (followFanType != type) {
    followFanType = type;
    pidBalance.SetControllerDirection(type == FOLLOW_PULL_FAN ? DIRECT : REVERSE);
  }
}
//////////////////////////////////////////////////////////////////////////

static void setOperationMode(OperationMode mode) {
  if (operationMode != mode) {
    operationMode = mode;
    switch (mode) {
      case OM_SPEED_PULL_FAN:
      case OM_FLOW_PULL_FAN:
      case OM_POWER_PULL_FAN:
        setMasterFan(FOLLOW_PULL_FAN);
        break;

      case OM_SPEED_PUSH_FAN:
      case OM_FLOW_PUSH_FAN:
      case OM_POWER_PUSH_FAN:
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
      setMasterFanSpeed(lastEncoderOperationValue * 10.23);
      break;

    case OM_FLOW_PULL_FAN:
    case OM_FLOW_PUSH_FAN:
      numberSelector.setRange(10.0, 200.0, 0.1, false, 1); // 0 - 200 (m3/h)
      setpointPidFlow = lastEncoderOperationValue >= 10.0 
        && lastEncoderOperationValue <= 200.0
        ? lastEncoderOperationValue : 10.0;
      numberSelector.setValue(setpointPidFlow);
      break;

    case OM_POWER_PULL_FAN:
    case OM_POWER_PUSH_FAN:
      numberSelector.setRange(0.0, 100.0, 0.1, false, 1); // 0 - 100%
      numberSelector.setValue(lastEncoderOperationValue >= 0.0 
        && lastEncoderOperationValue <= 100.0
        ? lastEncoderOperationValue : 10.0);
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
      numberSelector.setValue(MT_SELECT_OPERATION); // sets initial value
      displaySelectMode(MT_SELECT_OPERATION);
      break;

    case MT_SELECT_OPERATION:
      numberSelector.setRange(0, 5, 1, true, 0);
      numberSelector.setValue(operationMode);
      displaySelectOperationMode(operationMode);
      break;
    
    case MT_OPERATION:
      setOperationMode(operationMode);
      displayMeasurements();
      break;

    case MT_CAL_FLOW:
      numberSelector.setRange(0.9, 1.0, 0.001, false, 3);
      numberSelector.setValue(venturi.dischargeCoefficient);
      break;

    case MT_TUNE_PID_FLOW:
    case MT_TUNE_PID_BALANCE:
      pidTuneType = PID_TUNE_NONE;
      numberSelector.setRange(0, 3, 1, true, 0);
      numberSelector.setValue(PID_TUNE_NONE);
      displaySelectTunePID(PID_TUNE_NONE);
      break;

    case MT_ADJUST_OFFSETS:
      adjustSensorOffsets();
      initNextMode(MT_OPERATION);
      break;

    case MT_SET_DIAMETERS:
      setDiameterType = SET_DIA_NONE;
      numberSelector.setRange(0, 2, 1, true, 0);
      numberSelector.setValue(SET_DIA_NONE);
      displaySelectSetDiameter(SET_DIA_NONE);
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

    case MT_SELECT_OPERATION:
      setOperationMode((OperationMode)numberSelector.getValue());
      initNextMode(MT_OPERATION);
      break;

    case MT_OPERATION:
      initNextMode(MT_SELECT_OPERATION);
      break;

    case MT_CAL_FLOW:
      venturi.dischargeCoefficient = numberSelector.getValue();
      saveFloat(KEY_VENTURI_CD, venturi.dischargeCoefficient);
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
              numberSelector.setRange(0, 20, 0.1, false, 1);
              numberSelector.setValue(KpFlow);
              displayMeasurements();
              break;
            case PID_TUNE_I:
              pidTuneType = PID_TUNE_I;
              numberSelector.setRange(0, 10, 0.1, false, 1);
              numberSelector.setValue(KiFlow);
              displayMeasurements();
              break;
            case PID_TUNE_D:
              pidTuneType = PID_TUNE_D;
              numberSelector.setRange(0, 10, 0.1, false, 1);
              numberSelector.setValue(KdFlow);
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
              numberSelector.setRange(0, 20, 0.1, false, 1);
              numberSelector.setValue(KpBalance);
              displayMeasurements();
              break;
            case PID_TUNE_I:
              pidTuneType = PID_TUNE_I;
              numberSelector.setRange(0, 10, 0.1, false, 1);
              numberSelector.setValue(KiBalance);
              displayMeasurements();
              break;
            case PID_TUNE_D:
              pidTuneType = PID_TUNE_D;
              numberSelector.setRange(0, 10, 0.1, false, 1);
              numberSelector.setValue(KdBalance);
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
      }
      break;
   
    case MT_SET_DIAMETERS:
      switch (setDiameterType) {
        case SET_DIA_NONE:
          switch ((SetDiameterType)(uint8_t)numberSelector.getValue()) {
            case SET_DIA_NONE:
              initNextMode(MT_SELECT);
              break;
            case SET_DIA_INLET:
              setDiameterType = SET_DIA_INLET;
              numberSelector.setRange(0.01, 0.3, 0.001, false, 3);
              numberSelector.setValue(venturi.inletDiameter);
              displayMeasurements();
              break;
            case SET_DIA_THROAT:
              setDiameterType = SET_DIA_THROAT;
              numberSelector.setRange(0.01, 0.25, 0.001, false, 3);
              numberSelector.setValue(venturi.throatDiameter);
              displayMeasurements();
             break;
          }
          break;
        case SET_DIA_INLET:
          venturi.inletDiameter = numberSelector.getValue();
          venturi.areaInlet = M_PI * pow(venturi.inletDiameter / 2.0, 2);
          venturi.betaRatio = venturi.throatDiameter / venturi.inletDiameter;
          venturi.betaCoefficient = 1.0 - pow(venturi.betaRatio, 4);
          saveFloat(KEY_VENTURI_INLET_DIAMETER, venturi.inletDiameter);
          initNextMode(MT_SET_DIAMETERS);
          break;
        case SET_DIA_THROAT:
          venturi.throatDiameter = numberSelector.getValue();
          venturi.areaThroat = M_PI * pow(venturi.throatDiameter / 2.0, 2);
          venturi.betaRatio = venturi.throatDiameter / venturi.inletDiameter;
          venturi.betaCoefficient = 1.0 - pow(venturi.betaRatio, 4);
          saveFloat(KEY_VENTURI_THROAT_DIAMETER, venturi.throatDiameter);
          initNextMode(MT_SET_DIAMETERS);
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

      case MT_SELECT_OPERATION:
        displaySelectOperationMode((OperationMode)numberSelector.getValue());
        break;
      
      case MT_OPERATION:
        lastEncoderOperationValue = numberSelector.getValue();
        switch (operationMode) {
          case OM_SPEED_PULL_FAN:
          case OM_SPEED_PUSH_FAN:
            setMasterFanSpeed(lastEncoderOperationValue * 10.23);
            displayTextNumber("Sf %.1f%%", lastEncoderOperationValue);
            break;

          case OM_FLOW_PULL_FAN:
          case OM_FLOW_PUSH_FAN:
            setpointPidFlow = lastEncoderOperationValue;
            displayTextNumber("flow %.1fm3/s", setpointPidFlow);
            break;

          case OM_POWER_PULL_FAN:
          case OM_POWER_PUSH_FAN:
            setpointPidFlow = lastEncoderOperationValue;
            displayTextNumber("Pf %.1f%%", setpointPidFlow);
            break;
        }
        break;

      case MT_CAL_FLOW:
        venturi.dischargeCoefficient = numberSelector.getValue();
        displayTextNumber("Cd %.3f", venturi.dischargeCoefficient);
        break;

      case MT_TUNE_PID_FLOW:
        switch (pidTuneType) {

          case PID_TUNE_NONE:
            displaySelectTunePID((PidTuneType)(uint8_t)numberSelector.getValue());
            break;

          case PID_TUNE_P:
            KpFlow = numberSelector.getValue();
            pidFlow.SetTunings(KpFlow, KiFlow, KdFlow);
            displayTextNumber("Kp flow: %.2f", KpFlow);
            break;

          case PID_TUNE_I:
            KiFlow = numberSelector.getValue();
            pidFlow.SetTunings(KpFlow, KiFlow, KdFlow);
            displayTextNumber("Ki flow: %.2f", KiFlow);
            break;
          
          case PID_TUNE_D:
            KdFlow = numberSelector.getValue();
            pidFlow.SetTunings(KpFlow, KiFlow, KdFlow);
            displayTextNumber("Kd flow: %.2f", KdFlow);
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
            pidBalance.SetTunings(KpBalance, KiBalance, KdBalance);
            displayTextNumber("Kp balance: %.2f", KpBalance);
            break;

          case PID_TUNE_I:
            KiBalance = numberSelector.getValue();
            pidBalance.SetTunings(KpBalance, KiBalance, KdBalance);
            displayTextNumber("Ki balance: %.2f", KiBalance);
            break;
          
          case PID_TUNE_D:
            KdBalance = numberSelector.getValue();
            pidBalance.SetTunings(KpBalance, KiBalance, KdBalance);
            displayTextNumber("Kd balance: %.2f", KdBalance);
            break;
        } 
        break;
      case MT_SET_DIAMETERS:
        switch (setDiameterType) {
          case SET_DIA_NONE:
            displaySelectSetDiameter((SetDiameterType)(uint8_t)numberSelector.getValue());
            break;
          case SET_DIA_INLET:
            displayTextNumber("Inlet dia: %.3fm", numberSelector.getValue());
            break;
          case SET_DIA_THROAT:
            displayTextNumber("Throat dia: %.3fm", numberSelector.getValue());
            break;
        }
        break;  
    }
  } 
  handle_rotary_button();
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
      venturiPressure.add(differentialPressure - offsetVenturiPressure);
    }

    error = sdpBalance.readMeasurement(differentialPressure, temperature);
    if (error) {
      Serial.print("Error trying to execute readMeasurement from Balance Pressure sensor");
      balancePressure.add(0.0);
    } else {
      balancePressure.add(differentialPressure - offsetBalancePressure);
    }
  }
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
    switch (operationMode) {
      case OM_SPEED_PULL_FAN:
      case OM_SPEED_PUSH_FAN:
        // display setpoint speed fan
        display.printf("Sf %.1f%%", numberSelector.getValue());
        snprintf(message, sizeof(message), "%urpm", tachoPullFanRPM);
        printAlignRight(message,127,0);
        break;
      case OM_FLOW_PULL_FAN:
      case OM_FLOW_PUSH_FAN:
        // display setpoint flow
        display.printf("fl %.1fm3/h", numberSelector.getValue());
        snprintf(message, sizeof(message), "%urpm", tachoPullFanRPM);
        printAlignRight(message,127,0);
        break;
      case OM_POWER_PULL_FAN:
      case OM_POWER_PUSH_FAN:
        // display setpoint power
        display.printf("Pf %.1f%%", numberSelector.getValue());
        break;
    }

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
    }     
  
  } else if (modeType == MT_SET_DIAMETERS) {
    if (setDiameterType == SET_DIA_INLET) {
      // display inlet diameter venturi
      display.printf("Inlet dia: %.3fm", numberSelector.getValue());
    } else if (setDiameterType == SET_DIA_THROAT) {
      // display throat diameter venturi
      display.printf("Throat dia: %.3fm", numberSelector.getValue());
    }
  }
  readPressureSensors();

  // display flow
  snprintf(message, sizeof(message),"%.1f", flow.get());
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

  if (mtype == MT_SELECT_OPERATION)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Select Operation Mode");
  if (mtype == MT_SELECT_OPERATION) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_OPERATION)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Operation Mode");
  if (mtype == MT_OPERATION) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_CAL_FLOW) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Calibrate Flow");
  if (mtype == MT_CAL_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_TUNE_PID_FLOW) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Tune PID Flow");
  if (mtype == MT_TUNE_PID_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_TUNE_PID_BALANCE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 37);
  display.print("Tune PID Balance");
  if (mtype == MT_TUNE_PID_BALANCE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_ADJUST_OFFSETS) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 46);
  display.print("Adjust Offsets");
  if (mtype == MT_ADJUST_OFFSETS) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_SET_DIAMETERS) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 55);
  display.print("Set Diameters");
  if (mtype == MT_SET_DIAMETERS) display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void  displaySelectOperationMode(OperationMode mode) {
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

  if (mode == OM_POWER_PULL_FAN) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 32);
  display.print("Power Pull Fan");
  if (mode == OM_POWER_PULL_FAN) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mode == OM_POWER_PUSH_FAN) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 40);
  display.print("Power Push Fan");
  if (mode == OM_POWER_PUSH_FAN) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

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
  
  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectSetDiameter(SetDiameterType type) {
  
  display.clearDisplay();
  display.setTextSize(1);

  if (type == SET_DIA_NONE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Back");
  if (type == SET_DIA_NONE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == SET_DIA_INLET) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Set Inlet Dia");
  if (type == SET_DIA_INLET) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == SET_DIA_THROAT)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Set Throat Dia");
  if (type == SET_DIA_THROAT) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();
}
//////////////////////////////////////////////////////////////////////////

/**
 * Calculates the air density (Rho) based on temperature, pressure, and humidity.
 * @param tempC Temperature in Celsius of the BME280.
 * @param absPressurePa Absolute air pressure in Pascal of the BME280.
 * @param humidityPct Relative humidity in % of the BME280.
 */
static void setRho(float tempC, float absPressurePa, float humidityPct) {

    // --- 1. Air density calculate (humid air) ---
    float T = tempC + 273.15; // Kelvin
    float phi = humidityPct / 100.0;
    
    // Saturated vapor pressure (Magnus) and actual vapor pressure
    float pSat = 610.78 * exp((17.27 * tempC) / (tempC + 237.3));
    float pv = phi * pSat;
    float pd = absPressurePa - pv;

    // rho = (Pd / (Rd * T)) + (Pv / (Rv * T))
    rho = (pd / (287.058 * T)) + (pv / (461.495 * T));
}
//////////////////////////////////////////////////////////////////////////

// This function calculates the actual air density and uses it to determine the volumetric flow through the Venturi.

/**
 * Berekent de volumestroom (m3/h) gecorrigeerd voor temp, druk en vochtigheid.
 * @param deltaP De gemeten druk van de SDP800 in Pascal (Pa).
 * @param tempC Temperatuur in Celsius van de BME280.
 * @param absPressurePa Absolute luchtdruk in Pascal van de BME280.
 * @param humidityPct Relatieve vochtigheid in % van de BME280.
 */
float getFlow(float deltaP) {
    if (deltaP <= 0) return 0.0;

    // --- 1. get density air
    // float rho = getRho(tempC, absPressurePa, humidityPct);

    // --- 2. Venturi Constants ---
    // const float D = 0.116;      // Inlet diameter (m)
    // const float d = 0.087;      // Throat diameter (m)
    // const float Cd = 0.975;     // Discharge coefficient (adjust after calibration)    
    // const float beta = d / D;
    // const float Cb = 1 - pow(beta, 4);
    // const float A2 = (PI * pow(d, 2)) / 4.0;
    
    // The Flow Formule (Bernoulli + continuity)
    // Q = Cd * A2 * sqrt( (2 * deltaP) / (rho * (1 - beta^4)) )
    float velocityThroat = sqrt((2 * deltaP) / (rho * venturi.betaCoefficient));
    float flowM3s = venturi.dischargeCoefficient * venturi.areaThroat * velocityThroat;

    return flowM3s * 3600.0; // convert to m3/h
}
//////////////////////////////////////////////////////////////////////////

/**
  *                    ____________
  *                  /  dP * T
  * Q  = 0.17228 \  / ____________  (m³/s)
  *               \/      Pabs
  *
  *                                  ____________ 
  *                                 /  dP * T  
  * Q = 3600 x 0.17228 = 620.21 \  / ____________  (m³/h)
  *                              \/      Pabs
  * 
*/

/*
static float calculateFlowCompensated(float dP) {
  float Pabs = pressureAbsolute.get() * 100.0;
  float Tamb = temperatureAmbient.get() + 273.15;

  return dP > 0.0 ? Cf * sqrt(dP * Tamb / Pabs) : 0.0; // (m³/h)
}
/////////////////////////////////////////////////////////////////////////*/

// This function calculates the actual air density and uses it to determine the volumetric flow through the Venturi.

/**
 * Calculates the volumetric flow (m3/h) corrected for temperature, pressure, and humidity.
 * @param deltaP The measured pressure of the SDP800 in Pascal (Pa).
 * @param tempC Temperature in Celsius of the BME280.
 * @param absPressurePa Absolute air pressure in Pascal of the BME280.
 * @param humidityPct Relative humidity in % of the BME280.
 */
float _getCompensatedFlow(float deltaP, float tempC, float absPressurePa, float humidityPct) {
    if (deltaP <= 0) return 0.0;

    // --- 1. air density calculate (humid air) ---
    float T = tempC + 273.15; // Kelvin
    float phi = humidityPct / 100.0;
    
    // Saturated vapor pressure (Magnus) and actual vapor pressure
    float pSat = 610.78 * exp((17.27 * tempC) / (tempC + 237.3));
    float pv = phi * pSat;
    float pd = absPressurePa - pv;

    // rho = (Pd / (Rd * T)) + (Pv / (Rv * T))
    float rho = (pd / (287.058 * T)) + (pv / (461.495 * T));

    // --- 2. Venturi Constants ---
    const float D = 0.120;      // Inlet diameter (m)
    const float d = 0.090;      // Throat diameter (m)
    const float Cd = 0.975;     // Discharge coefficient (after calibration)
    
    float beta = d / D;
    float areaThroat = (PI * pow(d, 2)) / 4.0;

    // --- 3. De Flow Formule (Bernoulli and continuity equation) ---
    // Q = Cd * A2 * sqrt( (2 * deltaP) / (rho * (1 - beta^4)) )
    float velocityThroat = sqrt((2 * deltaP) / (rho * (1 - pow(beta, 4))));
    float flowM3s = Cd * areaThroat * velocityThroat;

    return flowM3s * 3600.0; // convert to m3/h
}

