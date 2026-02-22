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
const int FAN_ON_PIN  = 14;
const int PWM_FAN_PIN = 27; 

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
const int PWM_FAN_CHAN = LEDC_CHANNEL_0;
const int PWM_FREQ = 25000;    // 25 kHz frequency for computer fans
const int PWM_RESOLUTION = 10; // 10-bit resolution (0-1023)

// Venturi Constants
const float D = 0.116;      // Inlet diameter (m)
const float d = 0.087;      // Throat diameter (m)
float Cd = 0.975;           // Discharge coefficient (adjust after calibration)    
const float beta = d / D;
const float Cb = 1 - pow(beta, 4);
const float A1 = (PI * pow(D, 2)) / 4.0; // Inlet area (m3)
const float A2 = (PI * pow(d, 2)) / 4.0; // Throat area (m3)

//paramaters for button
const unsigned long shortPressAfterMiliseconds = 50;   // how long short press shoud be.
                                                       // Do not set too low to avoid bouncing (false press events).
const unsigned long longPressAfterMiliseconds = 1000;  // how long long press shoud be.

AiEsp32RotaryEncoder *rotaryEncoder = new AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS, false);
AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();

Preferences preferences;

Smoothed<float> flow;
Smoothed<float> flowPressure;
Smoothed<float> calibration;
Smoothed<float> pressureAbsolute;
Smoothed<float> temperatureAmbient;
Smoothed<float> humidityAmbient;

// Twee hardware I2C bussen
TwoWire I2C_A = TwoWire(0);
TwoWire I2C_B = TwoWire(1);

SensirionI2CSdp sdpFlow;

// Initialize the OLED display using Wire library
// ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically
// based on your board's pins_arduino.h 
// e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SH1106Wire      display(I2C_ADDRESS_DISPLAY, I2C_SDA0_PIN, I2C_SCL0_PIN, GEOMETRY_128_64);
Adafruit_SH1106G display(128, 64, &I2C_A);
Adafruit_BME280  bme280;     // I2C

hw_timer_t *timer0 = nullptr;
volatile bool ms10_passed = false;

ModeType modeType = MT_SPEED;
bool direction = false; // false for return

// Define Variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

// Specify the links and initial tuning parameters
double Kp=6, Ki=3, Kd=0;
PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, REVERSE);

const float flowFactor = 620.21;
float Cf = Cd * flowFactor;
float offsetFlowPressure;
float rho = 1.2; // densitity of air at 1013,25 hPa, 20 C, and 60 %Rh (Kg/m3)

static void  initDisplay(void);
static void  displayMeasurements();
static void  displaySelectMode(ModeType);
static void  displayNumberSetpoint();
static void  initBME280();
static void  initSDP(SensirionI2CSdp&, TwoWire&);
static float calculateFlowCompensated(float dP);
static void  drawString(int16_t x, int16_t y, const String &text);
static float getFlow(float dP);
static void  setRho(float tempC, float absPressurePa, float humidityPct);

//////////////////////////////////////////////////////////////////////////

// Interrupt Service Routine (ISR)
// IRAM_ATTR places the function in RAM for faster execution
void IRAM_ATTR Timer0_ISR() {
  ms10_passed = true;
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

void setupTimer0() {
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

float getFloat(const char* key, float value = NAN) {
  float f;
  preferences.begin("testbench", false);
  f = preferences.getFloat(key, value);
  preferences.end();
  return f == NAN ? 0.0 : f;
}
//////////////////////////////////////////////////////////////////////////

void saveFloat(const char* key, float value) {
  preferences.begin("testbench", false);
  preferences.putFloat(key, value);
  preferences.end();
}
//////////////////////////////////////////////////////////////////////////

void initCf() {
  preferences.begin("testbench", true);
  Cd = preferences.getFloat("Cd", 0.0);
  if (Cd >= 0.9 && Cd <= 1.0) {
    Cf = flowFactor * Cd;
  }
  preferences.end();
}
//////////////////////////////////////////////////////////////////////////

void setupRotaryEncoder() {
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

void setFanSpeed(uint32_t speed) {
  static uint32_t lastspeed = 0xFFFFFFFF;

  if (lastspeed != speed) {
    ledcWrite(PWM_FAN_CHAN, speed);
    if (speed == 0) {
      digitalWrite(FAN_ON_PIN, LOW);
    } else if (lastspeed == 0) {
      digitalWrite(FAN_ON_PIN, HIGH);
    }
    lastspeed = speed;
 }
} 
//////////////////////////////////////////////////////////////////////////

void setup() {
  
  // prefent fan from starting up
  pinMode(FAN_ON_PIN, OUTPUT);
  digitalWrite(FAN_ON_PIN, LOW);

  Serial.begin(115200);

  Serial.println("\ntestbench_airflowmeter is starting up...");

  // set Cf
  initCf();

  // setup RotaryEncoder
  setupRotaryEncoder();

  // Configure PWM
  Serial.println("Setup PWM");
  ledcSetup(PWM_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_FAN_PIN, PWM_FAN_CHAN);
 
  // Set initial fan speed to zero
  setFanSpeed(0);
  delay(1000);

  analogReadResolution(12);

  // Start both busses
  I2C_A.begin(I2C_SDA0_PIN, I2C_SCL0_PIN, 700000);   // SDA, SCL
  I2C_B.begin(I2C_SDA1_PIN, I2C_SCL1_PIN, 100000);

  Serial.println("Setup Display");
  initDisplay();
  delay(500);
  Serial.println("Setup BME280");
  initBME280();
  delay(500);
  Serial.println("Setup SDP810 Flow");
  initSDP(sdpFlow, I2C_B);
  delay(500);
  

  Serial.println("setup smoothed average.");
  flow.begin(SMOOTHED_AVERAGE, 50);
  flowPressure.begin(SMOOTHED_AVERAGE, 100);
  calibration.begin(SMOOTHED_AVERAGE, 200);
  pressureAbsolute.begin(SMOOTHED_AVERAGE, 40);
  humidityAmbient.begin(SMOOTHED_AVERAGE, 40);
  temperatureAmbient.begin(SMOOTHED_AVERAGE, 40);
  delay(500);

  //
  Serial.println("Getting offset flowsensor."); 
  for (size_t i = 0; i < 200; i++) {
    float differentialPressure;
    float temperature;
    uint16_t error = sdpFlow.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from flowsensor");
       break;
    }
    calibration.add(differentialPressure);
    delay(10);
  }
  delay(500);
  offsetFlowPressure = calibration.get();
  Serial.printf("offset flow %.1f\n", offsetFlowPressure);

  // initialize the PID variables
  Kp = getFloat("Kp", Kp);
  Ki = getFloat("Ki", Ki );
  Kd = getFloat("Kd", Kd);
  pid.SetTunings(Kp, Ki, Kd);
  pidSetpoint = 0.0;
  pidInput = 0.0;

  // turn the PID on
  Serial.println("turn the PID on.");
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 1023);
  pid.SetControllerDirection(direction ? REVERSE : DIRECT);
  
  delay(500);

  Serial.print("Setup timer interrupt\n");
  setupTimer0();
  delay(500);

  Serial.print("\n Setup done...\n\n");
  delay(500);
}
//////////////////////////////////////////////////////////////////////////

void initNextMode(ModeType type) {
  modeType = type;

  switch (type) {
    
    case MT_SELECT:
      break;

    case MT_SPEED:
      numberSelector.setRange(0.0, 100.0, 0.1, false, 1); // 0 - 100%
      numberSelector.setValue(10.0);
      setFanSpeed(102); // 10%
      break;

    case MT_FLOW:
      numberSelector.setRange(10.0, 200.0, 0.1, false, 1); // 0 - 200 (m3/h)
      pidSetpoint = 10.0;
      numberSelector.setValue(pidSetpoint);
      break;

    case MT_POWER:
      numberSelector.setRange(0.0, 100.0, 0.1, false, 1); // 0 - 100%
      numberSelector.setValue(10.0);
      break;

    case MT_CAL_FLOW:
      numberSelector.setRange(0.9, 1.0, 0.001, false, 3);
      numberSelector.setValue(Cd);
      break;

     case MT_PID_TUNE_P:
      numberSelector.setRange(0.0, 10.0, 0.01, false, 2);
      numberSelector.setValue(Kp);
      break;

    case MT_PID_TUNE_I:
      numberSelector.setRange(0.0, 10.0, 0.01, false, 2);
      numberSelector.setValue(Ki);
      break;

    case MT_PID_TUNE_D:
      numberSelector.setRange(0.0, 10.0, 0.01, false, 2);
      numberSelector.setValue(Kd);
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void on_button_short_click() {
  // pidSetpoint = compensationFactorB = numberSelector.getValue();
  // compensationFactorA = flow.get();
  switch (modeType) {
    case MT_SELECT:
      initNextMode((ModeType)numberSelector.getValue()); 
      break;

    case MT_SPEED:
    case MT_FLOW:
    case MT_POWER:
      break;

    case MT_CAL_FLOW:
      saveFloat("Cd", numberSelector.getValue());
      initCf();
      initNextMode(MT_SPEED);
      break;
    
    case MT_PID_TUNE_P:
      saveFloat("Kp", numberSelector.getValue());
      initNextMode(MT_FLOW);
      break;

    case MT_PID_TUNE_I:
      saveFloat("Ki", numberSelector.getValue());
      initNextMode(MT_FLOW);
      break;
      
    case MT_PID_TUNE_D:
      saveFloat("Kd", numberSelector.getValue());
      initNextMode(MT_FLOW);
      break;  
  }
}
//////////////////////////////////////////////////////////////////////////

void on_button_long_click() {
  // toggle direction
  // direction = !direction;
  // myPID.SetControllerDirection(direction ?  REVERSE : DIRECT);
  
  switch(modeType) {
    case MT_SPEED:
    case MT_FLOW:
    case MT_POWER:
      numberSelector.setRange(1, 7,  1, true, 0);
      numberSelector.setValue(modeType); // sets initial value
      displaySelectMode(modeType);
      modeType = MT_SELECT;
      break;
    default:
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void handle_rotary_button() {
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

void loopRotaryEncoder() {
  
  if (rotaryEncoder->encoderChanged()) {
    switch (modeType) {
      
      case MT_SELECT:
        displaySelectMode((ModeType)numberSelector.getValue());
        break;

      case MT_SPEED:
        setFanSpeed(numberSelector.getValue()* 10.23);
        display.printf("Sf %.1f%%", numberSelector.getValue());

        break;

      case MT_FLOW:
        pidSetpoint = numberSelector.getValue();
        displayTextNumber("flow %.1fm3/s", pidSetpoint);
        break;

      case MT_POWER:
        pidSetpoint = numberSelector.getValue();
        displayTextNumber("Pf %.1f%%", pidSetpoint);
        break;

      case MT_CAL_FLOW:
        Cf = flowFactor * numberSelector.getValue();
        displayTextNumber("Cd %.3f", Kp);
        break;

      case MT_PID_TUNE_P:
        Kp = numberSelector.getValue();
        pid.SetTunings(Kp, Ki, Kd);
        displayTextNumber("Kp %.2f", Kp);
        break;

      case MT_PID_TUNE_I:
        Ki = numberSelector.getValue();
        pid.SetTunings(Kp, Ki, Kd);
        displayTextNumber("Ki %.2f", Ki);
        break;
      
      case MT_PID_TUNE_D:
        Kd = numberSelector.getValue();
        pid.SetTunings(Kp, Ki, Kd);
        displayTextNumber("Kd %.2f", Kd);
        break;
    }
  } 
  handle_rotary_button();
}
//////////////////////////////////////////////////////////////////////////

void readPressureSensors() {
  
  if (ms10_passed == true) {
    ms10_passed = false;
    float differentialPressure, temperature;

    // 2 ms time to read
    uint16_t error = sdpFlow.readMeasurement(differentialPressure, temperature);
    if (error) {
      Serial.print("Error trying to execute readMeasurement from FlowPressure");
    } else {
      flowPressure.add(differentialPressure - offsetFlowPressure);
    }
  }
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

    flow.add(getFlow(flowPressure.get()));

    switch (modeType) {
      
      case MT_SPEED:
        setFanSpeed(numberSelector.getValue() * 10.23);
        break;

      case MT_FLOW:
      case MT_PID_TUNE_P:
      case MT_PID_TUNE_I:
      case MT_PID_TUNE_D:
        pidInput = flow.get();
        pid.Compute();
        setFanSpeed(pidOutput);
        break;
    }
    readPressureSensors();
    
    // every second
    if (loopcnt % 10 == 0) {

      // Only needed in forced mode! In normal mode, you can remove the next line.
      bme280.takeForcedMeasurement(); // has no effect in normal mode
      readPressureSensors();

      // get the measurements from the BME280
      pressureAbsolute.add(bme280.readPressure() / 100.0); // convert from Pa to hPa
      readPressureSensors();
      temperatureAmbient.add(bme280.readTemperature());
      readPressureSensors();
      humidityAmbient.add(bme280.readHumidity());
      readPressureSensors();

      Serial.printf("Flow pressure: %.1f Pa\n", flowPressure.get());
      Serial.printf("PWM fan dutycycle: %.1f%%\n", pidOutput / 10.23);
    }

    // every minute
    if (loopcnt % 600 == 0) {
       setRho(temperatureAmbient.get(), pressureAbsolute.get() * 100.0,humidityAmbient.get());
    }
    
    // every 2 seconds
    if (loopcnt % 20 == 0) {
      if (modeType != MT_SELECT) {
        displayMeasurements(); // 42ms
      }
    }

    ++loopcnt;
  }
}
//////////////////////////////////////////////////////////////////////////

void initBME280(void) {
  
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

static void displayMeasurements() {
  char message[32];
  
  display.clearDisplay();

  readPressureSensors();
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  
 if (modeType == MT_SPEED) {
    // display setpoint speed fan
    display.printf("Sf %.1f%%", numberSelector.getValue());
  } else if (modeType == MT_FLOW) {
    // display setpoint flow
    display.printf("flow %.1fm3/h", numberSelector.getValue());
  } else if (modeType == MT_POWER) {
    // display setpoint power
    display.printf("Pf %.1f%%", numberSelector.getValue());
  } else if (modeType == MT_CAL_FLOW) {
    // display discharge coefficient venturi
    display.printf("Cd: %.3f", numberSelector.getValue());
  } else if (modeType == MT_PID_TUNE_P) {
    // display Proportional(P) component PID
    display.printf("Kp %.2f", numberSelector.getValue());
  } else if (modeType == MT_PID_TUNE_I) {
    // display Integral(I) component PID
    display.printf("Ki %.2f", numberSelector.getValue());
  } else if (modeType == MT_PID_TUNE_D) {
    // display Derivative(D) component PID
    display.printf("Kd %.2f", numberSelector.getValue());     
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

  // display flow pressure
  display.setCursor(0, 41);
  display.printf("Pf %.1f Pa", flowPressure.get());
  readPressureSensors();

  // display temperature
  snprintf(message, sizeof(message),"%.1f C", temperatureAmbient.get());
  printAlignRight(message, 127, 41);
  readPressureSensors();

  // display absolute pressure
  display.setCursor(0, 57);
  display.printf("%.1f hPa", pressureAbsolute.get());
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

  if (mtype == MT_SPEED)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Speed Mode");
  if (mtype == MT_SPEED) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_FLOW)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Flow Mode");
  if (mtype == MT_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_POWER)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Power Mode");
  if (mtype == MT_POWER) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_CAL_FLOW) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Calibrate Flow");
  if (mtype == MT_CAL_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_PID_TUNE_P) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 37);
  display.print("Tune Kp");
  if (mtype == MT_PID_TUNE_P) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_PID_TUNE_I) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 46);
   display.print("Tune Ki");
  if (mtype == MT_PID_TUNE_I) display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  
  if (mtype == MT_PID_TUNE_D) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 55);
   display.print("Tune Kd");
  if (mtype == MT_PID_TUNE_D) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();

}
//////////////////////////////////////////////////////////////////////////

/**
 * Berekent de dichtheid (Rho) van lucht op basis van temp, druk en vochtigheid.
 * @param tempC Temperatuur in Celsius van de BME280.
 * @param absPressurePa Absolute luchtdruk in Pascal van de BME280.
 * @param humidityPct Relatieve vochtigheid in % van de BME280.
 */
static void setRho(float tempC, float absPressurePa, float humidityPct) {

    // --- 1. Luchtdichtheid berekenen (Vochtige lucht) ---
    float T = tempC + 273.15; // Kelvin
    float phi = humidityPct / 100.0;
    
    // Verzadigde dampdruk (Magnus) en actuele dampdruk
    float pSat = 610.78 * exp((17.27 * tempC) / (tempC + 237.3));
    float pv = phi * pSat;
    float pd = absPressurePa - pv;

    // rho = (Pd / (Rd * T)) + (Pv / (Rv * T))
    rho = (pd / (287.058 * T)) + (pv / (461.495 * T));
}
//////////////////////////////////////////////////////////////////////////

// Deze functie berekent eerst de actuele luchtdichtheid en gebruikt die vervolgens 
// om de volumestroom door de Venturi te bepalen.

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
    //float rho = getRho(tempC, absPressurePa, humidityPct);

    // --- 2. Venturi Constanten ---
    // const float D = 0.116;      // Inlet diameter (m)
    // const float d = 0.087;      // Throat diameter (m)
    // const float Cd = 0.975;     // Discharge coefficient (adjust after calibration)    
    // const float beta = d / D;
    // const float Cb = 1 - pow(beta, 4);
    // const float A2 = (PI * pow(d, 2)) / 4.0;
    
    // The Flow Formule (Bernoulli + continuïteit)
    // Q = Cd * A2 * sqrt( (2 * deltaP) / (rho * (1 - beta^4)) )
    float velocityThroat = sqrt((2 * deltaP) / (rho * Cb));
    float flowM3s = Cd * A2 * velocityThroat;

    return flowM3s * 3600.0; // Omrekenen naar m3/h
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

static float calculateFlowCompensated(float dP) {
  float Pabs = pressureAbsolute.get() * 100.0;
  float Tamb = temperatureAmbient.get() + 273.15;

  return dP > 0.0 ? 620.21 * sqrt(dP * Tamb / Pabs) : 0.0; // (m³/h)
}
//////////////////////////////////////////////////////////////////////////

//Deze functie berekent eerst de actuele luchtdichtheid en gebruikt die vervolgens 
// om de volumestroom door de Venturi te bepalen.

/**
 * Berekent de volumestroom (m3/h) gecorrigeerd voor temp, druk en vochtigheid.
 * @param deltaP De gemeten druk van de SDP800 in Pascal (Pa).
 * @param tempC Temperatuur in Celsius van de BME280.
 * @param absPressurePa Absolute luchtdruk in Pascal van de BME280.
 * @param humidityPct Relatieve vochtigheid in % van de BME280.
 */
float _getCompensatedFlow(float deltaP, float tempC, float absPressurePa, float humidityPct) {
    if (deltaP <= 0) return 0.0;

    // --- 1. Luchtdichtheid berekenen (Vochtige lucht) ---
    float T = tempC + 273.15; // Kelvin
    float phi = humidityPct / 100.0;
    
    // Verzadigde dampdruk (Magnus) en actuele dampdruk
    float pSat = 610.78 * exp((17.27 * tempC) / (tempC + 237.3));
    float pv = phi * pSat;
    float pd = absPressurePa - pv;

    // rho = (Pd / (Rd * T)) + (Pv / (Rv * T))
    float rho = (pd / (287.058 * T)) + (pv / (461.495 * T));

    // --- 2. Venturi Constanten ---
    const float D = 0.120;      // Inlaat diameter (m)
    const float d = 0.090;      // Keel diameter (m)
    const float Cd = 0.975;     // Discharge coefficient (na kalibratie aanpassen)
    
    float beta = d / D;
    float areaKeel = (PI * pow(d, 2)) / 4.0;

    // --- 3. De Flow Formule (Bernoulli + continuïteit) ---
    // Q = Cd * A2 * sqrt( (2 * deltaP) / (rho * (1 - beta^4)) )
    float velocityKeel = sqrt((2 * deltaP) / (rho * (1 - pow(beta, 4))));
    float flowM3s = Cd * areaKeel * velocityKeel;

    return flowM3s * 3600.0; // Omrekenen naar m3/h
}

