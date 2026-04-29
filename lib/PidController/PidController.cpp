#include "PidController.h"

PidController::PidController(double kp, double ki, double kd, double dt, double minOutput, double maxOutput)
  : _kp(kp), _ki(ki), _kd(kd)
  , _dt(dt), _setpoint(0), _output(0)
  , _integral(0)
  , _minOutput(minOutput), _maxOutput(maxOutput)
  , _controllerDirection(DIRECT) {
  _filter = new DoubleExponentialFilter(0.1, 0.01, 0.95, 5.0); // Voorbeeld parameters
}

PidController::~PidController() {
  delete _filter;
}

float PidController::update(double input) {
  // 1. Filter & Trend (Gebruik de klasse van hiervoor)
  double filtered = _filter->update(input);
  double trend = _filter->getTrend();
  double error = _setpoint - filtered;

  // 2. Bereken P en D actie
  double pTerm = _kp * error;
  double dTerm = _kd * trend; // Directe trend koppeling

  if (_ki != 0) {
    // 3. Integraal berekening met Clamping Anti-Windup
    double potentialIntegral = _integral + (error * _ki * _dt);
    double outputPreClamp = pTerm + potentialIntegral - dTerm;

    // Controleer op verzadiging (PWM 0-255)
    bool saturated = (outputPreClamp > _maxOutput  || outputPreClamp < _minOutput);
    bool sameDirection = (error > 0 && outputPreClamp > _maxOutput) || (error < 0 && outputPreClamp < _minOutput);

    // Alleen integreren als we NIET verzadigd zijn in de richting van de fout
    if (!(saturated && sameDirection)) {
      _integral = potentialIntegral;
    }
  } else if (_integral > 0.01 || _integral < -0.01) {
    // Kleine drempel om onnodig kleine integrale waarden te voorkomen
    _integral *= 0.9; // Exponentiële decay van de integraal als deze niet gebruikt wordt
  }

  // 4. Definitieve output berekenen en clampen
  _output = pTerm + _integral - dTerm;
  _output = _output > _maxOutput ? _maxOutput : (_output < _minOutput ? _minOutput : _output);
  return _output;
}

void PidController::setTunings(double kp, double ki, double kd) { 
  if (kp < 0 || ki < 0 || kd < 0) return; // Negatieve waarden niet toestaan

  if (_controllerDirection == DIRECT) {
    _kp = kp; _ki = ki; _kd = kd; 
  } else {
    _kp = -kp; _ki = -ki; _kd = -kd;
  }  
}

void PidController::setControllerDirection(Direction dir) { 
  if (dir != _controllerDirection) {
    _controllerDirection = dir;
    _kp = -_kp; _ki = -_ki; _kd = -_kd;
    reset(); // Reset integral en filter om abrupte veranderingen te voorkomen 
  } 
}
