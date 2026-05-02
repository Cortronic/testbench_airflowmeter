#include "PidController.h"

PidController::PidController(double kp, double ki, double kd, double dt, double minOutput, double maxOutput)
  : _kp(kp)
  , _ki(ki)
  , _kd(kd)
  , _dt(dt)
  , _alpha(0.1) // Voorbeeld waarde voor input low-pass filtering
  , _beta(0.5) // Voorbeeld waarde voor error scaling
  , _filteredInput(0)
  , _setpoint(0)
  , _lastOutput(0)
  , _outputTrendLimit(1.0) // Voorbeeld waarde voor output trend limit
  , _integral(0)
  , _minOutput(minOutput)
  , _maxOutput(maxOutput)
  , _controllerDirection(DIRECT) {
}

PidController::~PidController() {}
 

float PidController::update(double input) {
  
  if (_controllerDirection == REVERSE) {
    input = -input; // Invert input voor reverse mode
  }

  // In deze update functie gebruiken we een eenvoudige low-pass filter op de input voordat we de PID-berekeningen doen.
  double lastFilteredInput = _filteredInput;
  
  // 1. Filter de input met een eenvoudige low-pass filter om ruis te verminderen voordat we de PID-berekeningen doen
  _filteredInput = _alpha * input  +  (1 - _alpha) * _filteredInput; // Simpele low-pass filter op de input
  double error = _setpoint - _filteredInput;

  // 2. Schaal de error met de beta factor om de agressiviteit van de controller aan te passen bij grote fouten
  error *= _beta; // Schaal de error eerst terug naar een bruikbaar bereik, anders kan het kwadrateren te groot worden. Pas deze factor aan op basis van de typische foutwaarden in jouw toepassing.
  
  // 3. Kwadrateer de error om de controller minder agressief te maken bij kleine afwijkingen, maar behoud de richting van de fout
  error = error > 0 ? error * error : - (error * error);
  
  // 4. Bereken P en D actie
  double pTerm = _kp * error;
  double dTrem = _kd * (_filteredInput - lastFilteredInput) / _dt; // Afgeleid van de gefilterde input

  // 5. Integraal berekening met Clamping Anti-Windup
  if (_ki != 0) {
    double potentialIntegral = _integral + (error * _ki * _dt);
    double outputPreClamp = pTerm + potentialIntegral - dTrem;

    // Controleer op verzadiging (PWM 0-255)
    bool saturated = (outputPreClamp > _maxOutput  || outputPreClamp < _minOutput);
    bool sameDirection = (error > 0 && outputPreClamp > _maxOutput) || (error < 0 && outputPreClamp < _minOutput); 

    // Alleen integreren als we NIET verzadigd zijn in de richting van de fout
    if (!(saturated && sameDirection)) {
      _integral = potentialIntegral;
    }
  } else if (_integral > 0.01 || _integral < -0.01) {
    _integral *= 0.9; // Exponentiële decay van de integraal als deze niet gebruikt wordt
  }

  // 6. Definitieve output berekenen en clampen
  double output = pTerm + _integral - dTrem;
  output = output > _maxOutput ? _maxOutput : (output < _minOutput ? _minOutput : output);

  // 7. Output trend clamping om grote sprongen in de output te voorkomen
  _lastOutput = output - _lastOutput > _outputTrendLimit ?
    _lastOutput + _outputTrendLimit :
    (output - _lastOutput < -_outputTrendLimit ? _lastOutput - _outputTrendLimit : output);
    
  return _lastOutput;  
}

void PidController::setAlpha(double alpha) {
  _alpha = alpha; // Zet de alpha waarde voor de input low-pass filter
}

double PidController::getAlpha() const { 
  return _alpha;
}

void PidController::setTunings(double kp, double ki, double kd) { 
  if (kp < 0 || ki < 0 || kd < 0) return; // Negatieve waarden niet toestaan
   _kp = kp; _ki = ki; _kd = kd;
}

void PidController::setControllerDirection(Direction dir) { 
  if (dir != _controllerDirection) {
    _controllerDirection = dir;
    reset(); // Reset integral en filter om abrupte veranderingen te voorkomen 
  } 
}
