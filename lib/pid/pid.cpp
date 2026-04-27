/**********************************************************************************************
 * Arduino PID Library - Version 1.2.2
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <pid.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
Pid::Pid(double* input, double* output, double* setpoint,
        double Kp, double Ki, double Kd, ProportionalOn pOn, Direction controllerDirection) 
  : _output(output)
  , _input(input)
  , _setpoint(setpoint)
  , _inAuto(false)
  , _sampleTime(100)
  , _lastTime(millis() -_sampleTime) {

   // default output limit corresponds to the arduino pwm limits  
  setOutputLimits(0, 255);
  setControllerDirection(controllerDirection);
  setTunings(Kp, Ki, Kd, pOn);
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

Pid::Pid(double* input, double* output, double* setpoint,
    double Kp, double Ki, double Kd, Direction controllerDirection)
  : Pid::Pid(input, output, setpoint, Kp, Ki, Kd, P_ON_ERROR, controllerDirection) {}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool Pid::compute() {
  if (!_inAuto) return false;

  unsigned long now = millis();
  unsigned long timeChange = (now - _lastTime);

  if (timeChange >=_sampleTime) {
    /* Compute all the working error variables*/
    update();
    _lastTime = now;
	  return true;
  }
  return false;
}

bool Pid::update() {
  if (!_inAuto) return false;

  /* Compute all the working error variables*/
  double input = *_input;
  double error = *_setpoint - input;
  double dInput = (input - _lastInput);
  _outputSum += (_ki * error);

  /* Add Proportional on Measurement, if P_ON_M is specified*/
  if (!_pOnE)
    _outputSum -= _kp * dInput;

  if (_outputSum > _outMax)
    _outputSum = _outMax;
  else if (_outputSum < _outMin)
    _outputSum = _outMin;

  /* Add Proportional on Error, if P_ON_E is specified*/
  double output = _pOnE ? _kp * error : 0;
  
  /*Compute Rest of PID Output*/
  output += _outputSum - _kd * dInput;

  if (output > _outMax)
    *_output = _outMax;
  else if (output < _outMin)
    *_output = _outMin;
  else 
    *_output = output;

  /* Remember some variables for next time */
  _lastInput = input;  
  return true;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void Pid::setTunings(double Kp, double Ki, double Kd, ProportionalOn pOn) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  _pOn = pOn;
  _pOnE = pOn == Pid::P_ON_ERROR;

  _dispKp = Kp; _dispKi = Ki; _dispKd = Kd;

  double sampleTimeInSec = ((double)_sampleTime) / 1000;
  _kp = Kp;
  _ki = Ki * sampleTimeInSec;
  _kd = Kd / sampleTimeInSec;

  if (_controllerDirection == Pid::REVERSE) {
    _kp = (0 - _kp);
    _ki = (0 - _ki);
    _kd = (0 - _kd);
  }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void Pid::setTunings(double Kp, double Ki, double Kd) {
    setTunings(Kp, Ki, Kd, _pOn); 
}

/* setSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void Pid::setSampleTime(int newSampleTime) {
  if (newSampleTime > 0) {
    double ratio  = (double)newSampleTime / (double)_sampleTime;
    _ki *= ratio;
    _kd /= ratio;
    _sampleTime = (unsigned long)newSampleTime;
  }
}

/* setOutputLimits(...)****************************************************
 *     This function will be used far more often than setInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void Pid::setOutputLimits(double min, double max) {
  if (min >= max) return;
  
  _outMin = min;
  _outMax = max;

  if (_inAuto) {
	  if (*_output > _outMax)
      *_output = _outMax;
	  else if(*_output < _outMin)
      *_output = _outMin;

	  if (_outputSum > _outMax)
      _outputSum= _outMax;
	  else if(_outputSum < _outMin)
      _outputSum= _outMin;
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void Pid::setMode(Mode mode) {
  bool newAuto = (mode == AUTOMATIC);

  if (newAuto && !_inAuto) { 
    /*we just went from manual to auto*/
    initialize();
  }
  _inAuto = newAuto;
}

/* initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Pid::initialize() {
  _outputSum = *_output;
  _lastInput = *_input;

  if (_outputSum > _outMax)
    _outputSum = _outMax;
  else if (_outputSum < _outMin) 
    _outputSum = _outMin;
}

/* setControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void Pid::setControllerDirection(Direction direction) {
  
  if (_inAuto && direction != _controllerDirection) {
	  _kp = (0 - _kp);
    _ki = (0 - _ki);
    _kd = (0 - _kd);
  }
  _controllerDirection = direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double Pid::getKp() { return  _dispKp; }
double Pid::getKi() { return  _dispKi;}
double Pid::getKd() { return  _dispKd;}
Pid::Mode Pid::getMode() { return  _inAuto ? AUTOMATIC : MANUAL;}
Pid::Direction Pid::getDirection() { return _controllerDirection;}
