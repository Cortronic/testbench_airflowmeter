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
        double Kp, double Ki, double Kd, int POn, int ControllerDirection) 
  : _output(output)
  , _input(input)
  , _setpoint(setpoint)
  , _inAuto(false)
  , _sampleTime(100)
  , _lastTime(millis() -_sampleTime) {

   // default output limit corresponds to the arduino pwm limits  
  setOutputLimits(0, 255);
  setControllerDirection(ControllerDirection);
  setTunings(Kp, Ki, Kd, POn);
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

Pid::Pid(double* input, double* output, double* setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
    : Pid::Pid(input, output, setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool Pid::Compute() {
  if (!_inAuto) return false;

  unsigned long now = millis();
  unsigned long timeChange = (now - _lastTime);
  if (timeChange >=_sampleTime) {
    /*Compute all the working error variables*/
    double input = *_input;
    double error = *_setpoint - input;
    double dInput = (input - _lastInput);
    _outputSum += (_ki * error);

    /*Add Proportional on Measurement, if P_ON_M is specified*/
    if (!_pOnE)
      _outputSum -= _kp * dInput;

    if (_outputSum > _outMax)
      _outputSum = _outMax;
    else if (_outputSum < _outMin)
      _outputSum = _outMin;

    /*Add Proportional on Error, if P_ON_E is specified*/
	  double output;
    if (_pOnE)
      output = _kp * error;
    else
      output = 0;

    /*Compute Rest of PID Output*/
    output += _outputSum - _kd * dInput;

	  if (output > _outMax)
      output = _outMax;
    else if (output < _outMin)
      output = _outMin;
	  *_output = output;

    /*Remember some variables for next time*/
    _lastInput = input;
    _lastTime = now;
	  return true;
  }
  return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void Pid::setTunings(double Kp, double Ki, double Kd, int POn) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  _pOn = POn;
  _pOnE = POn == P_ON_E;

  _dispKp = Kp; _dispKi = Ki; _dispKd = Kd;

  double SampleTimeInSec = ((double)_sampleTime) / 1000;
  _kp = Kp;
  _ki = Ki * SampleTimeInSec;
  _kd = Kd / SampleTimeInSec;

  if (_controllerDirection == REVERSE) {
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
void Pid::setSampleTime(int NewSampleTime) {
  if (NewSampleTime > 0) {
    double ratio  = (double)NewSampleTime / (double)_sampleTime;
    _ki *= ratio;
    _kd /= ratio;
    _sampleTime = (unsigned long)NewSampleTime;
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
void Pid::setOutputLimits(double Min, double Max) {
  if (Min >= Max) return;
  
  _outMin = Min;
  _outMax = Max;

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
void Pid::setMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);

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
  else if ( _outputSum < _outMin) 
    _outputSum = _outMin;
}

/* setControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void Pid::setControllerDirection(int Direction) {
  
  if (_inAuto && Direction != _controllerDirection) {
	  _kp = (0 - _kp);
    _ki = (0 - _ki);
    _kd = (0 - _kd);
  }
  _controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double Pid::getKp(){ return  _dispKp; }
double Pid::getKi(){ return  _dispKi;}
double Pid::getKd(){ return  _dispKd;}
int Pid::getMode(){ return  _inAuto ? AUTOMATIC : MANUAL;}
int Pid::getDirection(){ return _controllerDirection;}
