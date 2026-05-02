#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PidController{
  
  public:
    typedef enum {
      DIRECT = 0,
      REVERSE = 1,
    } Direction;

    PidController(double kp, double ki, double kd, double dt, double minOutput, double maxOutput);
    ~PidController();
    float update(double input); // should be called every delta t seconds.
    void reset() { _integral = 0; }
    double getOutput() const { return _lastOutput; }
    void setSetpoint(double sp) { _setpoint = sp; }
    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double minOut, double maxOut, double trendLimit) { _minOutput = minOut; _maxOutput = maxOut; _outputTrendLimit = trendLimit; }
    void setSampleTime(double dt) { _dt = dt; } // in seconds
    void setControllerDirection(Direction dir);
    void setAlpha(double alpha);
    double getAlpha() const;
    void setBeta(double beta) { _beta = beta; } // setter for beta, the error-scaling factor
    double getBeta() const { return _beta; } // getter voor beta
    void setOutputTrendLimit(double trendLimit) { _outputTrendLimit = trendLimit; } // setter for output trend limit
    double getOutputTrendLimit() const { return _outputTrendLimit; } // setter for output trend limit
    double getFilteredInput() const { return _filteredInput; }

  private:
    double _kp, _ki, _kd;
    double _dt;
    double _alpha; // for input low-pass filtering.
    double _beta; // for error scaling, range 0-1, higher values makes the controller more aggressive.
    double _filteredInput;
    double _setpoint;
    double _lastOutput;
    double _outputTrendLimit; // Max change in output per update to prevent aggressive changes.
    double _integral;
    double _minOutput;
    double _maxOutput;
    Direction _controllerDirection;   
};
#endif // PIDCONTROLLER_H