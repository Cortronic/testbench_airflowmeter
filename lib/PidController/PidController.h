#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "DoubleExponentialFilter.h"

class PidController{
  private:
    double _kp, _ki, _kd;
    double _dt;
    double _setpoint;
    double _output;
    double _integral;
    double _minOutput, _maxOutput;
    DoubleExponentialFilter *_filter;

  public:
    PidController(double kp, double ki, double kd, double dt, double setpoint, double minOutput, double maxOutput);
    ~PidController();
    float update(double pressure);
    void reset() { _integral = 0; _filter->reset(); }
    double getOutput() const { return _output; }
    void setSetpoint(double sp) { _setpoint = sp; }
    void setTunings(double kp, double ki, double kd) { _kp = kp; _ki = ki; _kd = kd; }
    void setOutputLimits(double minOut, double maxOut) { _minOutput = minOut; _maxOutput = maxOut; }
    void setSampleTime(double dt) { _dt = dt; } // in seconds

    double getLevel() const { return _filter->getLevel(); }
    double getTrend() const { return _filter->getTrend(); }
    void setAlpha(double alpha) { _filter->setAlpha(alpha); }
    double getAlpha() const { return _filter->getAlpha(); }
    void setBeta(double beta) { _filter->setBeta(beta); }
    double getBeta() const { return _filter->getBeta(); }
    void setDamping(double d) { _filter->setDamping(d); }
    double getDamping() const { return _filter->getDamping(); }
    void setTrendLimit(double limit) { _filter->setTrendLimit(limit); }
    double getTrendLimit() const { return _filter->getTrendLimit(); }
    
};
#endif // PIDCONTROLLER_H