#include <cmath>
#include <algorithm> // Voor std::clamp

class DoubleExponentialFilter {
  private:
    double _alpha;  
    double _beta;   
    double _level;  
    double _trend; 
    double _maxTrendChange; // De "clamping" limiet 
    int _count;

  public:
    // Constructor with direct alpha/beta values
    // and with met extra parameter for trend-limit (example: 5.0 Pascal/stap)
    DoubleExponentialFilter(double a, double b, double limit = 9999.0) 
      : _alpha(a), _beta(b), _level(0.0), _trend(0.0), _maxTrendChange(limit), _count(0) {}

    // Static helper: calculates alpha based on time constant (tau) and sample time (dt)
    // The larger the tau, the stronger the filtering (slower signal)
    static double calculateAlpha(double tau, double dt) {
      if (tau <= 0.0) return 1.0;
      return dt / (tau + dt);
    }

    double update(double newValue) {
      if (_count == 0) {
        _level = newValue;
        _trend = 0.0;
        _count++;
      } else if (_count == 1) {
        _trend = newValue - _level;
        _level = newValue;
        _count++;
      } else {
        double lastLevel = _level;
        // 1. Update Level
        _level = _alpha * newValue + (1.0 - _alpha) * (_level + _trend);
        // 2. Bereken nieuwe trend
        double rawTrendChange = _level - lastLevel;

        // 3. Pas Clamping toe op de trend-verandering
        // Dit voorkomt dat een absurde sprong de D-actie volledig overstuurt
        double clampedTrendChange = std::max(-_maxTrendChange, std::min(_maxTrendChange, rawTrendChange));
            
        //_trend = _beta * (_level - lastLevel) + (1.0 - _beta) * _trend;
        // 4. Update Trend met de begrensde verandering
        _trend = _beta * (clampedTrendChange) + (1.0 - _beta) * _trend;
      }
      return _level;
    }

    double getTrend() const { return _trend; }
    double getLevel() const { return _level; }
    void reset() { _count = 0; }
};
