#ifndef VENTURI_H
#define VENTURI_H
#include <Arduino.h>

class Venturi {
  public:
    Venturi() {}
    Venturi(float inletDiameter, float throatDiameter, float dischargeCoefficient = 0.975) {
      begin(inletDiameter, throatDiameter, dischargeCoefficient);
    }
    void begin(float inletDiameter, float throatDiameter, float dischargeCoefficient) {
        _inletDiameter = inletDiameter;
        _throatDiameter = throatDiameter;
        _dischargeCoefficient = dischargeCoefficient;
        _betaCoefficient = 1 - pow(throatDiameter / inletDiameter, 4);
        _areaThroat = (PI * pow(throatDiameter, 2)) / 4.0;
    }
    void loop(float deltaP,  float absPressurePa=0, float tempC=20.0, float humidityPct=50.0) {
      if (absPressurePa > 0) {   
        setRho(absPressurePa, tempC, humidityPct);
      }
      if (deltaP <= 0) {
        _smoothedFlow = _flow = 0.0;
        return;
      }
      if (_smoothedFlow == 0.0) {
        _smoothedFlow = _flow = getFlow(deltaP); // Initialize smoothed flow with the first reading
      } else {
        _flow = getFlow(deltaP);
        // Optional: Implement flow smoothing here if needed
        _smoothedFlow = _flow * _smoothedFlowFactor + _smoothedFlow * (1.0 - _smoothedFlowFactor); // Simple exponential smoothing
      }  
    }
    float setRho(float absPressurePa, float tempC=20.0, float humidityPct=50.0) {
      // Calculate the density of humid air using the ideal gas law and accounting for humidity
      float R = 287.05; // Specific gas constant for dry air in J/(kg·K)
      float T = tempC + 273.15; // Convert temperature to Kelvin
      float P = absPressurePa; // Absolute pressure in Pascals

      // Calculate the saturation vapor pressure using the Magnus formula
      float Es = 610.94 * exp((17.625 * tempC) / (tempC + 243.04)); // Saturation vapor pressure in Pa
      float E = Es * (humidityPct / 100.0); // Actual vapor pressure in Pa
    
      // Calculate the density of humid air
      _rho = (P - E) / (R * T) + E / (461.495 * T); // 461.495 is the specific gas constant for water vapor
      return _rho;  // return the density in kg/m³
    }
    float getFlow () {
      return _flow;
    }
    float getFlow(float deltaP) {
      if (deltaP <= 0) return 0.0;

      // Calculate the velocity at the throat using Bernoulli's equation
      float velocityThroat = sqrt((2 * deltaP) / (_rho * _betaCoefficient));
      
      // Calculate the volumetric flow rate using the discharge coefficient and area of the throat
      float flowM3s = _dischargeCoefficient * _areaThroat * velocityThroat;

      return flowM3s * 3600.0; // Convert from m³/s to m³/h
    }
    float getSmoothedFlow() const {
      return _smoothedFlow;
    }
    void setInletDiameter(float diameter) {
      _inletDiameter = diameter;
      _betaCoefficient = 1 - pow(_throatDiameter / _inletDiameter, 4);
    }
    float getInletDiameter() const {
      return _inletDiameter;
    }
    void setThroatDiameter(float diameter) {
      _throatDiameter = diameter;
      _betaCoefficient = 1 - pow(_throatDiameter / _inletDiameter, 4);
      _areaThroat = (PI * pow(_throatDiameter, 2)) / 4.0;
    }
    float getThroatDiameter() const {
      return _throatDiameter;
    }
    void setDischargeCoefficient(float cd) {
      _dischargeCoefficient = cd;
    }
    float getDischargeCoefficient() const {
      return _dischargeCoefficient;
    }
    void setSmoothedFlowFactor(float factor) {
      _smoothedFlowFactor = factor;
    }
    float getSmoothedFlowFactor() const {
      return _smoothedFlowFactor;
    }
  private:
    float _inletDiameter;        // D
    float _throatDiameter;       // d
    float _dischargeCoefficient; // Cd
    float _betaCoefficient;      // Cb = 1 - (d/D)^4
    float _areaThroat;           // A2 = π * (d/2)^2
    float _rho = 1.225;          // Air density in kg/m³ (at standard conditions)
    float _flow = 0.0;         // Current flow in m³/h
    float _smoothedFlow = 0.0;   // For flow smoothing (optional)
    float _smoothedFlowFactor = 0.1; // Smoothing factor for exponential smoothing
};

#endif // VENTURI_H