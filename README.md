# Airflow Calibration Test Bench

<p align="left">
  <img src="testbench1.png" width="600" height= "300">
</p>  
<p align="left">
  <img src="testbench2.png" width="600" height= "240">
</p>

A high-precision, modular 125mm wind tunnel designed for the calibration and validation of airflow meters. This rig serves as the absolute reference for the Zero-Pressure Airflow Meter project.

## 1. Project Overview

The Calibration Test Bench establishes a known, stable volumetric flow rate ($m^3/h$) by drawing air through a laboratory-grade Venturi nozzle. It allows the user to set a fixed fan power and use the resulting flow as a "Golden Standard" to calibrate secondary measurement devices.

---

## 2. Hardware & Electronics

The rig uses an **ESP32-WROOM** to process environmental data and differential pressure, calculating the reference flow in real-time.

### Components

* **Sensor 1:** SDP800-500PA (Differential pressure across the Venturi).
* **Sensor 2:** BME280 (Ambient temperature, humidity, and barometric pressure).
* **Display:** 1.3" OLED (SH1106) for real-time reference data.
* **Encoder:** Rotary encode with pushbutton for speedcontrol and menu navigation.
* **Actuator:** Arctic S12038-8K High-Static Pressure Fan (12V PWM).

### ESP32 Pinout

| Component | Function | GPIO |
| --- | --- | --- |
| **I2C SDA** | Data | **21** |
| **I2C SCL** | Clock | **22** |
| **Fan PWM** | Speed Control | **27** |
| **Encoder** | Input (Menu/Power) | **18, 19, 5** |

---

## 3. Mechanical Design

### Modular 125mm Construction

The rig is built from 125mm diameter sections, optimized for standard ventilation parts:

1. **Inlet Interface:** A 500x500mm mounting panel for air valves.
2. **Flow Conditioning:** Dual honeycomb straighteners to eliminate swirl and ensure a laminar profile before the Venturi.
3. **The Venturi Core:** A 3-part 3D-printed assembly ($D=120\text{mm}$, $d=90\text{mm}$).
4. **Pressure Sensing:** Dual piezometric (averaging) rings at the inlet and throat for stable readings.
5. **Drive Section:** A high-power server fan capable of overcoming the resistance of filters and valves.

---

## 4. Calibration Physics

The reference flow $Q$ is calculated using the ISO 5167 standard principles, corrected for the actual air density at the time of measurement.

### Air Density ($\rho$)

Density is calculated using the **Magnus-Tetens** formula to account for water vapor:


$$\rho = \frac{p_{dry}}{R_d \cdot T} + \frac{p_{vapor}}{R_v \cdot T}$$

### Volumetric Flow ($Q$)

$$\text{Reference } Q = 3600 \cdot C_d \cdot A_{throat} \cdot \sqrt{\frac{2 \cdot \Delta P}{\rho \cdot (1 - \beta^4)}}$$

*Where $C_d$ is the discharge coefficient, determined during initial rig validation (typically 0.975).*

---

## 5. Usage: Calibrating the Flow Meter

This rig is designed to be used in conjunction with the **Zero-Pressure Airflow Meter** interactive calibration modes.

### Calibration Workflow:

1. **Establish Reference:** Run the Test Bench fan at a fixed power (e.g., 50%) and note the reference flow (e.g., $120.0 \text{ m}^3/h$).
2. **Attach Device Under Test (DUT):** Place the Zero-Pressure Flow Meter against the rig's inlet.
3. **Zero-Compensation:** Use the Flow Meter's `Tune Zero-Comp` menu to shift its setpoint until the Test Bench returns to exactly $120.0 \text{ m}^3/h$.
4. **Flow Gain:** Use the `Tune Flow` menu to match the Flow Meter's display to the Test Bench's reference value.

### Traceability & Validation
  
The reference flow of this Test Bench is validated against a calibrated Testo 420.
1. A multi-point measurement was performed across the full PWM range of the fan.
2. The Discharge Coefficient ($C_d$) was empirically determined by comparing the Venturi $\Delta P$ to the calibrated reference flow.
3. This ensures that the Test Bench acts as a traceable "Golden Standard" for secondary devices.

---

## 6. Manufacturing Notes

* **Material:** PETG (0.2mm layer height).
* **Assembly:** Parts are flanged for easy airtight coupling. The Venturi sections are bonded with epoxy to prevent leaks at the pressure rings.
* **Precision:** The internal surface of the Venturi should be sanded or printed with high precision to maintain a consistent $C_d$.

---

## License

MIT License - Created for the open-source ventilation community.
