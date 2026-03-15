# Airflow Calibration Test Bench

<p align="left">
  <img src="testbench1.png" width="600" height= "300">
</p>  
<p align="left">
  <img src="testbench2.png" width="600" height= "240">
</p>

A high-precision, modular 125mm wind tunnel designed for the calibration and validation of airflow meters. This rig serves as the absolute reference for the [Zero-Pressure Airflow Meter project](https://github.com/Cortronic/airflowmeter).

## 1. Project Overview

The Calibration Test Bench establishes a known, stable volumetric flow rate ($m^3/h$) by drawing air through a laboratory-grade Venturi nozzle. It allows the user to set a fixed fan power and use the resulting flow as a "Golden Standard" to calibrate secondary measurement devices.

---

## 2. Hardware & Electronics

The rig uses an **ESP32-WROOM** to process environmental data and differential pressure, calculating the reference flow in real-time. To eliminate extreme static underpressure when testing restricted valves, the rig utilizes a coordinated dual-fan setup.

### Components

* **Sensor 1:** SDP810-500PA (Differential pressure across the Venturi).
* **Sensor 2:** SDP810-125PA (Differential pressure at the inlet of the Venturi).
* **Sensor 2:** BME280 (Ambient temperature, humidity, and barometric pressure).
* **Display:** 1.3" OLED (SH1106) for real-time reference data.
* **Encoder:** Rotary encode with pushbutton for speedcontrol and menu navigation.
* **Actuator:** 2 x Arctic S12038-8K High-Static Pressure Fan (12V PWM).

### ESP32 Pinout

| Component | Function | ESP32 GPIO | Notes |
| --- | --- | --- | --- |
| **I2C Bus 0** | Data / Clock | **21 / 22** | BME280, OLED, SDP800 (Venturi) |
| **I2C Bus 1** | Data / Clock | **25 / 26** | SDP800 (Zero-Pressure Sensor) |
| **Encoder** | CLK / DT / SW | **2, 4, 15** | Menu & Calibration |
| **PWM Pull Fan** | Speed Control | **27** | 25kHz PWM signal |
| **PWM Push Fan** | Balance Control | **27** | 25kHz PWM signal |

---

## 3. Mechanical Design

### Modular 125mm Construction

The rig is built from 125mm diameter sections, optimized for standard ventilation parts:

1. **Inlet Interface:** A 500x500mm mounting panel for airextract valves.
2. **Push Drive Section:** A high-power server fan capable of overcoming the resistance of filters and valves.
3. **Flow Conditioning:** Dual honeycomb straighteners to eliminate swirl and ensure a laminar profile before the Venturi.
4. **The Venturi Core:** A 3-part 3D-printed assembly ($D=120\text{mm}$, $d=90\text{mm}$).
5. **Pressure Sensing:** Dual piezometric (averaging) rings at the inlet and throat for stable readings.
6. **Pull Drive Section:** A high-power server fan capable of overcoming the resistance of filters and valves.
7. **Outlet Interface:** A 500x500mm mounting panel for airsupply valves.

---

## 4. Active Static Pressure Balancing
The Test Bench features an advanced "Active Balance" control loop to ensure measurement linearity regardless of inlet restriction.

* **Primary Loop (Flow Control):** The Exhaust (Pull) fan is set to a specific PWM or RPM to achieve the target flow rate.
* **Secondary Loop (Pressure Balance):** A dedicated SDP810 sensor monitors the static pressure difference between the Venturi inlet and the ambient room.
* **Active Compensation:** A PID controller adjusts the Inlet (Push) fan in real-time. If a restrictive valve causes a pressure drop, the Push fan increases power until the internal static pressure returns to 0.0 Pa (Atmospheric Neutral).

**Result:** The Venturi operates in a "Stagnation-Free" zone, eliminating the "Negative Delta-P" phenomenon and ensuring laboratory-grade accuracy even at >90% valve restriction.

---

## 5. Calibration Physics

The reference flow $Q$ is calculated using the ISO 5167 standard principles, corrected for the actual air density at the time of measurement.

### Air Density ($\rho$)

Density is calculated using the **Magnus-Tetens** formula to account for water vapor:


$$\rho = \frac{p_{dry}}{R_d \cdot T} + \frac{p_{vapor}}{R_v \cdot T}$$

### Volumetric Flow ($Q$)

$$\text{Reference } Q = 3600 \cdot C_d \cdot A_{throat} \cdot \sqrt{\frac{2 \cdot \Delta P}{\rho \cdot (1 - \beta^4)}}$$

*Where $C_d$ is the discharge coefficient, determined during initial rig validation (typically 0.975).*

---

## 6. Usage: Calibrating the Flow Meter

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

## 7. Manufacturing Notes

* **Material:** PETG (0.2mm layer height).
* **Assembly:** Parts are flanged for easy airtight coupling. The Venturi sections are bonded with epoxy to prevent leaks at the pressure rings.
* **Precision:** The internal surface of the Venturi should be sanded or printed with high precision to maintain a consistent $C_d$.

---

## License

MIT License - Created for the open-source ventilation community.
