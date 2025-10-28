# Adaptive-Cruise-Control-System

---


## Overview

This project models and simulates an Adaptive Cruise Control (ACC) system using MATLAB/Simulink, focusing on two key functionalities: **vehicle speed-maintenance and distance-keeping** and **lane-centering control**.  

The system integrates realistic **longitudinal and lateral vehicle dynamics**, including aerodynamic drag, rolling resistance, and tire friction, to ensure physically accurate performance.  
Two control strategies,**PID** and **LQR**, were designed and compared under various speeds, slopes, and road conditions to analyze trade-offs between **stability**, **responsiveness**, and **energy efficiency**. 

Through simulation experiments, the ACC system successfully demonstrated smooth speed regulation, stable lane tracking, and reliable control responses comparable to modern autonomous driving features.


---


## Features
- **Vehicle Dynamics Modeling**: Longitudinal and lateral dynamics with aerodynamic drag, rolling resistance, and tire friction.  
- **PID Speed Control**: Maintains target speed under varying slopes and drag conditions.  
- **LQR Speed Optimization**: Minimizes control effort for smoother transitions and improved energy efficiency.  
- **Lane-Centering Control**: PID and state-space modeling to minimize lateral deviation.  
- **Comparative Simulation**: Evaluates controller responses under different speeds and road gradients.


---


## System Architecture
The system consists of four main functional modules implemented in MATLAB/Simulink:

1. **Vehicle Dynamics Module**: Simulates longitudinal motion through aerodynamic drag, rolling resistance, and tire friction models. It computes real-time velocity by integrating acceleration over time.
2. **PID Control Module**: Regulates the throttle input to minimize the difference between target and actual speed. Also used in lane-centering to correct lateral deviation.
3. **LQR Control Module**: Provides an alternative speed control structure optimized by minimizing the quadratic cost of state error and control effort.
4. **Lane-Centering System**: Uses sensor-based feedback to adjust the steering angle based on lateral position and yaw rate, maintaining lane stability.

These subsystems are interconnected within Simulink:
- The **longitudinal subsystem** includes drag and slope compensation.
- The **lateral subsystem** is represented by a **state-space model** describing steering and yaw dynamics.
- Closed-loop feedback links each control module to real-time vehicle responses.


---

---

## Technical Details

### Vehicle Dynamics
The longitudinal motion of the vehicle is modeled in Simulink using the following physical relation:

**Longitudinal Dynamics Equation:**
![Vehicle_Dynamics](Img/vehicle_dynamics.png)

where  
- Fi: traction force  
- CD: drag coefficient  
- ρ: air density  
- A: frontal area  
- μ: rolling resistance coefficient  
- θ: road slope angle  
- m: vehicle mass  

This model accounts for aerodynamic drag, rolling resistance, tire friction, and slope,  
allowing the simulation to reflect realistic driving conditions.

**Simulink Implementation:**

![Vehicle Dynamics Model1](Img/VehicleDynamics1.png)
![Vehicle Dynamics Model2](Img/VehicleDynamics2.png)

---

### PID Control System
The **PID controller** was designed for both longitudinal speed and lateral lane-centering control.  
It continuously adjusts throttle input based on the speed error between the target and actual velocity.  
A second PID loop is used to correct steering angle deviation, maintaining vehicle alignment within the lane.

**Simulink Block Diagram:**

![PID Control Diagram](Img/PID_speed_controller.png)

Performance tuning focused on achieving a compromise between fast response and minimal overshoot.

---

### LQR Control System
An **LQR (Linear Quadratic Regulator)** controller was implemented as an alternative to PID for speed control.  
It optimizes the control input by minimizing a performance index that balances speed error and control effort,  
achieving smoother and more energy-efficient speed transitions under varying road slopes.

**Simulink Block Diagram:**

![LQR Control Diagram](Img/LQR_speed_controller.png)

Compared with PID, the LQR system showed reduced oscillation and smoother acceleration.


---

## Simulation Setup
| Condition | Initial Speed | Target Speed | Controller | Result |
|------------|----------------|---------------|-------------|----------|
| Flat road | 0 km/h | 100 km/h | PID | Fast response, minor overshoot |
| Inclined road | 30 km/h | 110 km/h | LQR | Smooth, stable, minimal control effort |


---

## Results
- **PID Control:** Quick convergence but slightly higher overshoot.  
- **LQR Control:** Slower response but smoother and more energy-efficient.  
- **Lane-Centering:** Effective with minimal lateral error (< 0.05 m).  

Simulation verified that **LQR provides superior stability** while **PID offers faster transient response**.


---

## 🧩 Future Work
- Integration of **Model Predictive Control (MPC)**.  
- **Hardware-in-the-Loop (HIL)** implementation.  
- Unified control for combined longitudinal and lateral dynamics.
