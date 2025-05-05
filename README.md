# Autonomous Parking in CARLA Simulator

This project demonstrates an autonomous parking system in the [CARLA](https://carla.org/) simulator using a Tesla Model 3. It integrates real-time camera input and vehicle control to detect and park into a parking slot automatically.

---

## Features

-  Detects parking zones using camera and OpenCV
-  Performs a reverse parking maneuver autonomously
-  Spawns ego and parked vehicles in CARLA environment
-  Uses synchronous simulation for better control

---

##  Demo

The script spawns a Tesla Model 3, detects a parking spot, and performs a two-step reverse parking into the space between two parked cars.


https://github.com/user-attachments/assets/b3eec643-c2c2-473b-954a-743bff4336ab


---

## File Overview

- `park.py` - Main simulation and parking logic
- Uses CARLA Python API and OpenCV for simulation and perception

---

##  Requirements

- [CARLA Simulator 0.9.11](https://github.com/carla-simulator/carla/releases/tag/0.9.11)
- Python 3.7+
- Python Libraries:
  ```bash
  pip install numpy opencv-python
