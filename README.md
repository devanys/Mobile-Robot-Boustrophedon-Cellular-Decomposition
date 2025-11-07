https://github.com/user-attachments/assets/7f7e4197-ce6a-435c-b59d-b1a2c960b876

<img width="1448" height="987" alt="Screenshot 2025-11-06 014548" src="https://github.com/user-attachments/assets/27788a6f-073c-49b7-9af0-bd5e66ae3637" />

<img width="934" height="786" alt="Screenshot 2025-10-27 152738" src="https://github.com/user-attachments/assets/455d71fd-96d0-450b-a0a9-876bb470f4eb" />

# 🤖 Mobile-Robot-Boustrophedon-Cellular-Decomposition

A **four-wheeled mobile robot simulation (Ackermann steering)** using the **Pure Pursuit algorithm** to follow a **Coverage Path Planning (Lawn Mower / Boustrophedon)** trajectory in a **PyBullet 3D environment**.  
The robot is equipped with a **front saw (simulated)** to represent a cleaning or grass-cutting mechanism.

---

## 🧠 Key Features
- 🧭 **Pure Pursuit Controller** – Steers the robot dynamically toward a lookahead point.  
- 🧱 **Path Generator (Lawn Mower Pattern)** – Generates efficient coverage paths over the working area.  
- 🚗 **Ackermann Kinematics** – Simulates realistic front-wheel steering angles.  
- 🌈 **Dynamic Visualization**
  - Green line → target path  
  - Blue circle → lookahead radius  
  - Gray circle → sensor radius  
  - Black line → robot heading  
  - Red dot → lookahead point  
- 🧾 **Real-time Info Panel** – Displays position, yaw, steering angle, and curvature radius.  
- 🧩 **Interactive 3D simulation using PyBullet GUI**

---

## ⚙️ Main Parameters

| Parameter | Description | Default |
|------------|--------------|----------|
| `AREA_W`, `AREA_H` | Simulation area (meters) | 10.0 × 6.0 |
| `LANE_SPACING` | Distance between coverage lanes (m) | 1.0 |
| `LOOKAHEAD` | Lookahead distance for Pure Pursuit | 0.8 |
| `V_BASE` | Base velocity (m/s) | 1.6 |
| `MAX_STEER_DEG` | Maximum steering angle | 60° |
| `SENSOR_RADIUS` | Sensor visualization radius (m) | 1.6 |
| `DT` | Simulation time step | 1/60 s |
