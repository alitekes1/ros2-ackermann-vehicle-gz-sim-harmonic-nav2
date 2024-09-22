# Ackermann Steering Custom Vehicle Simulation with ROS2, Gazebo Sim (Harmonic)
This project involves the simulation of a custom vehicle with Ackermann steering capabilities, developed using ROS2 and the Gazebo Sim Harmonic environment. 
The vehicle is equipped with multiple sensors including LiDAR, front/rear cameras, and left/right cameras, all of which communicate via ROS2. 
The model is visualized in RViz2, where the sensors and robot state are displayed. The goal is to further enhance this project using the Nav2 stack for autonomous navigation. 
Additionally, this simulation is one of the first implementations of an Ackermann steering vehicle in the Gazebo Sim Harmonic environment.
## Features  - **Ackermann Steering**:
  Custom vehicle model with Ackermann steering dynamics. 
- **ROS2 Communication**: Sensor data and control signals are fully integrated with ROS2.
- **Sensors**:  
- IMU
- Odometry
- **LiDAR**: Mounted for obstacle detection.  
- **Cameras**:      
- Front-facing
- Rear-facing     
- Left-side     
- Right-side 
- **Visualization**:    
- Vehicle model and sensor data visualized in RViz2. 
- **Autonomous Navigation**: Future integration with Nav2 for path planning and autonomous control.  
- ## Requirements  
- **ROS2** (Humble) 
- **Gazebo Sim Harmonic**
- **Rviz2** 
- **Nav2** (to be integrated)  
## Future Work

- **Nav2 Integration**: The next phase of this project will include the Nav2 stack for autonomous navigation and path planning.
- **Deep Reinforcement Learning (DRL)**: Future efforts may focus on training the vehicle for complex scenarios using DRL algorithms.

## Gallery
![[Screenshot from 2024-09-23 00-09-48.png]](https://github.com/user-attachments/assets/dd5604c6-014e-4a7a-9a2f-c4dd237abb37)

| **Gazebo Sim Harmonic**                      | **RViz2**                                   |
| -------------------------------------------- | -------------------------------------------- |
| ![[Screenshot from 2024-09-23 00-13-03.png]](https://github.com/user-attachments/assets/1d2b56f7-34c1-4b01-9a85-fb01ceab5bd6) | ![[Screenshot from 2024-09-23 00-09-04.png]](https://github.com/user-attachments/assets/ba6853fd-4143-4b4d-bbc6-072895e4c75e) |
| ![[Screenshot from 2024-09-23 00-12-13.png]](https://github.com/user-attachments/assets/477cce7b-995b-471e-a684-4d82bee0fc34) | ![[Screenshot from 2024-09-23 00-15-04.png]](https://github.com/user-attachments/assets/bf9ad916-14a6-4b62-a799-4169a767e4dd) |

