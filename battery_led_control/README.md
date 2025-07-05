

# 🔋 Battery-LED Panel Control using ROS2 Services and Topics
This project simulates a basic Battery and LED Panel System using ROS2 Humble, showcasing how services and topics work together in a distributed robotic application. It demonstrates effective ROS2 communication patterns including custom message and service definitions, node management, service-client interaction, and simulated state transitions. This code is solely developed my me. 
[Activity is a part of ROS2 Beginner Course (Jazzy): by Edouard Renard]

## 📌 Problem Statement
I simulated the system with **2 nodes**:

**Battery Node**: Monitors and simulates battery status over time.  

**LED Panel Node**: Displays LED status and responds to service calls to update LEDs.

![Image1](img1.png)

When the battery is empty after **4 seconds**, it requests the LED panel to turn **ON** an **LED-3**. 

![Image1](img2.png)

When the battery is full again after **6 seconds**, it requests the panel to turn **OFF** the **LED-3**. 
![Image1](img3.png)

This cycle repeats indefinitely, until explicitly interrupted by keyboard.  


For this server and topic specific **Interfaces** have been created for task execution.

