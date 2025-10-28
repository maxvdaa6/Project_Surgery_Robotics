![University of Barcelona Logo](././Images/Session1/figure1.png)

## Degree in Biomedical Engineering
## ROBOTICS AND CONTROL OF BIOMEDICAL SYSTEMS
# **Surgery Robotics Project**
### Laboratory sessions 3 & 4: Biorobotics Laboratory Surgery Robotic system Project improvements

---

In the previous session you have seen the main performances of your first prototype of a DaVinci surgery system.

You have seen the complexity to perform a simple surgery process in roboDK simulation environment.

### The objective of this laboratory session will be:
- Identify the main limitation performances of your first prototype
- Propose and design useful solutions to improve the system
- Implement your own improved surgical robotic system
- Verify the improvements in the system performances for the same surgical process in roboDK simulation environment

### Proposed improvements for Surgery Robotic system prototype in Biorobotics Lab

The main improvements are based on:
- Servomotors module:
    - Review how the RPY angles are applied to the four servomotors to obtain the desired Gripper orientation. Make necessary corrections to:
        - Apply Roll angle from initial servomotor position in 90º
        - Apply Pitch angle from initial servomotor position in 90º
        - Apply `Yaw angle variation` from initial servomotor position in 90º to be independent of the geographical North 
    - the torques are already read on the four servomotors to detect the gripper contact with the tissue.
        ![ServomotorsModule](././Images/Session1/Servos1.png)
        ![ServomotorsModule](././Images/Session1/Servos2.png)
        ![ServomotorsModule](././Images/Session1/Servos3.png)
        - Add modifications in main.cpp program to send these torques to the gripper and PC.
- Endowrist module:
    - No needed changes are required
- Gripper module:
    - Add modifications in main.cpp program to receive the torques from the Servomotors module.
    - Add modifications in main.cpp program to drive the [vibration motor](https://www.amazon.es/dp/B0B82HS49C) according to the torques read to feel the contact with the tissue.
        - You have to add on void setup() the following code:
        ```cpp
        // Configure PWM for the vibration motor (channel 0)
        ledcSetup(0, 5000, 8); // Channel 0, frequency 5kHz, resolution 8 bits
        ledcAttachPin(vibrationPin, 0); // Attach the vibration motor to channel 0
        ````
        - You have to add on void receiveTorquesUDP() the following code:
        ```cpp
        // Vibration motor control based on torque values
        float totalTorque = Torque_roll1 + Torque_pitch + Torque_yaw;
        // Convert torque to PWM value (0-255)
        int vibrationValue = constrain(totalTorque * 2.5, 0, 255); // Adjust the scaling factor as needed
        ledcWrite(0, vibrationValue); // Set the PWM value for the vibration motor
        Serial.print("Vibration motor value: ");
        Serial.println(vibrationValue); 
        ```
- PC module:
    - Add Servomotor torques reading
        - Modify the Python program to add numeric torque values in TKinter pop-up window
        - Add a "buttom" with color code to indicate the torque level in the TKinter pop-up window
    - (Optional) Real time control of the UR5e robot arm:
        - Create a copy of the `InitSurgeryRobotic_simulation.rdk` file to `InitSurgeryRobotic_real.rdk` file. Implement the UR5e movements using Python based sockets program.
        - Modify the Python program to send the UR5e robot arm orientation commands according to the Endowrist tool orientation.
    
![Proposed Project Improvements](././Images/Session1/ProjectImprovements2.png)

### Laboratory session 3: Improvements experimental validation
The proposed tasks for this session are:
- Save the ESP32 modules with the new programs you have created at home implementing the described improvements
- Try to perform again the suture process in simulation according to the following video:
[![suture process in simulation](Images/Session1/training.png)](https://youtu.be/1t3-Ggcp_Hg?feature=shared)

Show and explain the system performances to your teacher.

### Laboratory session 4: Project Presentation

Each Laboratory group has up to 10 minutes project presentation with:

- Project presentation: with some slides summarising what you have learned and what you have done in this project
- Software and Hardware live demonstration of the assistive robotic tasks designed in the previous session.
- All the students have to contribute to the final presentation