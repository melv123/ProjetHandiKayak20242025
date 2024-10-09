# Handi-kayak

In this repository is the source code developed during a school project to allow a blind person to practice kayak without the help of anybody else.

The solution is based on on an IMU and a GPS to detect the position in the corridor and a buzzer to tell  the person whether he must turn right or left. Please refer to the report for more details about the solution.

The code is made to work with ROS2 Humble. We made two packages, the first *imu_package* to treat the data and the second *buzzer_package* to communicate with the person.

This is a schema that shows the interactions between the nodes.

<p align="center">
<img src="https://github.com/Axel927/Handi-kayak/assets/77966063/7fd969ae-529a-489b-8c88-ef2cc6af532b" width="800">
</p>

# Data treatment

Get the data from the sensors and treat them to calculate the position.

# Buzzer communication

A frequency is set to tell the person to turn right and another one to turn left. Then a bip is made with the signal to tell how much the person must turn.
