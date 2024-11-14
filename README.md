# FreeRTOS IRC "Car"

This project implements an IR controlled car using an Arduino and the FreeRTOS real-time operating system.  The car uses an ultrasonic sensor for distance measurement for basic obstacle avoidance and allows for speed control.

<img src="/images/the%20abomination.jpg" alt="The Abomination" width="300" height="200"/>

**Description**: This code controls an RC car using an Arduino board. It interprets IR remote signals for navigation and employs ultrasonic sensors for distance measurement.

**Hardware Components**: Arduino Mega 2560, L298N Motor Driver, 2x DC Motors, HC-SR04 Ultrasonic Sensor, Generic IR Remote.

**Functionality**: The car can move forwards, backwards, and turn left/right based on IR remote commands, it also has three different speedlevels. It includes obstacle avoidance using the ultrasonic sensor.
