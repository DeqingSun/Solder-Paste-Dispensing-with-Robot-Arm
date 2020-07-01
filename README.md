# Solder Paste Dispensing with Robot Arm

This is an open-sourced project that dispenses solder paste on PCB with a robot arm and Raspberry Pi. 

[![Demo Video](https://raw.githubusercontent.com/DeqingSun/Solder-Paste-Dispensing-with-Robot-Arm/master/img/videoImg.jpg)](https://vimeo.com/433447125 "Demo Video")

This project uses an off-shelf Dobot Magician as the robot arm part. The customized end effector dispenses solder paste. And another laser jig calibrates the position of the robot arm. 

![Dobot Magician with solder paste dispenser](https://raw.githubusercontent.com/DeqingSun/Solder-Paste-Dispensing-with-Robot-Arm/master/img/robotArm.jpg)

![PCB with solder paste applied](https://raw.githubusercontent.com/DeqingSun/Solder-Paste-Dispensing-with-Robot-Arm/master/img/pcbWithPaste.jpg)

## Hardware 

* Dobot Magician

A consumer-grade robot arm with an embed control box. 

* Raspberry Pi 3B

The Raspberry Pi is pretty convenient in playing with Python code. The Raspberry Pi hosted all files and run a python script. Users can use a browser on a computer or smartphone to control the robot arm with the Raspberry Pi. It's very easy to set up the Jupyter Notebook on the new version of Raspberry Pi OS, which saves a lot of time in debugging.

* 3D Printed Laser Jig

The laser jig uses 2 pairs of laser modules and photoresistors to calibrate the robot arm. There is also an auto-level sensor to fine-tune the Z-axis. The whole jig is controlled by an Arduino Micro Pro.

* 3D Printed solder paste dispenser

The solder paste dispenser is syringe driven by a stepper motor. There is also a heater attached to the needle to reduce the viscosity of the paste.

## Detail of the solder paste dispenser

![img of dispenser](https://raw.githubusercontent.com/DeqingSun/Solder-Paste-Dispensing-with-Robot-Arm/master/img/model.jpg)

The solder paste dispenser uses a NEMA 42 stepper motor. Dobot Magician has 2 external motor drivers. However, the driving current is not adjustable. So a heatsink may be needed. The stepper is attached to a T5*1 leading screw with coupling, and the flange nut is attached to the piston with an adaptor and a #4-1/2 sheet metal screw. The syringe is a Nordson EFD 30cc one. And the needle is an 18 gauge one. Also, there is a 70C PTC heating element that keeps the needle warm to reduce the viscosity of the paste.

## Detail of the laser jig

![laser jig](https://raw.githubusercontent.com/DeqingSun/Solder-Paste-Dispensing-with-Robot-Arm/master/img/laserFixture.jpg)

The laser jig is taped on a silicon pad for extra friction and Z-axis tolerance.  

## Why using a robot arm?

Most solder paste dispenser uses a Cartesian platform. A Cartesian platform is generally more accurate and rigid. But the machine is generally much bigger than its working area. The robot arm saves space. When the robot arm is not used, it only takes 40 sq in of desk space. Pretty neat for prototyping.    

## Why a laser jig?

A robot arm often has good repeatability and bad accuracy. With a laser-guided jig, the robot arm can get the position and orientation of target PCB accurately. 
