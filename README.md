# VibroTouch
## 1. General description
### 1.1 Hardware:
**VibroTouch ROS package** - creates a network of several devices connected together to perform vibrational frequency analysis of a variety of objects. The system includes severral peripherals:
1) **Teensy 3.2** - analog data acquisition from accelerometers and transmission to PC at 500Hz (5 samples at a time => enhanced data transmission frequency 2500 samples per second => 2.5 kHz)
2) **ADXL 345** - accelerometer sensors for vibration acquisitions
3) **Haptuator Mark II** - high-frequency vibro motors, used for the vibration generation (90-1000 Hz)
4) **Schunk EGN100** - high precision robotic gripping system
5) **Wittenstein HEX21 F/T** - force sensor used to detect actual contact time stamps, for future comparisons with VibroTouch detection time
### 1.2 Software:
1) **Teensy Loader + Arduino IDE** - for programming of the data acquisition using Teensy
2) **ROS Kinetic** - a framework for interfacing with all of the peripheral of the system. Below is the visual RQT node graph for VibroTouch ROS package with all required nodes in an active condition:

![vibroTouch _scheme](https://user-images.githubusercontent.com/52815976/215746474-4692564a-7bcf-40cd-94a6-fa08f013c8b3.PNG)

*Some notes regarding the figure above:*<br>
*imp_fft.. in the center is the rt_gui.py node*<br>
*rectangles - topics, circles - nodes*
### 1.3 ROSTOPIC list:
/chatter - data received via rosserial from the teensy<br>
/exp_status - connects the main python node (GUI) and force_control.cpp and alerts on the status of the current experiment<br>
/float_grip - publishes gripper data for recording from the original Schunk topic<br>
/forceFloat32 (\*/force_pub in the diagram) - republishes the force data from the wittenstein topic as a regular float32<br>
/send_force - sends desired force value or a command value to force_control.cpp<br>
/setData - a topic incorporated in the Schunk library that commands the gripper node on a lower level<br>
/wittenstein_topic - the original Wittenstein topic coming in from the force sensor<br>

## 2. ROS Node description:
For the description below the following markers are implemented for easied navigation and inderstanding:
- ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) `data being sent externally FROM the corresponding node`
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) `data being received TO the corresponding node`
### 2.1 vibroTouch -> force_pub (C++ file)
Republishes the data from wittenstein_topic in the form of a regular Float32 array instead of a custom Wittenstein message (because wittenstein packages are written in C++ and some of the nodes in the active_glove package are written in python and require data from wittenstein too).
- subscribes to **witenstein_topic** and get force data from here
- publishes to **forceFloat32** in the form of floats
### 2.2 vibroTouch -> force_control (C++ file)
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) **wittForceZ** - variable storing data from the wittenstein_topic on the Z-axis force
- **forceGoal** - the desired force that is received from the “send_force” topic (positive float - desired force, -10.0 - release signal, -5.0 - launches experimental procedure (! DEPRECATED !)
- **oldPosition** - a variable storing the gripper position value from the previous spin (loop)
- **newPosition** - a variable storing the gripper position value that is to be sent during the current spin (loop)
- **forceError** - the deviation of the current force from the goal force
- **positionStep** - a correction value to be introduced to the current position of the gripper, that is used in order to adjust when force control is used
- **forceGoalIndexCounter** - a threshold value that is waiting to be 200 as a counter that measures when the current force is close to the steady state (desired force)
- **forceControlStart** - a boolean flag indicating the start of force control procedure (that the force goal has been received from the user) 1 - procedure has started, 0 - procedure has ended
- **forceGoalHandle** - a handle variable for the desired force
- **firstLoop** - a boolean variable indicating whether the current force loop is the first one (0 - the current loop is the first one, 1 - the first loop has already passed)
- **goalAchieved** - a boolean variable indicating whether the desired force is achieved (0 - desired force is yet to be met, 1 -  the desired force is achieved and the force control task is finished)
- **expStatus** - a variable used in the custom experimental procedure (used for exp_type = 1 in rt_gui.py), takes in the values 0 - the start of the experiment/nothing to do yet,1 - squeezing until 5N,2 - 5N reached and going back
- **gripperCount** - the variable that is controlling the state of the “touch” experiment. Takes in the values of 0 - experimental procedure has not been started yet, 1-110 - experimental procedure has been started (the touch experiment moves the gripper from 20.0 to 18.9 mm)
- **gripperRelease** - a boolean variable indicating the current gripper movement direction in the “touch” experiment (0 - squeezing in, 1 - releasing)
