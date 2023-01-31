# VibroTouch GUI 1.0 [Teensy]
- Requirements:
- - ```ROS Kinetic```
- - ```Python 2.7```
### Overview
- [0. Fast Start + Command List](https://github.com/s0danissa/VibroTouch#0-fast-start--command-list)
- [1. General package description](https://github.com/s0danissa/VibroTouch#1-general-package-description)
- [2. ROS Node description](https://github.com/s0danissa/VibroTouch#2-ros-node-description)
## 0. Fast Start + Command List
1) Download all of the foulders present in the repository (*except for 'Teensy (.ino) files'*) into the ```$your_catkin_workspace$/src``` foulder
2) ```$ catkin build```
3) Load .ino file to Teensy using Arduino IDE (*the file is in 'Teensy (.ino) files'*)
4) Use below commands to launch and interact with the package nodes:
- ```source /opt/ros/$your_distro$/setup.bash``` - ROS distro sourcing
- ```. ~/$your_catkin_workspace$/devel/setup.bash``` - Catkin workspace sourcing
- ```roscore``` - Roscore launch
- ```roslaunch schunk_ezn64 ezn64_usb_control.launch``` - Schunk EGN100 main control launcher 
- ```rosservice call /schunk_ezn64/reference``` - bringing Schunk EGN100 to the 'reference/zero' position, required for initial callibration after each start-up of the gripper
- ```rosservice call /schunk_ezn64/set_position $velocity$ $position$``` - ros service used for fast interfacing with the Schunk gripper, just set required position and velocity (EX: ```rosservice call /schunk_ezn64/set_position 14.49 5.0```)
- ```rosrun rosserial_python serial_node.py $port$ _baud:=$baud_rate$``` - launches ros-serial node that recieves data from Teensy through the serial bus (EX: ```rosrun rosserial_python serial_node.py /dev/ttyACM1 _baud:=115200```)
- ```rosrun wittenstein wittenstein_main``` - launches the main Wittenstein communication node
- ```rosrun vibro_touch force_pub``` - launches **force_pub** node ([2.1 force_pub (force_pub.cpp)](https://github.com/s0danissa/VibroTouch#21-force_pub-force_pubcpp))
- ```rosrun vibro_touch force_control``` - launches **force_control** node ([2.2 force_control (force_control.cpp)](https://github.com/s0danissa/VibroTouch#22-force_control-force_controlcpp))
- ```rosrun active_glove rt_gui.py``` - launches the main Python GUI **rt_gui.py** ([2.3 rt_gui.py](https://github.com/s0danissa/VibroTouch#23-rt_guipy))
## 1. General package description
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
### 1.3 Included ROS Packages:
- **vibroTouch** - original package with C++ nodes and a Python GUI
- **wittenstein, wittenstein_msgs** - supplementary packages for interfacing with the Wittenstein F/T sensor via ROS
- **schunk_exn64** - package for interfacing with Schunk EGN100 via ROS servies (modified to include custom /float_grip gripper position publishing topic and /setData position and velocity listening from the external nodes topic)
- **rosserial** - package for interfacing Teensy/Arduino with ROS via serial USb bus 
### 1.4 ROSTOPIC list:
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
### 2.1 force_pub (force_pub.cpp)
Republishes the data from wittenstein_topic in the form of a regular Float32 array instead of a custom Wittenstein message (because wittenstein packages are written in C++ and some of the nodes in the active_glove package are written in python and require data from wittenstein too).
- subscribes to **witenstein_topic** and get force data from here
- publishes to **forceFloat32** in the form of floats
### 2.2 force_control (force_control.cpp)
#### GLOBAL VARIABLES DICTIONARY
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
#### ROS MESSAGE VARIABLES
- ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **gripMessage** (Float32MultiArray) - a message that is being sent to the gripper from position control (array of 2 floats, where [0] - speed, [1] - position)
- ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **expStatusData** (Int16MultiArray) -  an array consisting of one int16 that communicates with the exp_status topic (1 - experiment finished, 2 - experiment started)
#### ALGORITHM DESCRIPTION
This node is responsible for executing the experimental procedures. It sends commands to Schunk EGN100 topics to execute certain motions corresponding to the data received from Wittenstein. The main code is divided into three parts corresponding to three pre-defined procedures (all measures are written in mm or mm/s):
- **Procedure 1: One-time custom grip force control** (received force value is positive) - manual grip force control
- - takes time to achieve the desired force is a 0.1 accuracy
- **Procedure 2: Release the grip** (received force value is -10.0) - release/open position
- - fast release of the gripper
- **Procedure 3: Experimental protocol** (received force value is -5.0) - determining vibration feedback of different structures
- - moves until the contact of 0.5 N is established (at speed 2) and slows down after
- - slowly moves to 2N (at 0.5 speed)
- - once the grip of 2N is achieved the experiment starts
- - slowly goes to 5N (at 0.25 speed)
- - once 5N is reached goes back to 2N slowly (at 0.25 speed)
- - 2N is reached and the experiment is finished
- **Procedure 4: Contact detection experiment** (received force value is -15.0) - manual dimension grip
- - slowly moves from 20.0 to 18.9 with a step of 0.01 and a speed of 5
- - then goes back to 18.9 with the same step and speed
### 2.3 rt_gui.py
#### GLOBAL VARIABLES DICTIONARY
- **exp4CounterCSV** - a counter used to update the names of the .csv files during Procedure 4 and to track when 10 experiments are conducted
fftRecordingFlag** - a flag value that is used to indicate when fft recording is initiated (0 - recording is not initiated or has been stopped, 1 - recording is initiated, .csv file is open)
- **exp_start** - a flag that indicates the start of an experimental procedure (that is not embedded in the force_control.cpp - “Classification procedure”) and its current state (when the “Classify” button is pressed - 1, 2 - reaching for the grip of 2N, 3 - grip has been achieved and force is going to 5N, 4 - the previous goal is achieved and going to 4N, 5 - 4N was achieved going to 6N, 0 - experimental procedure has ended or has not been started)
- **expProcedureState** - a flag that indicates the progress of an experimental procedure (both Procedure 3 or Procedure 4), where 0 - no experiment is in progress, 1 - experimental procedure has started, 2 - the experimental procedure is in progress
- **exp3CounterCSV** - a counter used to update the names of the .csv files during Procedure 3 and to track when N_of_exp experiments are conducted
- **expType** - a variable used to indicate the type of an experimental procedure performed (1 - Procedure 3, 2 - Procedure 4)
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) **forceControlCallbackFlag** - a flag that indicates the state received from the exp_status topic (0 - the experiment has not been started, 1 - the goal has been achieved and the experiment is finished, 2 - the experiment is in progress, on its way to the goal)
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) **gripperPosition** - the data on gripper position collected from the “float_grip” topic, used for data recording
- **forceTrackbarPos** - the raw value received from the trackbar callback (0 to 500)
- ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **forceControlPub** - a publisher that communicates with the force_control.cpp and sends desired force values and commands
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) **wittForceY** - force applied along Y-axis received from the Wittenstein F/T sensor through the force_pub.cpp (required for human touch experiment since the force is applied vertically on the object)
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) **wittForceZ** - force applied along Z-axis received from the Wittenstein F/T sensor through the force_pub.cpp (required for data collection and grip control in experimental procedures 3 and 4)
- **bufferCounter** - counter to update fft buffer from “chatter” topic (accelerometer data)
- ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) **accelerometerData** - sample buffer of 2500 samples of data from the accelerometer
- **accDataSize** - the size of the accelerometerData array (default value is 2500)
- **bufferUpdateCounter** - a value that signals when the temporary buffer (5) is sufficient enough to compute new fft and shift the spectrogram
- **bufferUpdateThr** - a threshold variable for bufferUpdateCounter
- **initializerCounter** - a variable that keeps track of when the initialization has been finished (since fft needs to collect a first bag of samples before plotting normal data)
- **initializerThr** - a threshold value for the initializerValue variable (5 since one sample comes from the “chatter” topic)
- **localMaxArray** - an array containing the peak values of the corresponding frequency bandwidth regions
- **spectrIntensity** - the value that controls the max intensity value of the spectrogram
- **spectrSave** - a variable that is used to signal the save of a current spectrogram
- **fftSave** - a variable that is used to signal the save of one current spectrum (one frequency domain sample)
