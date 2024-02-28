#Pros With VOSS

### Introduction
VOSS is a [PROS](https://pros.cs.purdue.edu/) library that makes writing autonomous code for VEX robots a piece of cake.

## Installing VOSS
1. Download the most recent [template](https://github.com/purduesigbots/VOSS)

2. Run this command from terminal `pros c fetch VOSS@0.1.0.zip`

3.  `cd` into your pros project directory in your terminal

4. Apply the library to the project `pros c apply VOSS`

5. Put `#include "VOSS/api.h"` in your main.h

##Quick start guide
###Robot definitions
* 'LEFT_MOTORS' = list of motors on the left side of the drive
* 'RIGHT_MOTORS' = list of motors on the right side of the drive

###Creating a localizer
* We will set up a IME localizer
1. 'Call' auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
2. Setup inputs to localizer
    * 'Left encoders' = .with_left_motors(LEFT_MOTORS)
    * 'Right encoders' = .with_right_motors(RIGHT_Motors)
    * 'IMU' = .with_imu(IMU_PORT)
    * 'Left right TPi' is the ratio of rotations of the motor encoder to 1 inch of linear movement. It is called with .with_left_right_tip(TIP value)
3. 'Call it to build' --> .build()

##Creating a PID Controller
* 'Linear Proportional constant' = 