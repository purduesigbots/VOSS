# PROS With VOSS

### Introduction
VOSS is a [PROS](https://pros.cs.purdue.edu/) library that makes writing autonomous code for VEX robots a piece of cake.

## Installing VOSS
1. Download the most recent [template](https://github.com/purduesigbots/VOSS)

2. Run this command from terminal `pros c fetch VOSS@0.1.0.zip`

3.  `cd` into your pros project directory in your terminal

4. Apply the library to the project `pros c apply VOSS`

5. Put `#include "VOSS/api.h"` in your main.h

## Quick start guide
### Robot definitions
* **LEFT_MOTORS** = list of motors on the left side of the drive
* **RIGHT_MOTORS** = list of motors on the right side of the drive
* **IMU_PORT** = the smart port number in which your imu is plugged into

### Creating a localizer
* We will set up a IME localizer in global scope
1. Call `auto odom = voss::localizer::IMELocalizerBuilder::new_builder()`
2. Setup inputs to localizer
    * **Left encoders** = `.with_left_motors(LEFT_MOTORS)`
    * **Right encoders** = `.with_right_motors(RIGHT_Motors)`
    * **IMU** = `.with_imu(IMU_PORT)`
    * **Left right TPi** is the ratio of rotations of the motor encoder to 1 inch of linear movement. It is called with `.with_left_right_tip`(TIP value)
3. Call it to build --> `.build()`
```cpp
auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
                    .with_left_motors({-1, -2, -3})
                    .with_right_motors({4, 5, 6})
                    .with_left_right_tpi(20)
                    .with_track_width(5)
                    .with_imu(IMU_PORT)
                    .build();

void initialize() {
    
}
```

### Creating a PID controller
* We will set up a PID controller for chassis movements in global scope
* **Linear error** = Linear distance from desired position to current position
* **Angular error** = Angular distance from desired position to current position
1. Required constants and their meaning
    * **Linear proportional constant** = Weight of how much linear error affects motor power (Speeds up the robot movements)
    * **Linear derivative constant** = Weight of how much the change in linear error affects the motor power (increases the rate of acceleration and deceleration)
    * **Linear integral constant** = Weight of how much overall accumulated linear error affects the motor power (increase to improve slight long term error)
    * **Linear exit error** = Allowed linear distance from point in inches for the robot to exit the movement 
    * **Angular proportional constant** = Weight of how much Angular error affects motor power (Speeds up the robot movements)
    * **Angular derivative constant** = Weight of how much the change in Angular error affects the motor power (increases the rate of acceleration and deceleration)
    * **Angular integral constant** = Weight of how much overall accumulated Angular error affects the motor power (increase to improve slight long term error)
    * **Angular exit error** = Allowed Angular distance from point in degrees for the robot to exit the movement
    * **Minimum error** = linear distance allowed from desired position in inches where the robot will check if it has stopped moving
    * **Settle time** = defined time in miliseconds which the robot has not move in order to exit the movement when the desired point has not been reached
2. Call `auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)`
3. Set up inputs to pid controller
    * **Linear proportional, derivative, and integral constant (in this order)** = `.with_linear_constants(0.0, 0.0, 0.0)`
    * **Angular proportional, derivative, and integral constant (in this order)** = `.with_angular_constants(0.0, 0.0, 0.0)`
    * **Linear exit error** = `.with_exit_error(1.0)`
    * **Angular exit error** = `.with_angular_exit_error(1.0)`
    * **Minimum exit error** = `.with_min_error(5)`
    * **Settle time** = `.with_settle_time(200)`
4. Call it to build --> `.build()`
```cpp
auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
                   .with_linear_constants(0.1, 0.1, 0.1)
                   .with_angular_constants(0.1, 0.1, 0.1)
                   .with_exit_error(1.0)
                   .with_angular_exit_error(1.0)
                   .with_min_error(5)
                   .with_settle_time(200)
                   .build();
                
void initialize() {
    
}
```

### Creating the chassis object
* We will be creating a differential drive chassis in global scope
1. Call `voss::chassis::DiffChassis chassis(LEFT_MOTORS, RIGHT_MOTORS, pid, slew rate, brake mode)`
```cpp
voss::chassis::DiffChassis chassis(LEFT_MOTORS, RIGHT_MOTORS, pid, 8, pros::E_MOTOR_BRAKE_BRAKE);

void initialize() {
    
}
```

### Starting the odometry localization
* We will be starting odomentry localization in the initalize scope 
1. Call `odom->begin_localization()`
```cpp
void initialize() {
    odom->begin_localization();    
}
```

### Tuning localizer and controller
* We will be tuning the TPI (ticks per inch) of the localizer
    1. Move the robot forard a measured ammount
    2. Read the odometry value
    3. Divide the amount you moved the robot by the measured movement value from the odometry
        * adjustment factor = robot actual move amount/odometry measured amount
    4. Set the new tpi value to the current tpi value multiplied by the value you got from step 3
        * new tip = old tpi x adjustment factor 
* We will be tuning the PID controller constants
    * This is a lot of guessing and making corrections based off of the behavior of the robot. This will change with any signifcant robot changes
    * Tune linear constants first
        1. Start with the constants all being 0
        2. Increase the proportional constant until oscillations start (the amount you need to increase by and total amount will vary with each robot)
        3. Slowly increase the derivative constant until the robot is no longer overshooting its target and oscilations have stopped
        4. If the robot is lowly compounding error over time, slowly increase the integral constant to reduce that error
    * Tune the angular constants using steps 1-4 of tuning the linear constants
* For more information on PID and Odometry check out the SIGBots Wiki at https://wiki.purduesigbots.com/
* Another great intro to PID article can be found at http://georgegillard.com/documents/2-introduction-to-pid-controllers

### Driver Control
* We will be setting up tank control scheme for the drive in the opcontrol scope
1. Define the controller
    * Call `pros::Controller master(pros::E_CONTROLLER_MASTER)`
2. Inside the while loop set the movement
    * Call `chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y))`
```cpp
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while(true){
        chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
    }
}
```

### Autonomus Movement
* There are two types of basic movment calls which you can use to write an automous
1. Move
    * Parameters
        1. **Desired pose** = {x, y}
        2. **Speed** = 0 - 100
        3. **Flags** = options of movements
            * **THRU** = No PID
            * **Async** = Next lines of code start executing even before movement is finished
            * **REVERSE** = Robot moves backwards
            * **Relative** = not absolute coordinate system
            * **NONE** = defualt
    * Call `chassis.move(Parameters)`
```cpp
void autonomous(){
    chassis.move(voss::Point{1.0, 1.0});
    chassis.move(voss::Point{1.0, 1.0}, 100);
    chassis.move(voss::Point{1.0, 1.0}, 100, voss::Flags::RELATIVE);
    chassis.move(voss::Point{1.0, 1.0}, 100, voss::Flags::REVERSE | voss:Flags::ASYNC);
}
```

2. Turn
    * Parameters
    1. **Desired angle**
    2. **Desired speed**
    3. **Flags** = options of movements
        * **THUR** = Enable motion chaining
        * **ASYNC** = Next lines of code start executing even before movement is finished
        * **RELATIVE** = not absolute coordinate system
        * **REVERSE** = Go backward
        * **NONE** = default
    4. **Angular Direction** = direction of turn
        * **AUTO** = default
        * **COUNTERCLOCKWISE** or **CCW**
        * **CLOCKWISE** or **CW**
    * Call `chassis.turn(parameters)`
```cpp
void autonomous(){
    chassis.turn(90);
    chassis.turn(90, 100);
    chassis.turn(90, 100, voss::Flags::RELATIVE);
    chassis.turn(90, 100, voss::Flags::THRU | voss::Flags::ASYNC);
    chassis.turn(90, 100, voss::Flags::THRU | voss::Flags::ASYNC, voss::AngularDirection::CW);
}
