# PROS With VOSS

### Introduction
VOSS is a [PROS](https://pros.cs.purdue.edu/) library that makes writing autonomous code for VEX robots a piece of cake.

## Installing VOSS
1. Download the most recent [template](https://github.com/purduesigbots/VOSS/releases/tag/0.1.2)

2. Run this command from terminal `pros c fetch VOSS@0.1.2.zip`

3.  `cd` into your pros project directory in your terminal
4.  run `pros c info-project`, your kernel version MUST be 4.0.7. If it is not, create a new 4.0.7 project by `cd` into the directory you want to make your project, and run `pros c n PROJECT_NAME -ea`. 

5. Apply the library to the project `pros c apply VOSS`

6. Put `#include "VOSS/api.h"` in your main.h

### Creating exit conditions
* We will set up a exit conditions object in global scope
1. Call `auto ec = voss::controller::ExitConditions::new_conditions()`
2. Setup conditions
    * **Velocity base exit** = `.add_settle(int settle_time, double tolerance, int initial_delay)`
    * **Distance base exit** = `.add_tolerance(double linear_tolerance, double angular_tolerance, double tolerance_time)`
    * **Time base exit**(ms) = `.add_timeout(int time)`
    * **Motion chaining early exit**(as smoothness increase, accuracy decrease) = `.add_thru_smoothness(double thru_smoothness)`
3. Call it to build --> `.build()`
```cpp
auto ec = voss::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.0, 2.0, 200)
              .add_timeout(22500)
              .add_thru_smoothness(4)
              .build();
```
### Creating a localizer
* We will set up a localizer in global scope
* You have three choices, IME (Internal motor encoder), ADI Encoders, or Smart Port Rotation sensors.
1. Call `auto odom = voss::localizer::voss::localizer::TrackingWheelLocalizerBuilder::new_builder()`
2. Setup inputs to localizer
    * **Left** :
      - `.with_left_encoder(int adi_port)`
      - `.with_left_encoder(int smart_port, int adi_port)`
      - `.with_left_rotation(int port)`
      - `.with_left_motor(int port)`
    * **Right** :
      - `.with_right_encoder(int adi_port)`
      - `.with_right_encoder(int smart_port, int adi_port)`
      - `.with_right_rotation(int port)`
      - `.with_right_motor(int port)`
    * **IMU** :
      - `.with_imu(int imu_port)`
    * **Track width**
      - `.with_track_width(double track_width_distance)`
    * **Left right TPI** (ratio of encoder rotations to 1 inch of linear movement)
      - `.with_left_right_tip(double tpi_value)`
3. Call it to build --> `.build()`
```cpp
auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
                .with_right_motor(10)
                .with_left_motor(-4)
                .with_track_width(11)
                .with_left_right_tpi(18.43)
                .with_imu(16)
                .build();


void initialize() {
    pros::lcd::initialize();
    odom->begin_localization(); //calibrate and begin localizing
}
```
### Tuning a localizer
* We will be tuning the TPI (ticks per inch) of the localizer
    1. Move the robot forard a measured ammount
    2. Read the odometry value
    3. Divide the amount you moved the robot by the measured movement value from the odometry
        * adjustment factor = robot actual move amount/odometry measured amount
    4. Set the new tpi value to the current tpi value multiplied by the value you got from step 3
        * new tip = old tpi x adjustment factor

### The basics of PID (Proportional Integral Derivative controllers)
* **Linear error** = Linear distance from desired position to current position (inches)
* **Angular error** = Angular distance from desired position to current position (degrees)
* **Linear proportional constant** = Weight of how much linear error affects motor power (speeds up the robot movements)
* **Linear derivative constant** = Weight of how much the change in linear error affects the motor power (increases the rate of acceleration and deceleration)
* **Linear integral constant** = Weight of how much overall accumulated linear error affects the motor power (increase to improve slight long term error)
* **Angular proportional constant** = Weight of how much Angular error affects motor power (speeds up the robot movements)
* **Angular derivative constant** = Weight of how much the change in Angular error affects the motor power (increases the rate of acceleration and deceleration)
* **Angular integral constant** = Weight of how much overall accumulated Angular error affects the motor power (increase to improve slight long term error)
* **Output of the control loop** = The error X proportional constant + the change in error X derivative constant + the sum of error over the entire time X integral constant

      
### Creating a PID controller
* We will set up a PID controller for chassis movements in global scope
1. Call `auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)`
2. Set up inputs to pid controller
    * **Linear proportional, derivative, and integral constant (in this order)** = `.with_linear_constants(20, 0.02, 169)`
    * **Angular proportional, derivative, and integral constant (in this order)** = `.with_angular_constants(250, 0.05, 2435)`
    * **Minimum exit error** = `.with_min_error(5)`
    * **Minimun velocity for thru motion** = `.with_min_vel_for_thru(100)`
3. Call it to build --> `.build()`
```cpp
auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(20, 0.02, 169)
               .with_angular_constants(250, 0.05, 2435)
               .with_min_error(5)
               .with_min_vel_for_thru(100)
               .build();    
```

### Creating a Boomerang controller
* Boomerang controller demo on [Desmos](https://www.desmos.com/calculator/zl0pizecei?lang=zh-TW)
* We will set up a Boomerang controller for chassis movements in global scope
1. Call `auto boomerang = voss::controller::BoomerangControllerBuilder::new_builder(odom)`
2. Set up inputs to boomerang controller
    * **Linear proportional, derivative, and integral constant (in this order)** = `.with_linear_constants(20, 0.02, 169)`
    * **Angular proportional, derivative, and integral constant (in this order)** = `.with_angular_constants(250, 0.05, 2435)`
    * **Leading percentage**(greater than 0, but less than 1) = `.with_lead_pct(0.5)`
    * **Minimum exit error** = `.with_min_error(5)`
    * **Minimun velocity for thru motion** = `.with_min_vel_for_thru(100)`
3. Call it to build --> `.build()`
```cpp
auto boomerang = voss::controller::BoomerangControllerBuilder::new_builder(odom)
                     .with_linear_constants(20, 0.02, 169)
                     .with_angular_constants(250, 0.05, 2435)
                     .with_lead_pct(0.5)
                     .with_min_vel_for_thru(70)
                     .with_min_error(5)
                     .build();
```

### Creating a Swing controller
* We will set up a Swing controller for chassis movements in global scope
1. Call `auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)`
2. Set up inputs to swing controller
    * **Angular proportional, derivative, and integral constant (in this order)** = `.with_angular_constants(250, 0.05, 2435)`
3. Call it to build --> `.build()`
```cpp
auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
                 .with_angular_constants(250, 0.05, 2435)
                 .build();
```

### Creating a Arc controller
* We will set up a Arc controller for chassis movements in global scope
* **Please avoid using this controller, because we are still trying to optimize it.**
1. Call `voss::controller::ArcPIDControllerBuilder(odom)`
2. Set up inputs to pid controller
    * **Track width** = `.with_track_width(16)`
    * **Linear proportional, derivative, and integral constant (in this order)** = `.with_linear_constants(20, 0.02, 169)`
    * **Angular proportional, derivative, and integral constant (in this order)** = `.with_angular_constants(250, 0.05, 2435)`
    * **Leading percentage** (greater than 0, but less than 1) = `.with_lead_pct(0.5)`
    * **Minimum exit error** = `.with_min_error(5)`
    * **Slew rate(limits linear acceleration. Higher slew rate = higher acceleration)** = `.with_slew(8)`
4. Call it to build --> `.build()`
```cpp
auto arc = voss::controller::ArcPIDControllerBuilder(odom)
               .with_track_width(16)
               .with_linear_constants(20, 0.02, 169)
               .with_angular_constants(250, 0.05, 2435)
               .with_min_error(5)
               .with_slew(8)
               .build();
```

## Tuning Controllers
### Tuning PID
* We will be tuning the PID controller constants
    * This is a lot of guessing and making corrections based off of the behavior of the robot. This will change with any signifcant robot changes
    * Tune linear constants first
        1. Start with the constants all being 0
        2. Increase the proportional constant until oscillations start (the amount you need to increase by and total amount will vary with each robot)
        3. Slowly increase the derivative constant until the robot is no longer overshooting its target and oscilations have stopped
        4. If the robot is compounding error over time, slowly increase the integral constant to reduce that error
    * Tune the angular constants using steps 1-4 of tuning the linear constants
* For more information on PID and Odometry check out the SIGBots Wiki at https://wiki.purduesigbots.com/
* Another great intro to PID article can be found at http://georgegillard.com/documents/2-introduction-to-pid-controllers

### Tuning Other Controllers
* Most of our controllers use PID but the logic of how it is applied is what makes the controller unique
    * For linear and angular constants reference **Tuning PID** above
* Minimum exit error
    * The Outer tolerance zone in which the robot slows down to the tolerance point
        * In PID controller, once in this zone the robot stops correcting for heading
        * In Boomerang controller, once it is in this zone the robot starts correcting for heading
    * Increasing this value will increase the tolerance allowing for the robot to exit the movement easier, but will decrease the accuracy of the movement
    * This value will vary for the types of movements your robot is going to make but should be as small as possible without the robot getting stuck in movements and with the robot being able to correct for heading. To tune this start with small value and increase until desired results
* Minimun velocity for thru motion
    * Sets the robot's velocity as it reaches the target poing and transitions between movements
    * This value depends on the desired behavior for the robot
* Leading percentage
    * Must be greater than 0
    * Must be less than 1
    * The larger the leading percentage, the further the robot will stray from a straight path to the point
        * This allows for the robot to correct for large difference in starting and ending heading
        * This requires more space for the robot to move around
    * This value will vary based on the desired behavior of the robot, but to tune this start small and increase until desired results 
* Slew rate
    * Limits linear acceleration
    * Higher slew rate = higher acceleration
    * This value will vary based on the desired behavior of the robot, but to tune this start small and increase until desired results
* For other parameters reference each controller's description

## Setting up and starting robot control
### Creating the chassis object
* We will be creating a differential drive chassis in global scope
* Call `DiffChassis(std::initializer_list<int8_t> left_motors, std::initializer_list<int8_t> right_motors, controller_ptr default_controller, ec_ptr ec, double slew_step, pros::motor_brake_mode_e brakeMode)`
```cpp
auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec, 8, pros::E_MOTOR_BRAKE_COAST);
//we recommend using the pid controller as default controller
```

### Starting the odometry localization
* We will be starting odomentry localization in the initalize scope 
1. Call `odom->begin_localization()`
```cpp
void initialize() {
   odom->begin_localization(); //calibrate and begin localizing
   pros::delay(3000); //don't move the robot for 3 seconds
}
```

### Driver Control
* We will be setting up control scheme for the drive in the opcontrol scope
1. Define the controller
    * Call `pros::Controller master(pros::E_CONTROLLER_MASTER)`
2. Inside the while loop set the movement
    * **Tank control** = `chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y))`
    * **Arcade control** = `chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X))`
```cpp
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while(true){
        chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
        //or
        chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));
    }
}
```
## Autonomous Programming
### Autonomus Movement
* There are two types of basic movment calls which you can use to write an autonomous
1. Move
    * Controllers
      - PID Controller
      - Boomerang Controller
      - Arc Controller **Please avoid using this controller, because we are still trying to optimize it.** 
    * Parameters
        1. **Target** = Relative distance, {x, y}, or {x, y, theta} (Remember for boomerang controller, you need to specify theta)
        2. **Controller** = PID, Boomerang, or Arc
        3. **Speed** = 0 - 100 (100 is default)
        4. **Flags** = options of movements
            * **THRU** = Enable motion chaining
            * **ASYNC** = Next lines of code start executing even before movement is finished
            * **REVERSE** = Robot moves backwards
            * **RELATIVE** = Not absolute coordinate system
            * **NONE** = Defualt
    * Call `chassis.move(Parameters)`
```cpp
void autonomous(){
   // using default controller:
    chassis.move(10); //move forward 10
    chassis.move(-10, 100, voss::Flags::REVERSE) //move backward 10
    chassis.move(10, 70);
    chassis.move(voss::Point{1.0, 1.0}, 70);
    chassis.move(voss::Point{1.0, 1.0, 30}, 100, voss::Flags::RELATIVE);
    chassis.move(voss::Point{1.0, 1.0}, 100, voss::Flags::REVERSE | voss:Flags::ASYNC | voss::Flags::THRU);
   // using boomerang controller:
    chassis.move(voss::Point{1.0, 1.0, 90}, boomerang); 
    chassis.move(voss::Point{1.0, 1.0, 20}, boomerang, 70);
    chassis.move(voss::Point{1.0, 1.0, 30}, boomerang, 100, voss::Flags::RELATIVE);
    chassis.move(voss::Point{1.0, 1.0, 10}, boomerang, 100, voss::Flags::REVERSE | voss:Flags::ASYNC | voss::Flags::THRU);
   // using arc controller:
    chassis.move(voss::Point{1.0, 1.0}, arc); 
    chassis.move(voss::Point{1.0, 1.0}, arc, 70);
    chassis.move(voss::Point{1.0, 1.0}, arc, 100, voss::Flags::RELATIVE);
    chassis.move(voss::Point{1.0, 1.0}, arc, 100, voss::Flags::REVERSE | voss:Flags::ASYNC | voss::Flags::THRU);
}
```

2. Turn
    * Controllers
       - PID controller
       - Swing controller
    * Parameters
    1. **Target** = angle or {x, y}
    2. **Controller** = PID or Swing
    3. **Desired speed** = 0 - 100 (100 is default)
    4. **Flags** = options of movements
       * **THRU** = Enable motion chaining
       * **ASYNC** = Next lines of code start executing even before movement is finished
       * **REVERSE** = Robot moves backwards (for swing controller)
       * **RELATIVE** = Not absolute coordinate system
       * **NONE** = Defualt
    5. **Angular Direction** = direction of turn
        * **AUTO** = default
        * **COUNTERCLOCKWISE** or **CCW**
        * **CLOCKWISE** or **CW**
    * Call `chassis.turn(parameters)` or `chassis.turn_to(parameters)`
```cpp
void autonomous(){
//using default controller:
    chassis.turn(90);
    chassis.turn_to({10, 10});
    chassis.turn(90, 50);
    chassis.turn_to({10, 10}, 40);
    chassis.turn(90, 100, voss::Flags::RELATIVE);
    chassis.turn_to({10, 10}, 40, voss::Flags::RELATIVE);
    chassis.turn(90, 100, voss::Flags::THRU | voss::Flags::ASYNC);
    chassis.turn(90, 100, voss::Flags::NONE, voss::AngularDirection::CW);
    chassis.turn_to({10, 10}, 40, voss::Flags::RELATIVE | voss::Flags::THRU, voss::AngularDirection::CW);

//using swing controller:
    chassis.turn(90, swing);
    chassis.turn_to({10, 10}, swing);
    chassis.turn(90, swing, 50);
    chassis.turn_to({10, 10}, swing, 40);
    chassis.turn(90, 100, swing, voss::Flags::RELATIVE);
    chassis.turn_to({10, 10}, swing, 40, voss::Flags::RELATIVE | voss::Flags::REVERSE);
    chassis.turn(90, 100, swing, voss::Flags::NONE, voss::AngularDirection::CW);
    chassis.turn_to({10, 10}, swing, 40, voss::Flags::RELATIVE | voss::Flags::THRU, voss::AngularDirection::CW);
}
```
## Additional Resources

By following the In Depth Documentation(**Coming Soon!**), your team should be able to create a competitive program for your competition robot. For people who are interested in more advanced programming such as programming skills runs, there is a lot of potential customization with this library. The following resources may interest people who want to take their programming skills further:

- [Take a C++ programming course.](https://www.codecademy.com/learn/learn-c-plus-plus)

- [Explore the PROS API](https://pros.cs.purdue.edu/v5/index.html)

- [Learn PID](http://georgegillard.com/documents/2-introduction-to-pid-controllers)

- [Read the Vex Forums a lot](http://vexforum.com)

- [Get help from other teams on discord](https://discordapp.com/invite/9JDWW8e) 
