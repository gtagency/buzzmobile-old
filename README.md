Buzzmobile - The self driving parade float
================


## This project contains:
* buzzmobile_launch - Launch files for starting the car systems.
* navigator - Navigator node used to navigate the car across a graph of waypoints.
* steering/arduino - The arduino sketch for the steering system, built on an Arduino Uno and Arduino motor controller.
* steering/ros - ROS nodes to control the steering arduino sketch using "joy"
* test - Scripts and other test code, used for validating car subsystems.


####Updates and new stuff below here

##Subfolders
- Sensors
  - Take in data from phyical things and spit out minimally processed data (generally)
  - could also be "lines" in that the sesor returns a set of lines
- Processors
  - take in sensor data, process it and provide information about the external world
  - these can essentially be used to build a world model
- planners
  - take in world model information and emit real world data
  - for example "turn left 5 degrees"
- actuator
  - takes in planner results ("go forward 10 feet", "turn left 5 degrees") and emit machine instructions for motor controllers, etc.


