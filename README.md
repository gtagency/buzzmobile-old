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
  - nodes
    - camera
    - gps
    - lidar
    - encoder(s)
  - topics
    - ../image_raw
    - scan
- Processors
  - take in sensor data, process it and provide information about the external world
  - these can essentially be used to build a world model
  - nodes
    - classifier
    - short range lane detector
    - projection
    - lane extractor
    - obstacle extractor
  - topics
    - road_class_train
    - road_class
    - ../image_projected
    - lanes
    - obstacles
    - blocked
- planners
  - take in world model information and emit real world data
  - for example "turn left 5 degrees"
  - nodes
    - stay right
    - driver
  - topics
    - planned_path
    - planned_pose(s)
- actuator
  - takes in planner results ("go forward 10 feet", "turn left 5 degrees") and emit machine instructions for motor controllers, etc.


##Topics
- ../image_raw
  - namespaced raw camera data
- road_class_train
  - the data used to train the road classifier
- road_class
  - ternary road classification output
- ../image_projected
  - namespaced projections of images
- scan
  - lidar laser scan depth information
- lanes
  - lanes, in terms of high level road lanes as detected
- obstacles
  - things that we consider unusual, obstacles
- planned_path
