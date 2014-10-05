#!/bin/bash
#
#rosrun image_view image_view image:=image_raw image_transport:=compressed&
rosrun image_view image_view image:=image_projected image_transport:=compressed&
rosrun image_view image_view image:=image_driveable image_transport:=compressed&
roslaunch buzzmobile_launch test_lane_classifier.launch
