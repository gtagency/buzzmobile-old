#!/bin/bash

rosservice call /setup "{'startLat':34.24, 'startLon':-88.39, 'endLat':34.24,'endLon':-88.41, 'velocity':[1,-1], 'updateRateMS':500}"
rosservice call /playControl
