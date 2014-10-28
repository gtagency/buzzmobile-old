#!/bin/bash
#

gcc -shared -Wl,-soname,detector_node_lib -o detector_node_lib.so -fPIC src/detector_lib.c

# then copy to /usr/lib
