#!/bin/bash

#g++ -O3 FPS.cpp detectVideoLatency.cpp `pkg-config --libs --cflags opencv`  -std=gnu++11 -lboost_program_options  -pthread -o detectVideoLatency

FPS_HEADERS=../../FPS/src/
FPS_LIBS=../../FPS/build
mkdir  -p build
cd build
g++ -O3 ../src/detectVideoLatency.cpp -I$FPS_HEADERS -L$FPS_LIBS  `pkg-config --libs --cflags opencv`  -std=gnu++11 -lboost_program_options -lFPS -pthread -o detectVideoLatency

LD_LIBRARY_PATH=../../FPS/build/ ./detectVideoLatency  --help
