[![Build Status](https://travis-ci.org/tum-phoenix/drive_teensy_tester.svg?branch=master)](https://travis-ci.org/tum-phoenix/drive_teensy_tester)

# Introduction

Test application for UAVCAN implementation for teensy consisting of 
* `main.cpp`: main file containing setup() and loop()
* `teensy_uavcan.cpp`: some useful functions for the teensy in combination with uavcan
* `parameter.hpp`: examples parameter server
* `publisher.hpp`: example publisher
* `subscriber.hpp`: example subscriber

The libuavcan library with teensy driver implementation can be found here: 
https://github.com/tum-phoenix/drive_teensy_libuavcan/

⚠⚠⚠ Please be aware that the teensy driver implementation is still work in progress! It is not finished and there are still some open issues: https://github.com/tum-phoenix/drive_teensy_libuavcan/issues

## Install

Clone and run `git submodule update --init --remote`. Make sure `lib/libuavcan`
is on branch `teensy-driver` and updated to have the latest implementation of libuavcan.

## Build

In PlatformIO just hit the build button and magic happens....
