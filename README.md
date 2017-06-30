[![Build Status](https://travis-ci.org/tum-phoenix/drive_teensy_tester.svg?branch=master)](https://travis-ci.org/tum-phoenix/drive_teensy_tester)
# drive_teensy_tester
Test application for UAVCAN implementation for teensy consisting of a `publisher.cpp`
and a `subscriber.cpp`, one for publishing LogMessages and the other one subscribed to
them dumping stuff on Serial port.

## Install

Clone and run `git submodule update --init --remote`. Make sure `lib/libuavcan`
is on branch `teensy-driver` and updated to have the latest implementation of libuavcan.

## Build

In PlatformIO just hit the build button and magic happens....
