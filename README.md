[![Build Status](https://travis-ci.org/tum-phoenix/drive_teensy_tester.svg?branch=master)](https://travis-ci.org/tum-phoenix/drive_teensy_tester)

# Introduction
TUM Phoenix main electronic firmware repository! Open this directory as project with [PlatformIO](https://platformio.org/).

## Libraries
Environment specific libraries can be specified in `platformio.ini`.

Some common libraries are defined in `lib`. Currently they are git submodules. Therefore do:
* run `git submodule update --init --remote`.
* make sure `lib/libuavcan` is on branch `teensy-driver` and updated to the latest implementation of libuavcan

The libuavcan library with teensy driver implementation can be found here:
https://github.com/tum-phoenix/drive_teensy_libuavcan/

⚠⚠⚠ Please be aware that the teensy driver implementation is still work in progress! It is not finished and there are still some open issues: https://github.com/tum-phoenix/drive_teensy_libuavcan/issues

## Sources
Sources are stored in `src/<environment>`. Add new environments in the `platformio.ini` file.
To only build / upload one specific environment you can:
* uncomment all other environments
* use the command line like ([more info](http://docs.platformio.org/en/latest/userguide/cmd_run.html#cmdoption-platformio-run-e))
* use `env_default` like ([more info](http://docs.platformio.org/en/latest/projectconf/section_platformio.html#projectconf-pio-env-default))


## Build
In PlatformIO just hit the build button and magic happens....
