[![Build Status](https://travis-ci.org/tum-phoenix/drive_teensy_main.svg?branch=master)](https://travis-ci.org/tum-phoenix/drive_teensy_main)

# Introduction
TUM Phoenix main electronic firmware repository! Open this directory as project with [PlatformIO](https://platformio.org/). It is structured in multiple [environments](http://docs.platformio.org/en/latest/projectconf/section_env.html) for different devices.

## Libraries
Environment specific libraries can be specified in `platformio.ini`.

## Sources
Sources are stored in `src/<environment>`. Add new environments in the `platformio.ini` file.
To only build / upload one specific environment you can:
* uncomment all other environments
* use the command line ([more info](http://docs.platformio.org/en/latest/userguide/cmd_run.html#cmdoption-platformio-run-e))
* use `env_default` ([more info](http://docs.platformio.org/en/latest/projectconf/section_platformio.html#projectconf-pio-env-default))