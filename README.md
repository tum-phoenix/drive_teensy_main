[![Build Status](https://travis-ci.org/tum-phoenix/drive_teensy_main.svg?branch=master)](https://travis-ci.org/tum-phoenix/drive_teensy_main)

# Introduction
TUM Phoenix main electronic firmware repository! Open this directory as project with [PlatformIO](https://platformio.org/). It is structured in multiple [environments](http://docs.platformio.org/en/latest/projectconf/section_env.html) for different devices.

## Libraries
Environment specific libraries can be specified in `platformio.ini`.

## UAVCAN GUI Tool
An easy to use debug tool is available: https://uavcan.org/GUI_Tool/Overview/ .
Custom message types must be present in a special folder ([more Info](https://uavcan.org/Implementations/Pyuavcan/Tutorials/2._Basic_usage/#using-vendor-specific-dsdl-definitions)). The easiest way to keep these up to date is to create a symlink. Under Linux systems do:

    mkdir ~/uavcan_vendor_specific_types
    ln -s drive_teensy_main/lib/phoenix_msgs ~/uavcan_vendor_specific_types/

## Sources
Sources are stored in `src/<environment>`. Add new environments in the `platformio.ini` file.
To only build / upload one specific environment you can:
* recommended: use the command line ([more info](http://docs.platformio.org/en/latest/userguide/cmd_run.html#cmdoption-platformio-run-e)) `pio run -t upload -e <env>`
* uncomment all other environments
* use `env_default` ([more info](http://docs.platformio.org/en/latest/projectconf/section_platformio.html#projectconf-pio-env-default))
