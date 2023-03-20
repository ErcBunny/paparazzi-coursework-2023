# MAIN README - Hacked By Group 12 in Q3 2023

**Authors**:
* [Andriuškevičius, Justas](https://github.com/justas145)
* Fuser, Michaël
* [Liu, Yueqian](https://github.com/ErcBunny)
* Long, Youyuan
* [Monarcha, Enzo](https://github.com/enzomonarcha)
* [Ou, Dequan](https://github.com/dfordequan)

Our airframe is `bebop_maze_runner`, feel free to play around. This `README` is WIP and will get updates about usage and implementation.

Our strategy is mainly based on [DIS dense optical flow](https://arxiv.org/abs/1603.03590): the MAV stops and turn if the expansion of flow (EOF) is over a threshold. In addition, we add a image gradient check at the start to find the very close object ahead.

**Pros**:
* can detect obstacles regardless of their colors
* can avoid thin panels even if the MAV is approaching the panel from the side

**Cons**:
* the thresholds maybe hard to tune
* cannot detect obstacles that are really close and not moving (though added img gradient based detection)
* can crash into obstacles in the break and go back phase
* not robust to collisions, will lose control if the obtacle blocks the turning phase

**Changed files**:
* `sw/airborne/modules/maze_runner/`: *main logic code*
  1. waypoint generation
  2. strategy for going the waypoint while avoiding obstacles
  3. receive vision frontend ABI topic and publish debug message
* `sw/airborne/modules/computer_vision/cv_maze_runner.c`: *main vision module*
  1. call opencv functions (written in another cpp file)
  2. handle frame queue and publish ABI topic
  3. take care of multithreading
* `sw/airborne/modules/computer_vision/opencv_maze_runner.cpp`: *image processing*
  1. DIS dense optical flow
  2. gradient calculation
* `conf/modules`: *conf files for the modules*
  1. lib links
  2. param description
  3. dlsettings for tuning params in flight
* `conf/airframes/tudelft/bebop_course_maze_runner.xml`: *an independent airframe file*
  1. parameter presets
  2. specify which modules to run
* `conf/flight_plans/tudelft/course_maze_runner_cyberzoo.xml`: *dedicated flight plan*
  1. set functions for the GCS buttons
  2. add a new interactive waypoint GUIDED_GOAL
* `conf/messages.xml`: *for telem msg logging, follow the crashcourse doc*
* `conf/telemetry/default_rotorcraft.xml`: *register telem msg sending frequency*
* `conf/userconf/tudelft/course_conf.xml`: *this file tells the paparazzi centre and GCS about the airframes and what code to compile*
* ` conf/userconf/tudelft/course_control_panel.xml`: *comment out the annoying speech dispatcher*
* `misc/`
  1. legacy code, python notebooks
  2. useful info for setting up the environment in `misc.ipynb` (opencv, gtk imshow, apple silicon)
  3. convenient scripts for opening vlc and telnet

> See the pull requests for more info

Paparazzi UAS
=============
[![Build Status](https://semaphoreci.com/api/v1/paparazziuav/paparazzi/branches/master/shields_badge.svg)](https://semaphoreci.com/paparazziuav/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)
<a href="https://scan.coverity.com/projects/paparazzi-paparazzi">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/4928/badge.svg"/>
</a>

Paparazzi is a free open source software package for Unmanned (Air) Vehicle Systems.
For many years, the system has been used successfuly by hobbyists, universities and companies all over the world, on vehicles of various sizes (11.9g to 25kg).
Paparazzi supports fixed wing, rotorcraft, hybrids, flapping vehicles and it is even possible to use it for boats and surface vehicles.

Documentation is available here https://paparazzi-uav.readthedocs.io/en/latest/

More docs is also available on the wiki http://wiki.paparazziuav.org

To get in touch, subscribe to the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi), the IRC channel (freenode, #paparazzi) and Gitter (https://gitter.im/paparazzi/discuss).

Required software
-----------------

Instructions for installation can be found on the wiki (http://wiki.paparazziuav.org/wiki/Installation).

Quick start:

```
git clone https://github.com/paparazzi/paparazzi.git
cd ./paparazzi
./install.sh
```



For Ubuntu users, required packages are available in the [paparazzi-uav PPA] (https://launchpad.net/~paparazzi-uav/+archive/ppa),
Debian users can use the [OpenSUSE Build Service repository] (http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/)

Debian/Ubuntu packages:
- **paparazzi-dev** is the meta-package on which the Paparazzi software depends to compile and run the ground segment and simulator.
- **paparazzi-jsbsim** is needed for using JSBSim as flight dynamics model for the simulator.

Recommended cross compiling toolchain: https://launchpad.net/gcc-arm-embedded


Directories quick and dirty description:
----------------------------------------

_conf_: the configuration directory (airframe, radio, ... descriptions).

_data_: where to put read-only data (e.g. maps, terrain elevation files, icons)

_doc_: documentation (diagrams, manual source files, ...)

_sw_: software (onboard, ground station, simulation, ...)

_var_: products of compilation, cache for the map tiles, ...


Compilation and demo simulation
-------------------------------

1. type "make" in the top directory to compile all the libraries and tools.

2. "./paparazzi" to run the Paparazzi Center

3. Select the "Bixler" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" in Operation tab and click "Start Session".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same as in simulation !
