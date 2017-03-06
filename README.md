# MAIN README

Paparazzi UAS
=============

[![Build Status](https://travis-ci.org/paparazzi/paparazzi.png?branch=master)](https://travis-ci.org/paparazzi/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)

Paparazzi is an attempt to develop a free software Unmanned (Air) Vehicle System.
 As of today the system is being used successfuly by a number of hobbyists, universities and companies all over the world, on vehicle of various size ( 100g to 25Kg ) and of various nature ( fixed wing, rotorcrafts, boats and surface vehicles).

Up to date information is available in the wiki http://wiki.paparazziuav.org

and from the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi)
and the IRC channel (freenode, #paparazzi).


Required Software
-----------------

Installation is described in the wiki (http://wiki.paparazziuav.org/wiki/Installation).

For Ubuntu users, required packages are available in the [paparazzi-uav PPA] (https://launchpad.net/~paparazzi-uav/+archive/ppa),
Debian users can use the [OpenSUSE Build Service repository] (http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/)

Debian/Ubuntu packages:
- **paparazzi-dev** is the meta-package that depends on everything needed to compile and run the ground segment and the simulator.
- **paparazzi-jsbsim** is needed for using JSBSim as flight dynamic model for the simulator.

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

3. Select the "Microjet" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" from
  the upper-right session combo box and click "Execute".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading of the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same than in simulation !


Setup
-----

Volg deze regels code een voor een op:


```
sudo add-apt-repository -y ppa:paparazzi-uav/ppa && sudo add-apt-repository -y ppa:team-gcc-arm-embedded/ppa && sudo apt-get update && \ sudo apt-get -f -y install paparazzi-dev paparazzi-jsbsim gcc-arm-embedded && cd ~ && git clone --origin upstream https://github.com/paparazzi/paparazzi.git && \ cd ~/paparazzi && git remote update -p && \ git checkout -b v5.10 upstream/v5.10 && sudo cp conf/system/udev/rules/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && \ make && ./paparazzi

git remote add mavlabCourse https://github.com/RickHutten/paparazzi

git fetch mavlabCourse mavlabCourse

git checkout mavlabCourse

git submodule sync

git submodule init

git submodule update

make

python start.py

cd ./sw/tools/parrot/bebop

gdebi ./parrot-tools-linuxgnutools-2012.03 1.0.0-1 i386.deb

cd ../../../ext/libopengles/lib/arm

make

cd ./paparazzi/sw/ext/opencv_bebop

git config --global user.email lalala@example.com

git config --global user.name githubaccountname

make
```
