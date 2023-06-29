
# Xsens MTi ROS Driver and Ntrip Client

This code was based on the official ``xsens_ros_mti_driver`` and tested on MTi-680.
#### Note: the Pvt Data needs to be enabled, so we could get GPGGA data: MT Manager - Device Settings - Output Configuration - GNSS Data , select "Pvt Data", click "Apply"

## Changes made to the MTi ROS Driver:

 - add ``ntrip_util.h`` and ``ntrip_util.cpp`` under ``lib/xspublic/xscommon``
 - add ``nmeapublisher.h`` under ``src/messagepublisher`` folder, to send GPGGA message, ``/nmea`` rostopic.

change:
 - ``lib/xspublic/xscontroller/iointerface.h``, line 138, change to ``PO_OneStopBIt`` for PO_XsensDefaults.
 - ``lib/xspublic/xscommon/threading.cpp``, line 387 to 408, change the threading behavior, this will be useful for ubuntu 22 OS.

## Ntrip_Client
The Ntrip_client subscribes to the ``/nmea`` rostopic from ``xsens_ros_mti_driver``, and wait until it gets data, it will send GPGGA to the Ntrip Caster(Server) every 10 seconds.

User needs to change the ``ntrip.launch`` for their own credentials/servers/mountpoint. 

## How to Install:
install dependency:
```
sudo apt install ros-[ROSDISTRIBUTION]-nmea-msgs
sudo apt install ros-[ROSDISTRIBUTION]-mavros-msgs
```
for example for ROS Melodic:
```
sudo apt install ros-melodic-nmea-msgs
sudo apt install ros-melodic-mavros-msgs
```

clone the source file to your ``catkin_ws``, and run the code below:
```
cd ~/catkin_ws
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
catkin_make
```
Source the ``/devel/setup.bash`` file inside your catkin workspace
```
source ./devel/setup.bash
```
or 

add it into rules:
```
sudo nano ~/.bashrc
```
At the end of the file, add the following line:
```
source /[PATH_TO_Your_catkin_ws]/devel/setup.bash
```
save the file, exit.

## How to Use:
change the credentials/servers/mountpoint in ``src/ntrip/launch/ntrip.launch`` to your own one.


open two terminals:
```
roslaunch xsens_mti_driver xsens_mti_node.launch
```
and then
```
roslaunch ntrip ntrip.launch
```

## How to confirm your RTK Status

you could check ``rostopic echo /rtcm``, there should be HEX RTCM data coming,

or ``rostopic echo /status`` to check the RTK Fix type, it should be Floating or Fix.

