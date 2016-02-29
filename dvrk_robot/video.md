Video pipeline
==============

This describes a fairly low cost setup that can be used with the
dVRK/HRSV display (High Resolution Stereo Video).  We use a couple of
cheap USB frame grabbers along with a graphic card with two spare
video outputs.  The software relies heavily on ROS tools to grab and
display the stereo video.  Some lag is to be expected.

# Hardware

The frame grabbers we use most often are Hauppage USB Live 2:
 * Manufacturer: http://www.hauppauge.com/site/products/data_usblive2.html
 * Amazon, about $45: http://www.amazon.com/Hauppauge-610-USB-Live-Digitizer-Capture/dp/B0036VO2BI

When you plug these in your PC, make sure both frame grabbers are on
different USB channels otherwise you won't have enough bandwidth to
capture both left and right videos.  To check, use `lsusb`.  The output should look like:
```sh
$> lsusb 
Bus 004 Device 006: ID 0461:4e22 Primax Electronics, Ltd 
Bus 004 Device 005: ID 413c:2107 Dell Computer Corp. 
Bus 004 Device 004: ID 0424:2514 Standard Microsystems Corp. USB 2.0 Hub
Bus 004 Device 003: ID 2040:c200 Hauppauge 
Bus 004 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 004 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 002: ID 2040:c200 Hauppauge 
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 003 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```
In this example, the frame grabbers on bus `004` and `001`.

To check if the frame grabbers are working, one can use `tvtime`
(available on most Linux distributions).  The two frame grabbers
should appear in `/dev` with the prefix `video`.  For example:

```sh
$> ls /dev/video*
/dev/video0  /dev/video1
```

To test each channel one after another:

```sh
tvtime -d /dev/video0
```

Once in tvtime, change the input to S-Video.  If you see a black
image, it's possible that you don't have enough light in front of your
camera or endoscope.  If you happen to use a real da Vinci endoscope
and CCUs (Camera Control Units), you can use the toggle swith
`CAM/BAR` to use the video pattern
(https://en.wikipedia.org/wiki/SMPTE_color_bars).

# Software

## gscam

gscam is a ROS node using the gstreamer library.  The gstreamer
library supports a few frame grabbers including the Hauppage one, the
gstreamer developement library can be installed using `apt-get install
libgstreamer0.10-dev`.

The gscam node is part of ROS Hydro but hasn't made it to ROS Indigo
yet (as of March 2016).  If you have ROS Hydro, use `apt-get` to
install it.  If you're using a more recent version of ROS, get the
sources from github and build it.  Assuming your Catkin workspace is
in ~/catkin and you're using the catkin build tools:

```sh
cd ~/catkin/src
git clone https://github.com/ros-drivers/gscam
catkin build
```