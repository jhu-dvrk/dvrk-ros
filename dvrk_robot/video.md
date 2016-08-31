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
In this example, the Hauppage frame grabbers are on bus `004` and `001`.

To check if the frame grabbers are working, one can use `tvtime`
(available on most Linux distributions).  The two frame grabbers
should appear in `/dev` with the prefix `video`.  For example:

```sh
$> ls /dev/video*
/dev/video0  /dev/video1
```

The numbering (i.e. which frame grabber is `/dev/video0` and which one
is `/dev/video1`) depends on the order the grabbers are plugged in.
To have a consistent ordering, always plug the frame grabbers in the
same order, e.g. first the left channel and then the right channel.
To test each channel one after another:

```sh
tvtime -d /dev/video0
```
Then:
```sh
tvtime -d /dev/video1
```

Once in `tvtime`, change the input to S-Video.  If you see a black
image, it's possible that you don't have enough light in front of your
camera or endoscope.  If you happen to use a real da Vinci endoscope
and CCUs (Camera Control Units), you can use the toggle switch
`CAM/BAR` to use the video test pattern
(https://en.wikipedia.org/wiki/SMPTE_color_bars).

Using the color bar is also useful to test your video connections,
i.e. if your video is noisy or not visible, put the CCUs in bar mode.
If the video is still not working, the problem likely comes from your
S-video cables.  If the color bars show correctly, the problem comes
from the cables to the endoscope or the endoscope itself.

# Software

## gscam

`gscam` is a ROS node using the `gstreamer` library.  The gstreamer
library supports a few frame grabbers including the Hauppage one.  The
gstreamer developement library can be installed using `apt-get install
libgstreamer0.10-dev`.

The gscam node is part of ROS Hydro but hasn't made it to ROS Indigo
yet (as of March 2016).  If you have ROS Hydro, use `apt-get` to
install it.  If you're using a more recent version of ROS, get the
sources from github and build it.  Assuming your Catkin workspace is
in `~/catkin` and you're using the Catkin Python build tools:

```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/gscam
catkin build
```

To start the `gscam` node, we provide a couple of ROS launch scripts.  For a stereo system, use:

```sh
roslaunch dvrk_robot gscam_stereo.launch rig_name:=jhu_daVinci
```

Where `jhu_daVinci` is a name you want to give to your camera rig.  This name will be used to define the ROS namespace for all the data published.  It is also used to define a directory to save the results of your camera calibration or load said camera calibration (i.e. `dvrk_robot/data/<rig_name>`).  If you don't have a calibration for your rig, you can still render both video channels using the ROS topics:
  * `/jhu_daVinci/left/image_raw`
  * `/jhu_daVinci/right/image_raw`

## (rqt_)image_view

One can use the `image_view` node to visualize a single image:

```sh
rosrun image_view image_view image:=/jhu_daVinci/right/image_raw
```

If you prefer GUI, you can use `rqt_image_view`, a simple program to
view the different camera topics.  Pick the image to display using the
drop-down menu on the top left corner.

## RViz

Use RViz to display both channels at the same time.  Add image, select
topic and then drop image to separate screen/eye on the HRSV display.
You can save your settings to everytime you start RViz you will have
both images.

## Camera calibration

```sh
roslaunch dvrk_robot gscam_stereo.launch rig_name:=jhu_daVinci rect:=false
```

To start the camera calibration:
```
# ros camera calibration
# NOTE: checkerboard 11x10 square with = 5 mm
rosrun camera_calibration cameracalibrator.py --size 11x10 --square 0.005 right:=/jhu_daVinci/right/image_raw left:=/jhu_daVinci/left/image_raw right_camera:=/jhu_daVinci/right left_camera:=/jhu_daVinci/left --approximate=0.050
```

Save results:
 * manual save: calibration result is located /tmp/calibrationdata.tar.gz
 * commit button to save (note camera_info_url should be correct)
 * calibration result is saved as `ost.txt`, which is Videre INI format
 * `gscam` requires calibration file with file extension `.ini` or `.yaml`.
 * `gscam` only takes package://dvrk_robot/data/<rig_name>
 * from now on, use topics `image_rect_color`

References:
 * http://wiki.ros.org/camera_calibration
 * http://wiki.ros.org/camera_calibration_parsers
