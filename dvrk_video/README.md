Video pipeline
==============

This describes a fairly low cost setup that can be used with the dVRK
HRSV display (High Resolution Stereo Video).  We use a couple of cheap
USB frame grabbers (Hauppage Live 2) for the analog videos from SD
cameras (640x480).  For HD systems (720p and 1080p), we tested a
BlackMagic DeckLink Duo frame grabber with dual SDI inputs (see also
the dVRK video pipeline page for the hardware setup:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Video-Pipeline).
For displaying the video back, we just use a graphic card with two
spare video outputs.  The software relies heavily on ROS tools to grab
and display the stereo video.  Some lag is to be expected.

The general steps are:
 * Make sure the frame grabber works (e.g. using tvtime or vendor application)
 * Figure out the gstreamer pipeline and test using `gst-launch-1.0`
 * Create a lauch file for gscam with the gstreamer pipeline you just tested
 
# Disclaimer

This page is a collection of notes that might be helpful for the dVRK
community but it is in no way exhaustive.  If you need some help
re. gstreamer and gscam, you should probably start searching online
and/or reach out to the gstreamer and gscam developers.

# Hardware

## USB frame grabbers

The frame grabbers we use most often for SD endoscopes are Hauppage USB Live 2:
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
Alternatively, you can setup `udev` rules to automatically assign a
device name for a specific frame grabber identified by serial number
(see below).

Some Linux distributions might restrict access to the video devices using the `video` group.  To check, do:
```sh
ls -l /dev/video*
```
If the result shows something like:
```sh
crw-rw----+ 1 root video 81, 0 Nov 14 11:47 /dev/video0
```

you will need to add your user id to the `video` group.  Do not use
`sudo tvtime`, it might work for `tvtime` but it's not going to work
with `gscam`.  You should fix the unix file permissions first and make
sure you can access the video without `sudo`.

To test each channel one after another:
```sh
tvtime -Ld /dev/video0
```
Then:
```sh
tvtime -Ld /dev/video1
```

Once in `tvtime`, change the input to S-Video by pressing `i` key.  If you see a black
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

Once you have the video showing in tvtime, you need to figure out the
gstreamer options.  There is some information online and you can use
`gst-inspect-1.0` (see more details in DeckLink Duo section).  You can
also use the command line tool `v4l2-ctl` to figure out the output
format of your frame grabber.  The option `-d0` is to specify
`/dev/video0`, if you're using a different device, make sure the digit
matches.  Example of commands:

```sh
v4l2-ctl -d0 --get-fmt-video
v4l2-ctl -d0 --list-formats-ext
```

On Ubuntu 18.04, we found the following syntax seems to work with the USB Hauppage Live2:
```sh
 gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,interlace-mode=interleaved ! autovideosink
 ```

To setup a `udev` rule, you first need to find a way to uniquely
identify each frame grabber.  To start, plug just one frame grabber
then do `ls /dev/video*`.  Use the full path to identify each frame
grabber (e.g. `/dev/video0`, `/dev/video1`...):

```
udevadm info --attribute-walk /dev/video0
```
Scroll through the output to find the serial number:
```
    ATTR{manufacturer}=="Hauppauge"
    ...
    ATTR{serial}=="0011485772"
```

Note that this info should correspond to the messages in `dmesg -w`
when you plugged your frame grabber.  Now we can create a `udev` rule
to automatically assign the frame grabber to a specific `/dev/video`
"device".  You can write the rules in
`/etc/udev/rules.d/90-hauppauge.rules` using sudo privileges, replace
the serial numbers with yours, the following example is for a stereo
system:

```
SUBSYSTEM=="video4linux", ATTRS{manufacturer}=="Hauppauge", ATTRS{serial}=="0011367747", SYMLINK+="video-left"
SUBSYSTEM=="video4linux", ATTRS{manufacturer}=="Hauppauge", ATTRS{serial}=="0011485772", SYMLINK+="video-right"
```

Save the file and then do `sudo udevadm control --reload-rules` to
apply the rules.  No need to reboot the computer, just unplug your
frame grabber, wait a few seconds, replug it and then do `ls -l
/dev/video*` to confirm that the rule worked.  If this didn't work,
these pages have some useful info for debugging `udev` and
`video4linux` rules:
https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux
and
https://unix.stackexchange.com/questions/424887/udev-rule-to-discern-2-identical-webcams-on-linux

## Blackmagic DeckLink Duo frame grabber

You first need to install the drivers from Blackmagic, see
https://www.blackmagicdesign.com/support/family/capture-and-playback
The drivers are included in the package "Desktop Video".  Once you've
downloaded the binaries and extracted the files from Blackmagic,
follow the instructions on their ReadMe.txt.  For 64 bits Ubuntu
system, install the `.deb` files in subfolder `deb/x86_64` using `sudo
dpkg -i *.deb`.  If your card is old, the DeckLink install might ask
to run the BlackMagic firmware updater, i.e. something like
`BlackmagicFirmwareUpdater update 0`.  After you reboot, check with
`dmesg | grep -i black` to see if the card is recognized.  If the
driver is working properly, the devices will show up under
`/dev/blackmagic`.

You can quickly test the frame grabber using `MediaExpress` which
should be installed along the drivers.  You can also select the video
input using `BlackmagicDesktopVideoSetup` (also installed along
drivers).

If you need to remove all the Blackmagic packages to test a different
version, use `sudo apt remove desktopvideo* mediaexpress*`.

To test if the drivers are working and the cards are working, use
gstreamer 1.0 or greater.  On Ubuntu 16.04/18.04 both gstreamer 1.0
and 0.1 are available.  Make sure you ONLY install the 1.0 packages.
You will also need the proper gstreamer plugins installed:

```sh
  sudo apt install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad 
```

Once gstreamer is installed, you can use a few command lines to test the drivers:
  * `gst-inspect-1.0 decklinkvideosrc` will show you the different parameters for the Decklink gstreamer plugin
  * `gst-launch-1.0` can be used to launch the streamer and pipe it to see the video live on the computer.   For example, we used `gst-launch-1.0 -v decklinkvideosrc mode=0 connection=sdi device-number=0 ! videoconvert ! autovideosink`.
    * `mode=0` is for auto detection and is optional
    * `connection=sdi` is to force to use an SDI input if your card has different types of inputs.  This is optional.
    * `device-number=0` is to select which input to use if you have multiple inputs
  * On a Decklink Duo, we found that one can see the stereo video using two text terminals:
    * `gst-launch-1.0 decklinkvideosrc device-number=0 ! videoconvert ! autovideosink`
    * `gst-launch-1.0 decklinkvideosrc device-number=1 ! videoconvert ! autovideosink`

**Note:**  For some reason, in Ubuntu 20.04 you need to add all users to the `plugdev` group.  It's a bit odd since `/dev/blackmagic/*` has `rw` permissions for all users.  If you figure out a fix, let us know.

# Software

## gscam

`gscam` is a ROS node using the `gstreamer` library.  The gstreamer
library supports a few frame grabbers including the Hauppage one.  The
gstreamer developement library can be installed using `apt-get
install`.  Make sure you install gstreamer 1.0, not 0.1.  It is
important to note that when you're installing `gscam`, the
dependencies will also be installed and you might install the wrong
version of `gstreamer` without realizing it.

To figure out if the ROS provided gscam uses gstreamer 0.1 or 1.x, use the command line:
```sh
apt-cache showpkg ros-kinetic-gscam # or ros-lunar-gscam, melodic or whatever ROS version
```

Look at the output of the `apt-cache showpkg` command and search the "Dependencies" to find the gstreamer version used.

As far as we know, ROS Kinetic on Ubuntu 16.04 uses the gstreamer 0.1
so you will have to manually compile `gscam` to use gstreamer 1.x.
Melodic on 18.04 seems to use gstreamer 1.x so you should be able
install using `apt`.

### ROS Ubuntu packages vs build from source

Use `apt install` to install gscam on Ubuntu 18.04.  The package name should be `ros-melodic-gscam`.   It will install all the required dependencies for you.

On Ubuntu 20.04, gscam binaries are not available via `apt` so you
will need to compile it in your ROS workspace.  The original source
code is on github: https://github.com/ros-drivers/gscam.  But you need
a different version which can be found using the pull request for
Noetic Devel.  So you need to clone https://github.com/hap1961/gscam
in your `catkin_ws/src`.  then make sure you switch to the Noetic
branch: `cd ~/catkin_ws/src/gscam; git checkout noetic-devel`.  Then
`catkin build`.  This info is from June 2022, it might need to be
updated.

### Manual compilation

If you need gstreamer 1.x and gscam is not built against it, you need to manually compile it.  You will first need to install some development libraries:
```sh
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

Then, assuming your catkin workspace is in `~/catkin` and you're using the Catkin Python build tools:

```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/gscam
catkin build
source ~/catkin_ws/devel/setup.bash
```

**Note:** See https://github.com/ros-drivers/gscam.  As of March 2018,
  the readme on gscam/github are a bit confusing since they still
  indicate that gstreamer 1.x support is experimental but they provide
  instructions to compile with gstreamer 1.x.  So, make sure you
  compile for 1.x version.


### Using gscam

To start the `gscam` node, we provide a couple of ROS launch scripts.
**Make sure the launch script has been updated to use a working
gstreamer pipeline** (as descrided above using `gst-launch-1.01`).
The main difference is that your pipeline for gscam should end with
`videoconvert` and you need to remove `autovideosink`.

For a stereo system with the USB frame grabbers, use: ```sh roslaunch
dvrk_robot gscam_stereo.launch rig_name:=jhu_daVinci ``` Where
`jhu_daVinci` is a name you want to give to your camera rig.  This
name will be used to define the ROS namespace for all the data
published.  It is also used to define a directory to save the results
of your camera calibration or load said camera calibration
(i.e. `dvrk_robot/data/<rig_name>`).  If you don't have a calibration
for your rig, you can still render both video channels using the ROS
topics:

  * `/jhu_daVinci/left/image_raw`
  * `/jhu_daVinci/right/image_raw`

For a system with a Decklink Duo, the `gscam_config` in a launch script would look like:
```xml
    <param name="gscam_config" value="decklinkvideosrc connection=sdi device-number=0 ! videoconvert"/>
```

**Note:** ROS topic names might changes when upgraded from 18.04/Melodic to 20.04/Noetic

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
