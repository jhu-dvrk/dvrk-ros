# Introduction

The goal of this plugin is to create a cisst/SAW component that has a "standard" cisstMultiTask arm interface (for the dVRK C++ stack) relying on ROS topics to communicate with the actual arm.  This can be used to replace a dVRK arm by any other robotic arm as long as the replacement arm has a ROS interface similar to the dVRK one.  This can also be used for applications that can't use a single process such as remote teleoperation over ethernet.  In this case, the remote arm can actually be a dVRK PSM.

# Usage

To run this, you need 2 PCs, one controlling your MTM(s) and one controlling your PSM(s).   The two PCs have to be configured to share a single rosmaster/roscore (google can help with that).

Then on the PSM PC, start `dvrk_robot dvrk_console_json -j console-PSMx.json`.  Nothing new on this side.

On the MTM+Teleop PC, start `dvrk_robot dvrk_console_json -j` with a console configuration file that looks like: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/devel/share/jhu-dVRK/console-MTML-PSM1_ROS-Teleop.json
