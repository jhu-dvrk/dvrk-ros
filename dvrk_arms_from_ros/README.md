# Introduction

The goal of this plugin is to create a cisst/SAW components that has a standard cisstMultiTask arm interface (for the dVRK C++ stack)  

# Usage

To run this, you need 2 PCs, one controlling your MTM(s) and one controlling your PSM(s).   The two PCs have to be configured to share a single rosmaster/roscore (google can help with that).

Then on the PSM PC, start dvrk_robot dvrk_console_json -j console-PSMx.json (nothing new on this side).

On the MTM+Teleop PC, start dvrk_robot dvrk_console_json -j with a console that look like: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/devel/share/jhu-dVRK/console-MTML-PSM1_ROS-Teleop.json
