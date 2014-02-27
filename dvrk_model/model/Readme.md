### File description

**common.urdf.xacro**  
general material info

**mtm.urdf.xacro**
definition of xacro macros e.g. mtm, mtm_console 

**mtm_right_only.urdf.xacro**
call *mtm.urdf.xacro* to generate mtmr only urdf file

**psm_stable.urdf**
original psm urdf file from WPI

**psm_jhu.urdf**
manual urdf file manual tweaked by Zihan


### How to generate urdf form xacro
```
rosrun xacro xacro.py mtm_right_only.urdf.xacro > mtm_right_only.urdf
```
