### File description

**common.urdf.xacro**  
general material info


#### MTM 
**mtm.urdf.xacro**  
definition of xacro macros for da Vinci MTM

**mtm_right.urdf.xacro**  
call *mtm.urdf.xacro* to generate mtmr only urdf file

**mtm_left.urdf.xacro**  
call *mtm.urdf.xacro* to generate mtml only urdf file

**TODO: mtm_console.urdf.xacro**
TODO: xacro file with 1 MTMR + MTML and ideally a console 

#### PSM
**psm_stable.urdf**
original psm urdf file from WPI

**psm_one.urdf.xacro**
call *psm_exp.urdf.xacro* to generate PSM1 urdf file

**psm_two.urdf.xacro**
call *psm_exp.urdf.xacro* to generate PSM2 urdf file


### How to generate urdf form xacro
```
rosrun xacro xacro.py mtm_right_only.urdf.xacro > mtm_right_only.urdf
```
