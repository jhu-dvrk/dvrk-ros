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
**psm_exp.urdf.xacro**
original psm urdf file from WPI for backup 

**psm.urdf.xacro**
psm urdf file defines psm xacro block. see THIS file for real definition. 

**psm_one.urdf.xacro**
call *psm_one.urdf.xacro* to generate PSM1 urdf file

**psm_two.urdf.xacro**
call *psm_two.urdf.xacro* to generate PSM2 urdf file

**both_psms.urdf.xacro**
call *both_psms.urdf.xacro* to generate urdf file with two PSM arms urdf


### How to generate urdf form xacro
```
rosrun xacro xacro.py mtm_right_only.urdf.xacro > mtm_right_only.urdf
```
