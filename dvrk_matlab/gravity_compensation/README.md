# Overview
This _dvrk_Gravity_Compensation_ package is designed for gravity compensation(GC) of MTM of dVRK. It is a improved version of [dvrk_gravity_comp
 of WPI AIM Lab](https://github.com/WPI-AIM/dvrk_gravity_comp/). It inherits symbolic derivation of GC using Lagrange method from their works. The former work provides a method to identify GC parameters using LSE. However, they fail to model external cable dynamic effect. Moreover one-step LSE is limited in this scenario with uneven mass across the serial manipulator. Here we proposed our method of GC of dVRK, including following features:

* Isolated joint coupling data collection
* New fitting model for predicting coupling effect of cable and gravity of MTM.
* Multi-steps LSE
* Coulomb Friction compensation

# Collect data, estimate dynamic parameters and run gravity compensation controller
Run dVRK console
```
#use a console config that contains the config file names of MTML and MTMR
$rosrun dvrk_robot dvrk_console_json -j <path_to_your_console_config.json>
```
In dVRK console, press `home` button to turn on mtm arms and move them to home position.

In matlab 
```
addpath('<path-to-dvrk_matlab>');
rosinit;
```
for two MTM arms cases:
```
gc_demo('MTML&MTMR')
```

For MTML arms case:
```
gc_demo('MTML')
```

For MTMR arms case:
```
gc_demo('MTMR')
```

If you have done dataCollection and MLSE, you can run gravity controller by calling GC_controller() function, For example:
```
GC_controller('MTML&MTMR', 'GC_controller_config.json','../GC_Data_test','4POL',9.81)
```

More Information:

Code overview can be found [here](https://github.com/CUHK-BRME/dvrk-ros-1/wiki/Code-Overview) .

Theory of dVRK GC can be found in [here](https://github.com/CUHK-BRME/dvrk-ros-1/wiki/GC-Procedure).

## Prerequisite Setting Wizard

After calling MAIN() function in matlab, the program will step into Prerquisite Setting Wizard automatically. The wizard program is to set user's neccesary customized parameters of dvrk. It is of great importance because dvrk might be install in different scenarios and it might hit environment if you just use default setting. [Prerequisite Setting Wizard](https://github.com/CUHK-BRME/dvrk-ros-1/wiki/Prerequisite-Setting) before running MATLAB scripts.

## Contact
Feel free to contact us.  

Hongbin LIN:  [hongbinlin@cuhk.edu.hk](hongbinlin@cuhk.edu.hk)

Vincent Hui: [vincent.hui@cuhk.edu.hk](vincent.hui@cuhk.edu.hk) 

## Acknowledgements
The software has been developed with the support of BRME,CUHK.

Some source files are copied or modified from [dvrk_gravity_comp
 of WPI AIM Lab](https://github.com/WPI-AIM/dvrk_gravity_comp/tree/master/Final_Submission/MATLAB%20code)
