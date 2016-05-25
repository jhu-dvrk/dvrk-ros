### dvrk_kinematics package

**launch:**  
 - test\_mtm\_logic.launch: start mtm\_right\_only visulization and a MTM kinematics node  
 - test\_psm\_logic.launch: start psm\_rviz visulization and a PSM kinematics node  
   
**src:**    
source directory with 1 MTM & 1 PSM kinematics class and sim_mtm_pose_correction: 
**control mode:**  
Both MTM/PSM have severl control modes as listed below:

# How to run sim_mtm_pose_correction
Just type rosrun sim_mtm sim_mtm_node to run this package independantly. However, this package will
be automatically fired up using roslauch dvrk_robot test_dvrk_mtm.launch.
