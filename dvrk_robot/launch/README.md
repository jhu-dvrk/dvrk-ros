launch scripts
==============

* dvrk_arm_rviz.launch:
  * Starts a console application with Qt GUI as well as RViz
  * Parameters:
    * `arm:=`: arm name, i.e. ECM, MTML, MTMR, PSM1, ...
    * `config:=`: full path to console.json configuration file.  The console can use an actual arm or a simulated arm (i.e. no need for physical arm)
* dvrk_arm_rviz_only.launch:
  * Only starts RViz, assumes the console is already started - convenient if you have a single simulated arm
  * Parameter:
    * `arm:=`: arm name, i.e. ECM, MTML, MTMR, PSM1, ...
* dvrk_master_slave_rviz.launch
  * Starts a console application with Qt GUI as well as RViz
  * Parameters:
    * `master:=`: master name, i.e. MTML or MTMR
    * `slave:=`: master name, i.e. PSM1, PSM2 or PSM3
    * `config:=`: full path to console.json configuration file with teleoperation for a master and slave (names must match `master` and `slave` parameters).  The console can use an actual PSM or a simulated one (i.e. no need for physical arm)
