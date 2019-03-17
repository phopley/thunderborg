^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thunderborg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2019-03-17)
------------------
* Added the height of the base_link above ground to odom
* Odometry topic now named /raw_odom and not /odom. Part of adding EKF to fuse IMU data
* odom /tf now not broadcast from this node. Part of adding EKF to fuse IMU data
* Parameters moved to /thunderborg_node ns
* tranform tree now odom<-bade_footprint<-base_link

0.1.1 (2019-01-28)
------------------
* Add test code file test.py
* Corrected name of function DynamicCallback from DynamicCallbak
* Corrected name circumference from circumfrence
* In thunderborg.cfg changed D-Integral to D-Derivative

0.1.0 (2019-01-23)
------------------
* First formal release of the package
