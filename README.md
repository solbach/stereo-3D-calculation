# Calculate 3D Points from a stereo image pair
This small program is actually the rewritten code of some parts of another repository of mine ((6DOF Stereo EKF SLAM)[https://github.com/srv/6dof_stereo_ekf_slam]) in C++ using openCV instead of MatLab.

## what it does
it takes a stereo image pair, in this case given by the stereo camera rig bumblebee2 and the corresponding reprojection matrix Q to calculate 3D Points from corresponding SIFT Features.

## requires
* cmake 2.8
* openCV
* gcc

## tested on
* Ubuntu 14.04
* gcc 4.8.2
* cmake 2.8.12.2
* openCV 2.4.8

## literature
* Siciliano, Bruno, and Oussama Khatib. _Springer Handbook of Robotics_. Berlin: Springer, 2008. Print.
