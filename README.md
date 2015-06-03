<head>
    <script type="text/javascript"
            src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML">
    </script>
</head>
# Calculates 3D Points from a stereo image pair
This small program is actually the rewritten code of some parts of another repository of mine ([6DOF Stereo EKF SLAM](https://github.com/srv/6dof_stereo_ekf_slam)) in C++ instead of MatLab.

## what it does
It takes a stereo image pair, in this case given by the stereo camera rig Bumblebee2 and the corresponding reprojection matrix Q to calculate 3D Points from corresponding SIFT Features.

## usage
    $ ./stereo3Dcalculation <Left_Image_Path> <Right_Image_Path> <output.bin>

## requires
* cmake 2.8
* openCV
* gcc

## tested on
* Ubuntu 14.04
* gcc 4.8.2
* cmake 2.8.12.2
* openCV 2.4.8

## to do
* code clean up 
* code modularisation

## literature
* Siciliano, Bruno, and Oussama Khatib. _Springer Handbook of Robotics_. Berlin: Springer, 2008. Print.

## example output
![Example Output](/stere_matching.png?raw=true "Example Output")

