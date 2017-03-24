# stereoPoseEKF

This is a prototype code for solveing visual odometry from stereo vision camera.

This code is compiled under linux c++ enviornment.

The following library is required for compile:
vtk
opecv
pcl
boost
armadillo
Eigen

Run the programe under the folder where the stereo image store.
The stereo image must be named in the following format:

Left1.ppm
Right1.ppm

The number following Left/Right is the frame number,
could be start with arbitary value with argument :

-i number_of_first_frame

example: -i 1 (image index start at 1)

image file extenstion can be change with argument :

-e image_file_extention 

example : -e ppm 

