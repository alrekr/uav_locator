# uav_locator
For locating a UAV.

## Prerequisites
The code is dependant on OpenCV. Built and tested with CMake/GCC using Atom on Ubuntu 14.04.

## Technical
Tested to perform at at least 30 FPS.

## Usage

1.  Place [uav_locator.hpp] and [uav_locator.cpp] in the same folder. Place a sample image in the same folder, name it [sample.png]. This sample image *must* be a black profile image of the UAV to identify with a white background, as it is to be used for identifying a UAV.

2.  Include [uav_locator.hpp] in main project.

3.  Run function 'init_uav_locator()' *before* doing anything else regarding 'uav_locator()'! The program *will* fail silently/with exit code -1 if you neglect to run 'init_uav_locator()'!

4.  Call 'uav_locator()' with a Mat image as argument. The function will return an object of type cv::Point3d containing contour mass center x and y coordinates and angle in radians (-pi to pi). Repeat "ad nauseam". 
