# uav_locator
For locating a UAV.

## Usage

1.  Place [uav_locator.hpp] and [uav_locator.cpp] in the same folder. Place a sample image in the same folder, name it sample.png. This sample image *must* be a black profile image of the UAV to identify with a white background.

2.  Include [uav_locator.hpp] in main project.

3.  Optionally run `init_locate_uav()`. This function will be called automatically first time `locate_uav()` is called, if not called manually.

4.  Call `locate_uav()` with a *Mat* image as argument. The function will return an object of type *Point3d* (from OpenCV) containing contour mass center *x* and *y* coordinates and angle *z* in radians. Repeat "ad nauseam".
