# uav_locator
For locating a UAV.

## Usage

1.  Place uav_locator.hpp and uav_locator.cpp in the same folder. Place a sample
    image in the same folder, name it sample.png. This sample image *must* be a
    black profile image of the UAV to identify with a white background.

2.  Include uav_locator.hpp in main project.

3.  Run function init_uav_locator() *before* doing anything else regarding
    uav_locator! The program *will* fail silently/with exit code -1!

4.  Call uav_locator() with a Mat image as argument. The function will return
    a custom struct, p3d, containing contour mass center x and y coordinates,
    angle in radians (-pi to pi), and (if implemented by the user) the
    approximate size.
    Repeat "ad nauseam".
