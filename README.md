## Corner detection in sparsely featured rooms

Detects corners in a point cloud and gives their locations relative to the sensor.

The registration is done using ICP on Kinect point clouds. The Kinect is mounted on
a servo and rotated at known angles which are passed as hints to ICP algorithm.
From the registered 3d map, planes are segmented out which are in turn used for
detecting corners of the room. The corners can then be used for path planning or
localization.
This gives good results even in environments with limited or sparse features.

## Build
Depends on `freenect2` for integration with Kinect, OpenCV and PCL for point
cloud operations.
```
$ cd build
$ cmake ..
$ make -j
```
