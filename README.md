# loam_velodyne_kitti
 This package is a simple modified copy of [loam_velodyne git repository](https://github.com/laboshinl/loam_velodyne) from **laboshinl** to work with the KITTI dataset.
 
 Main changes have been done in:
 
ScanRegistration.cpp
MultiScanRegistration.cpp
LaserOdometry.cpp
LaserMapping.cpp
TransformMaintenance.cpp

where there are functions that return transform matrices based on the Eigen library instead of ROS odometry topics.
 
How to build:

```
$ mkdir build
$ cd build
$ make
```
Running:
```
$ ./main "KITTI Dataset Path" "NumberOfSequence"
