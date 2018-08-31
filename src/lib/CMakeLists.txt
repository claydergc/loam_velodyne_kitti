add_library(loam
            math_utils.h
            BasicScanRegistration.cpp
	    ScanRegistration.cpp
            MultiScanRegistration.cpp
	    BasicLaserOdometry.cpp
	    LaserOdometry.cpp
            BasicLaserMapping.cpp
	    LaserMapping.cpp
            BasicTransformMaintenance.cpp
	    TransformMaintenance.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
