// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <pcl/filters/filter.h>

#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/common.h"
#include "math_utils.h"

namespace loam
{

  using std::sin;
  using std::cos;
  using std::asin;
  using std::atan2;
  using std::sqrt;
  using std::fabs;
  using std::pow;


  LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations):
    BasicLaserOdometry(scanPeriod, maxIterations),
    _ioRatio(ioRatio)
  {
  }

  void LaserOdometry::reset()
  {
    _newCornerPointsSharp = false;
    _newCornerPointsLessSharp = false;
    _newSurfPointsFlat = false;
    _newSurfPointsLessFlat = false;
    _newLaserCloudFullRes = false;
    _newImuTrans = false;
  }

  void LaserOdometry::setLaserCloudSharp(const PointCloudFrame& cloudFrame) //be careful with time. Maybe it will be necessary to be calculated as the time that pass after receiving the point cloud 
  {
    _timeCornerPointsSharp = cloudFrame.second;
    cornerPointsSharp()->clear();
    *cornerPointsSharp() = cloudFrame.first;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsSharp(), *cornerPointsSharp(), indices);
    _newCornerPointsSharp = true;
  }



  void LaserOdometry::setLaserCloudLessSharp(const PointCloudFrame& cloudFrame)
  {
    _timeCornerPointsLessSharp = cloudFrame.second;
    cornerPointsLessSharp()->clear();
    *cornerPointsLessSharp() = cloudFrame.first;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp(), *cornerPointsLessSharp(), indices);
    _newCornerPointsLessSharp = true;
  }



  void LaserOdometry::setLaserCloudFlat(const PointCloudFrame& cloudFrame)
  {
    _timeSurfPointsFlat = cloudFrame.second;
    surfPointsFlat()->clear();
    *surfPointsFlat() = cloudFrame.first;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsFlat(), *surfPointsFlat(), indices);
    _newSurfPointsFlat = true;
  }



  void LaserOdometry::setLaserCloudLessFlat(const PointCloudFrame& cloudFrame)
  {
    _timeSurfPointsLessFlat = cloudFrame.second;
    surfPointsLessFlat()->clear();
    *surfPointsLessFlat() = cloudFrame.first;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat(), *surfPointsLessFlat(), indices);
    _newSurfPointsLessFlat = true;
  }



  void LaserOdometry::setLaserCloudFullRes(const PointCloudFrame& cloudFrame)
  {
    _timeLaserCloudFullRes = cloudFrame.second;
    laserCloud()->clear();
    *laserCloud() = cloudFrame.first;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);
    _newLaserCloudFullRes = true;
  }



  /*void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
  {
    _timeImuTrans = imuTransMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZ> imuTrans;
    pcl::fromROSMsg(*imuTransMsg, imuTrans);
    updateIMU(imuTrans);
    _newImuTrans = true;
  }


  void LaserOdometry::spin()
  {
    ros::Rate rate(100);
    bool status = ros::ok();

    // loop until shutdown
    while (status)
    {
      ros::spinOnce();

      // try processing new data
      process();

      status = ros::ok();
      rate.sleep();
    }
  }*/


  bool LaserOdometry::hasNewData()
  {

  	//std::cout<<( fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat)) < 0.005 )<<std::endl;
    //std::cout<<( fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat)) < 0.005 )<<std::endl;
    //std::cout<<( fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat)) < 0.005 )<<std::endl;
    //std::cout<<( fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat)) < 0.005 )<<std::endl;

    return _newCornerPointsSharp && _newCornerPointsLessSharp && _newSurfPointsFlat &&
      _newSurfPointsLessFlat && _newLaserCloudFullRes &&
      fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat)) < 0.005 &&
      fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat)) < 0.005 &&
      fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat)) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat)) < 0.005;
      //&&
      //fabs((_timeImuTrans - _timeSurfPointsLessFlat)) < 0.005;
  }



  void LaserOdometry::process()
  {
  	//std::cout<<"holaMundo0"<<std::endl;
    //if (!hasNewData())
    //{
      //std::cout<<"no new data"<<std::endl;
      //return;// waiting for new data to arrive...
    //}

    reset();// reset flags, etc.
    BasicLaserOdometry::process();
    //std::cout<<"holaMundo"<<std::endl;
    //publishResult();
  }

  PointCloudFrame LaserOdometry::getLastCornerCloud(float stamp)
  {
  	// publish cloud results according to the input output ratio
    //if (_ioRatio < 2 || frameCount() % _ioRatio == 1)
  		return getCloudFrame(*lastCornerCloud(), stamp);
  }

  PointCloudFrame LaserOdometry::getLastSurfaceCloud(float stamp)
  {
  	// publish cloud results according to the input output ratio
    //if (_ioRatio < 2 || frameCount() % _ioRatio == 1)
  		return getCloudFrame(*lastCornerCloud(), stamp);
  }

  PointCloudFrame LaserOdometry::getLaserCloud(float stamp)
  {
  	// publish cloud results according to the input output ratio
    //if (_ioRatio < 2 || frameCount() % _ioRatio == 1)
    //{
    	transformToEnd(laserCloud());  // transform full resolution cloud to sweep end before sending it
  		return getCloudFrame(*laserCloud(), stamp);
    //}
  }

  //Transform LaserOdometry::getOdometryToInitTransform(float stamp)
  TransformFrame LaserOdometry::getOdometryToInitTransform(float stamp)
  {
  	//Transform transOdom;
  	//transOdom = Transform::Identity();
  	transOdom.translation() << transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z();
  	Eigen::Quaternionf q;
	  q = Eigen::AngleAxisf(transformSum().rot_z.rad(), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(-transformSum().rot_x.rad(), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(-transformSum().rot_y.rad(), Eigen::Vector3f::UnitZ());
  	transOdom.rotate(Eigen::Quaternionf(q.w(), -q.y(), -q.z(), q.x()));

  	//Eigen::MatrixXf transformMatrix;
  	//transformMatrix = transOdom.matrix();
  	//std::cout<<transformMatrix.rows()<<","<<transformMatrix.cols()<<std::endl;

  	//return transOdom;
    return getTransformFrame(transOdom, stamp);
  }
 

  /*void LaserOdometry::publishResult()
  {
    // publish odometry transformations
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(),
                                                                               -transformSum().rot_x.rad(),
                                                                               -transformSum().rot_y.rad());

    _laserOdometryMsg.header.stamp            = _timeSurfPointsLessFlat;
    _laserOdometryMsg.pose.pose.orientation.x = -geoQuat.y;
    _laserOdometryMsg.pose.pose.orientation.y = -geoQuat.z;
    _laserOdometryMsg.pose.pose.orientation.z = geoQuat.x;
    _laserOdometryMsg.pose.pose.orientation.w = geoQuat.w;
    _laserOdometryMsg.pose.pose.position.x    = transformSum().pos.x();
    _laserOdometryMsg.pose.pose.position.y    = transformSum().pos.y();
    _laserOdometryMsg.pose.pose.position.z    = transformSum().pos.z();
    _pubLaserOdometry.publish(_laserOdometryMsg);

    _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;
    _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    _laserOdometryTrans.setOrigin(tf::Vector3(transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z()));
    _tfBroadcaster.sendTransform(_laserOdometryTrans);

    // publish cloud results according to the input output ratio
    if (_ioRatio < 2 || frameCount() % _ioRatio == 1)
    {
      ros::Time sweepTime = _timeSurfPointsLessFlat;
      publishCloudMsg(_pubLaserCloudCornerLast, *lastCornerCloud(), sweepTime, "/camera");
      publishCloudMsg(_pubLaserCloudSurfLast, *lastSurfaceCloud(), sweepTime, "/camera");

      transformToEnd(laserCloud());  // transform full resolution cloud to sweep end before sending it
      publishCloudMsg(_pubLaserCloudFullRes, *laserCloud(), sweepTime, "/camera");
    }
  }*/

} // end namespace loam
