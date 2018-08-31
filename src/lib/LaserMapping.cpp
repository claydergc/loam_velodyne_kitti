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

#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/common.h"

namespace loam
{

LaserMapping::LaserMapping(const float& scanPeriod, const size_t& maxIterations):
   BasicLaserMapping(scanPeriod, maxIterations)
{

}

void LaserMapping::setLaserCloudCornerLast(const PointCloudFrame& cloudFrame)
{
   //std::cout<<"hola"<<std::endl;
   _timeLaserCloudCornerLast = cloudFrame.second;
   laserCloudCornerLast()->clear();
   //std::cout<<"hola 2"<<std::endl;
   *laserCloudCornerLast() = cloudFrame.first;
   //*_laserCloudCornerLast = cloudFrame.first;
   //std::cout<<"hola 3"<<std::endl;
   _newLaserCloudCornerLast = true;
}

void LaserMapping::setLaserCloudSurfLast(const PointCloudFrame& cloudFrame)
{
   _timeLaserCloudSurfLast = cloudFrame.second;
   laserCloudSurfLast()->clear();
   *laserCloudSurfLast() = cloudFrame.first;
   _newLaserCloudSurfLast = true;
}

void LaserMapping::setLaserCloudFullRes(const PointCloudFrame& cloudFrame)
{
   _timeLaserCloudFullRes = cloudFrame.second;
   laserCloud()->clear();
   *laserCloud() = cloudFrame.first;
   _newLaserCloudFullRes = true;
}

void LaserMapping::setLaserOdometry(const TransformFrame& transformFrame)
{
   _timeLaserOdometry = transformFrame.second;

   float roll, pitch, yaw;
   //geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;//getRotation
   //tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   Eigen::Matrix3f rotation;
   rotation = transformFrame.first.rotation();

   Eigen::Vector3f ea = rotation.eulerAngles(0, 1, 2);

   roll = ea[0];
   pitch = ea[1]; 
   yaw = ea[2]; 


   /*updateOdometry(-pitch, -yaw, roll,
                  laserOdometry->pose.pose.position.x,
                  laserOdometry->pose.pose.position.y,
                  laserOdometry->pose.pose.position.z);*/

   updateOdometry(-pitch, -yaw, roll,
                  transformFrame.first.matrix()(0,3),
                  transformFrame.first.matrix()(1,3),
                  transformFrame.first.matrix()(2,3));

   _newLaserOdometry = true;
}

/*void LaserMapping::spin()
{
   ros::Rate rate(100);
   bool status = ros::ok();

   while (status)
   {
      ros::spinOnce();

      // try processing buffered data
      process();

      status = ros::ok();
      rate.sleep();
   }
}*/

void LaserMapping::reset()
{
   _newLaserCloudCornerLast = false;
   _newLaserCloudSurfLast = false;
   _newLaserCloudFullRes = false;
   _newLaserOdometry = false;
}

bool LaserMapping::hasNewData()
{
   return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
      _newLaserCloudFullRes && _newLaserOdometry &&
      fabs((_timeLaserCloudCornerLast - _timeLaserOdometry)) < 0.005 &&
      fabs((_timeLaserCloudSurfLast - _timeLaserOdometry)) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeLaserOdometry)) < 0.005;
}

void LaserMapping::process()
{
   /*if (!hasNewData())// waiting for new data to arrive...
   {
      std::cout<<"no new mapping data"<<std::endl;
      return;
   }*/

   reset();// reset flags, etc.

   //if (!BasicLaserMapping::process(fromROSTime(_timeLaserOdometry)))
   if (!BasicLaserMapping::process())
   {
      std::cout<<"mapping process error"<<std::endl;
      return;
   }

   //publishResult();
}

TransformFrame LaserMapping::getOdometryAftMappedTransform(float stamp)
{
   //Transform transOdom;
   //transOdom = Transform::Identity();
   transOdom.translation() << transformAftMapped().pos.x(), transformAftMapped().pos.y(), transformAftMapped().pos.z();
   Eigen::Quaternionf q;
     q = Eigen::AngleAxisf(transformAftMapped().rot_z.rad(), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(-transformAftMapped().rot_x.rad(), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(-transformAftMapped().rot_y.rad(), Eigen::Vector3f::UnitZ());
   transOdom.rotate(Eigen::Quaternionf(q.w(), -q.y(), -q.z(), q.x()));

   //Eigen::MatrixXf transformMatrix;
   //transformMatrix = transOdom.matrix();
   //std::cout<<transformMatrix.rows()<<","<<transformMatrix.cols()<<std::endl;

   //return transOdom;
   return getTransformFrame(transOdom, stamp);
}

Velocity LaserMapping::getOdometryAftMappedVelocity()
{
   Eigen::Vector3f angular(transformBefMapped().rot_x.rad(), transformBefMapped().rot_y.rad(), transformBefMapped().rot_z.rad());
   Eigen::Vector3f linear(transformBefMapped().pos.x(), transformBefMapped().pos.y(), transformBefMapped().pos.z());

   return getVelocity(angular, linear);
}

/*void LaserMapping::publishResult()
{
   // publish new map cloud according to the input output ratio
   if (hasFreshMap()) // publish new map cloud
      publishCloudMsg(_pubLaserCloudSurround, laserCloudSurroundDS(), _timeLaserOdometry, "/camera_init");

   // publish transformed full resolution input cloud
   publishCloudMsg(_pubLaserCloudFullRes, laserCloud(), _timeLaserOdometry, "/camera_init");

   // publish odometry after mapped transformations
   geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
   (transformAftMapped().rot_z.rad(), -transformAftMapped().rot_x.rad(), -transformAftMapped().rot_y.rad());

   _odomAftMapped.header.stamp = _timeLaserOdometry;
   _odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
   _odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
   _odomAftMapped.pose.pose.orientation.z = geoQuat.x;
   _odomAftMapped.pose.pose.orientation.w = geoQuat.w;
   _odomAftMapped.pose.pose.position.x = transformAftMapped().pos.x();
   _odomAftMapped.pose.pose.position.y = transformAftMapped().pos.y();
   _odomAftMapped.pose.pose.position.z = transformAftMapped().pos.z();
   _odomAftMapped.twist.twist.angular.x = transformBefMapped().rot_x.rad();
   _odomAftMapped.twist.twist.angular.y = transformBefMapped().rot_y.rad();
   _odomAftMapped.twist.twist.angular.z = transformBefMapped().rot_z.rad();
   _odomAftMapped.twist.twist.linear.x = transformBefMapped().pos.x();
   _odomAftMapped.twist.twist.linear.y = transformBefMapped().pos.y();
   _odomAftMapped.twist.twist.linear.z = transformBefMapped().pos.z();
   _pubOdomAftMapped.publish(_odomAftMapped);

   _aftMappedTrans.stamp_ = _timeLaserOdometry;
   _aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
   _aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped().pos.x(),
                                         transformAftMapped().pos.y(),
                                         transformAftMapped().pos.z()));
   _tfBroadcaster.sendTransform(_aftMappedTrans);
}*/

} // end namespace loam
