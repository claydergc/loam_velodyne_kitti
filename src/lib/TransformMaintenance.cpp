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

#include "loam_velodyne/TransformMaintenance.h"

namespace loam
{

TransformMaintenance::TransformMaintenance()
{
   // initialize odometry and odometry tf messages
   //_laserOdometry2.header.frame_id = "/camera_init";
   //_laserOdometry2.child_frame_id = "/camera";

   //_laserOdometryTrans2.frame_id_ = "/camera_init";
   //_laserOdometryTrans2.child_frame_id_ = "/camera";
}


/*bool TransformMaintenance::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
{
   // advertise integrated laser odometry topic
   _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry>("/integrated_to_init", 5);

   // subscribe to laser odometry and mapping odometry topics
   _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &TransformMaintenance::laserOdometryHandler, this);

   _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &TransformMaintenance::odomAftMappedHandler, this);

   return true;
}*/



TransformFrame TransformMaintenance::updateLaserOdometry(const TransformFrame& laserOdometry)
{
   double roll, pitch, yaw;
   //geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   //tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   Eigen::Matrix3f rotation;
   rotation = laserOdometry.first.rotation();

   Eigen::Vector3f ea = rotation.eulerAngles(0, 1, 2);

   roll = ea[0];
   pitch = ea[1]; 
   yaw = ea[2];

   updateOdometry(-pitch, -yaw, roll,
      laserOdometry.first.matrix()(0,3),
      laserOdometry.first.matrix()(1,3),
      laserOdometry.first.matrix()(2,3));

   transformAssociateToMap();

   //geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped()[2], -transformMapped()[0], -transformMapped()[1]);

   Eigen::Quaternionf geoQuat;
   geoQuat = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ());

   transOdomIntegrated.translation() << transformMapped()[3], transformMapped()[4], transformMapped()[5];
   transOdomIntegrated.rotate(Eigen::Quaternionf(geoQuat.w(), -geoQuat.y(), -geoQuat.z(), geoQuat.x()));

   return getTransformFrame(transOdomIntegrated, laserOdometry.second);
}

void TransformMaintenance::setOdomAftMapped(const TransformFrame& odomAftMapped, const Velocity& velocityAftMapped)
{
   double roll, pitch, yaw;

   Eigen::Matrix3f rotation;
   rotation = odomAftMapped.first.rotation();

   Eigen::Vector3f ea = rotation.eulerAngles(0, 1, 2);

   roll = ea[0];
   pitch = ea[1]; 
   yaw = ea[2];


   updateMappingTransform(-pitch, -yaw, roll,
      odomAftMapped.first.matrix()(0,3),
      odomAftMapped.first.matrix()(1,3),
      odomAftMapped.first.matrix()(2,3),

      velocityAftMapped.first[0],
      velocityAftMapped.first[1],
      velocityAftMapped.first[2],

      velocityAftMapped.second[0],
      velocityAftMapped.second[1],
      velocityAftMapped.second[2]);
}

} // end namespace loam
