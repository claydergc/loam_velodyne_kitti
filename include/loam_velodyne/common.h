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

#ifndef LOAM_COMMON_H
#define LOAM_COMMON_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "time_utils.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <iostream>

namespace loam {


typedef std::pair<pcl::PointCloud<pcl::PointXYZI>, float> PointCloudFrame;

typedef Eigen::Affine3f Transform;

typedef std::pair<Transform, float> TransformFrame;

typedef std::pair<Eigen::Vector3f, Eigen::Vector3f> Velocity;

//typedef std::pair<pcl::PointCloud<PointT>, float> PointCloudFrame;

/** \brief Construct a new point cloud frame from the specified information and publish it via the given publisher.
 *
 * @tparam PointT the point type
 * @param cloud the cloud to return
 * @param stamp the time stamp of the cloud frame
 */
inline PointCloudFrame getCloudFrame(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                            const float& stamp) {
  return PointCloudFrame(cloud, stamp);
}

inline TransformFrame getTransformFrame(const Transform& transform,
                            const float& stamp) {
  return TransformFrame(transform, stamp);
}

inline Velocity getVelocity(const Eigen::Vector3f& angular, const Eigen::Vector3f& linear) {
  return Velocity(angular, linear);
}


} // end namespace loam

#endif // LOAM_COMMON_H
