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

#include "loam_velodyne/ScanRegistration.h"
#include "math_utils.h"


namespace loam {

PointCloudFrame ScanRegistration::getLaserCloud(float stamp)
{
  return getCloudFrame(laserCloud(), stamp);
}

PointCloudFrame ScanRegistration::getCornerPointsSharp(float stamp)
{
  return getCloudFrame(cornerPointsSharp(), stamp);
}

PointCloudFrame ScanRegistration::getCornerPointsLessSharp(float stamp)
{
  return getCloudFrame(cornerPointsLessSharp(), stamp);
}

PointCloudFrame ScanRegistration::getSurfacePointsFlat(float stamp)
{
  return getCloudFrame(surfacePointsFlat(), stamp);
}

PointCloudFrame ScanRegistration::getSurfacePointsLessFlat(float stamp)
{
  return getCloudFrame(surfacePointsLessFlat(), stamp);
}

/*void ScanRegistration::publishResult()
{
  auto sweepStartTime = toROSTime(sweepStart());
  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, "/camera");

  // publish corresponding IMU transformation information
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, "/camera");
}*/

} // end namespace loam
