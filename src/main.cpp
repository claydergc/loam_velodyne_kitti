#include "loam_velodyne/MultiScanRegistration.h"
#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/TransformMaintenance.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>

#include <chrono>
#include <ctime>

std::string getSequenceStr(int sequence)
{

	if(sequence<=9)
		return "0"+std::to_string(sequence);
	else
		return std::to_string(sequence);
}

std::string getFrameStr(int frame)
{
	if(frame>9999)
		return "0"+std::to_string(frame);
	else if(frame>999)
		return "00"+std::to_string(frame);
	else if(frame>99)
		return "000"+std::to_string(frame);
	else if(frame>9)
		return "0000"+std::to_string(frame);
	else if(frame<=9)
		return "00000"+std::to_string(frame);
}

/** Main node entry point. */
int main(int argc, char **argv)
{
  /*loam::MultiScanRegistration multiScan;
  multiScan.setup();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile (argv[1], cloud);
  multiScan.processCloudFrame(cloud, 0.0);//count time after this function to feed multiScan functions

  loam::LaserOdometry laserOdom(0.1);
  laserOdom.setLaserCloudFullRes(multiScan.getLaserCloud(0.0));
  laserOdom.setLaserCloudSharp(multiScan.getCornerPointsSharp(0.0));
  laserOdom.setLaserCloudLessSharp(multiScan.getCornerPointsLessSharp(0.0));
  laserOdom.setLaserCloudFlat(multiScan.getSurfacePointsFlat(0.0));
  laserOdom.setLaserCloudLessFlat(multiScan.getSurfacePointsLessFlat(0.0));
  laserOdom.process();

  loam::Transform transform = laserOdom.getOdometryToInitTransform(0.0);*/


  std::ofstream out;
  out.open ("odometry.txt");
    
  pcl::PointCloud<pcl::PointXYZ> cloud;

  loam::MultiScanRegistration multiScan;
  multiScan.setup();
  loam::LaserOdometry laserOdom(0.1);
  loam::LaserMapping laserMapping(0.1);
  loam::TransformMaintenance transMaintenance;

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  int currentFrame = 0;

  std::string path = argv[1];//load path
  std::string sequence = argv[2];//load sequence
  //std::string file = path + "/" + sequence + "/velodyne/" + getFrameStr(currentFrame) + ".bin";
  std::string file = path + "/data_odometry_velodyne/dataset/" + sequence + "/velodyne/" + getFrameStr(currentFrame) + ".bin";

  // load point cloud
  FILE *stream;
  stream = fopen (file.c_str(),"rb");

  std::cout<<file<<std::endl;


  std::string timesFileStr = path + "/data_odometry_calib/dataset/sequences/" + sequence + "/times.txt";
  std::ifstream timesFile(timesFileStr);
  std::string timeStr;
  float time;

  std::chrono::high_resolution_clock::time_point t0,t1;
  float stamp;

  while(stream!=NULL)//read all .bin files in a sequence
  //for(int i=0; i<10; ++i)
  {
	  num = fread(data,sizeof(float),num,stream)/4;

	  for (int32_t i=0; i<num; i++) {
	    pcl::PointXYZ point;
	    point.x = *px;
	    point.y = *py;
	    point.z = *pz;
	    //point.intensity = *pr;
	    cloud.push_back(point);
	    px+=4; py+=4; pz+=4; pr+=4;
	  }

	  //pcl::io::savePCDFileASCII ("cloud.pcd", cloud);

	  //std::cout<<"cloud size 0: "<<cloud.size()<<std::endl;

	  getline (timesFile,timeStr);
	  time = std::stof(timeStr);

	  t0 = std::chrono::high_resolution_clock::now();
	  multiScan.processCloudFrame(cloud, time);//count time after this function to feed multiScan functions
	  t1 = std::chrono::high_resolution_clock::now();
	  
	  //std::cout<<"size: "<<multiScan.getLaserCloud(time).first.size()<<std::endl;
	  stamp = time + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserOdom.setLaserCloudSharp(multiScan.getCornerPointsSharp(stamp));
	  t1 = std::chrono::high_resolution_clock::now();
	  
	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserOdom.setLaserCloudLessSharp(multiScan.getCornerPointsLessSharp(stamp));
	  t1 = std::chrono::high_resolution_clock::now();

	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserOdom.setLaserCloudFlat(multiScan.getSurfacePointsFlat(stamp));
	  t1 = std::chrono::high_resolution_clock::now();

	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserOdom.setLaserCloudLessFlat(multiScan.getSurfacePointsLessFlat(stamp));
	  t1 = std::chrono::high_resolution_clock::now();
	  
	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserOdom.setLaserCloudFullRes(multiScan.getLaserCloud(stamp));
	  t1 = std::chrono::high_resolution_clock::now();
	  
	  //Get topic /laser_odom_to_init
	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserOdom.process();
	  loam::TransformFrame transformFrameLaser = laserOdom.getOdometryToInitTransform(stamp); //Odometry topic /laser_odom_to_init
	  t1 = std::chrono::high_resolution_clock::now();

	  //Begin Get ready for LaserMapping
	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserMapping.setLaserCloudCornerLast(laserOdom.getLastCornerCloud(stamp));
	  t1 = std::chrono::high_resolution_clock::now();

	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserMapping.setLaserCloudSurfLast(laserOdom.getLastSurfaceCloud(stamp));
	  t1 = std::chrono::high_resolution_clock::now();

	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserMapping.setLaserCloudFullRes(laserOdom.getLaserCloud(stamp));
	  t1 = std::chrono::high_resolution_clock::now();

	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserMapping.setLaserOdometry(transformFrameLaser);
	  t1 = std::chrono::high_resolution_clock::now();
	  //End Get ready for LaserMapping

	  stamp = stamp + std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0).count();
	  t0 = std::chrono::high_resolution_clock::now();
	  laserMapping.process();
	  loam::TransformFrame transformFrameMapping = laserMapping.getOdometryAftMappedTransform(stamp); //Odometry topic /aft_mapped_to_init
	  loam::Velocity velocity = laserMapping.getOdometryAftMappedVelocity();
	  t1 = std::chrono::high_resolution_clock::now();

	  transMaintenance.setOdomAftMapped(transformFrameMapping, velocity);
	  loam::TransformFrame integratedTransform = transMaintenance.updateLaserOdometry(transformFrameLaser);


	  out << integratedTransform.first.matrix()(0,0)<<" "<<integratedTransform.first.matrix()(0,1) << " " << integratedTransform.first.matrix()(0,2) << " " << integratedTransform.first.matrix()(0,3) << " " <<
	  	     integratedTransform.first.matrix()(1,0)<<" "<<integratedTransform.first.matrix()(1,1) << " " << integratedTransform.first.matrix()(1,2) << " " << integratedTransform.first.matrix()(1,3) << " " <<
	  	     integratedTransform.first.matrix()(2,0)<<" "<<integratedTransform.first.matrix()(2,1) << " " << integratedTransform.first.matrix()(2,2) << " " << integratedTransform.first.matrix()(2,3) << "\n";

	  /*out << transform.matrix()(0,0)<<" "<<transform.matrix()(0,1) << " " << transform.matrix()(0,2) << " " << transform.matrix()(0,3) << " " <<
	  	     transform.matrix()(1,0)<<" "<<transform.matrix()(1,1) << " " << transform.matrix()(1,2) << " " << transform.matrix()(1,3) << " " <<
	  	     transform.matrix()(2,0)<<" "<<transform.matrix()(2,1) << " " << transform.matrix()(2,2) << " " << transform.matrix()(2,3) << "\n";*/

	  //out << transform.matrix()(0,3)<<" "<<transform.matrix()(2,3) << "\n";
	  
      
	  cloud.clear();
	  fclose(stream);
	  currentFrame++;
	  file = path + "/data_odometry_velodyne/dataset/" + sequence + "/velodyne/" + getFrameStr(currentFrame) + ".bin";
	  std::cout<<file<<std::endl;
	  fflush(stream);
	  stream = fopen (file.c_str(),"rb");
	  free(data);
	  num = 1000000;
	  data = (float*)malloc(num*sizeof(float));
	  px = data+0;
  	  py = data+1;
      pz = data+2;
      pr = data+3;
      //multiScan = loam::MultiScanRegistration();
      //multiScan.setup();
  }

  free(data);
  out.close();

  //std::cout<<multiScan.getLaserCloud(0.0).first.size()<<std::endl;
  
  //pcl::io::savePCDFileASCII("cloud.pcd", multiScan.getLaserCloud(0.0).first);

  //pcl::io::savePCDFileASCII("corners.pcd", multiScan.getCornerPointsSharp(0.0).first);

  //pcl::io::savePCDFileASCII ("ring.pcd", multiScan._laserCloudScans[0]);

  return 0;
}
