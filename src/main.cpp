/**
 * @file main.cpp
 * @author huizeyu (huizeyu@siasun.com)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <glog/logging.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/impl/point_types.hpp>

const std::string pcd_path = "/home/hzy/code/rt_map/";
const std::string loadName = "pose6d.pcd";
const std::string saveName = "pose6d_r.pcd";

struct EIGEN_ALIGN16 PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time,
                                                                 time))

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;

  // clang-format off
  Eigen::Affine3f transMat = pcl::getTransformation(
      1.25, 
      0.5275, 
      0, 
      0 * M_PI / 180.0f, 
      0 * M_PI / 180.0f, 
      45.8 * M_PI / 180.0f);
  // clang-format on

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  LOG(INFO) << "Current down size leaf size: " << downSizeFilter.getLeafSize();

  pcl::PointCloud<PointXYZIRPYT>::Ptr read_cloud(
      new pcl::PointCloud<PointXYZIRPYT>);
  pcl::PointCloud<PointXYZIRPYT>::Ptr write_cloud(
      new pcl::PointCloud<PointXYZIRPYT>);

  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  std::string loadFullPath = pcd_path + loadName;
  std::string saveFullPath = pcd_path + saveName;

  LOG(INFO) << "loadFullPath:" << loadFullPath;
  LOG(INFO) << "saveFullPath:" << saveFullPath;

  reader.read(loadFullPath, *read_cloud);
  LOG(INFO) << "read_cloud->size():" << read_cloud->size();

  //   pcl::transformPointCloud(*read_cloud, *write_cloud, transMat.inverse());

  for (auto p : read_cloud->points) {
    Eigen::Affine3f point_T =
        pcl::getTransformation(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
    point_T = transMat.inverse() * point_T;

    PointXYZIRPYT pose;
    pose.intensity = p.intensity;
    pose.time = p.time;
    pcl::getTranslationAndEulerAngles(point_T, pose.x, pose.y, pose.z,
                                      pose.roll, pose.pitch, pose.yaw);
    write_cloud->push_back(pose);
  }

  //   writer.write(saveFullPath, *write_cloud, true);
  writer.writeASCII(saveFullPath, *write_cloud);
  LOG(INFO) << "Save Success";
}
