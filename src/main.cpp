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
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/impl/point_types.hpp>

const std::string pcd_path = "/home/hzy/code/rt_map/";
const std::string loadName = "GlobalMap.pcd";
const std::string saveName = "GlobalMap_rt.pcd";

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr read_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr write_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  std::string loadFullPath = pcd_path + loadName;
  std::string saveFullPath = pcd_path + saveName;

  LOG(INFO) << "loadFullPath:" << loadFullPath;
  LOG(INFO) << "saveFullPath:" << saveFullPath;

  reader.read(loadFullPath, *read_cloud);
  LOG(INFO) << "read_cloud->size():" << read_cloud->size();

  pcl::transformPointCloud(*read_cloud, *write_cloud, transMat);

  writer.write(saveFullPath, *write_cloud, true);
  LOG(INFO) << "Save Success";
}
