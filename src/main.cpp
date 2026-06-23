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

#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <string>

void printUsage() {
  std::cout << "Usage: rt_map [options]\n";
  std::cout << "Options:\n";
  std::cout << "  -i <input_file>    Input PCD file path (required)\n";
  std::cout << "  -o <output_file>   Output PCD file path (required)\n";
  std::cout << "  -d <leaf_size>     Enable voxel grid downsampling with leaf "
               "size (default: 0.2, 0.2, 0.2)\n";
  std::cout << "                     If -d is not specified, no downsampling "
               "is performed\n";
  std::cout << "  -tx <x>            Translation in X direction (meters, "
               "default: 0.0)\n";
  std::cout << "  -ty <y>            Translation in Y direction (meters, "
               "default: 0.0)\n";
  std::cout << "  -tz <z>            Translation in Z direction (meters, "
               "default: 0.0)\n";
  std::cout << "  -rx <roll>         Rotation around X axis (degrees, default: "
               "0.0)\n";
  std::cout << "  -ry <pitch>        Rotation around Y axis (degrees, default: "
               "0.0)\n";
  std::cout << "  -rz <yaw>          Rotation around Z axis (degrees, default: "
               "0.0)\n";
  std::cout << "  -h                 Print this help message\n";
  std::cout << "\nExample:\n";
  std::cout << "  rt_map -i input.pcd -o output.pcd -d 0.2 -tx 0 -ty -90 -tz 0 "
               "-rx 0 -ry 0 -rz 90\n";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;

  std::string input_file;
  std::string output_file;
  bool enable_downsample = false;
  float leaf_size = 0.2f;
  float tx = 0.0f;
  float ty = 0.0f;
  float tz = 0.0f;
  float rx_deg = 0.0f;
  float ry_deg = 0.0f;
  float rz_deg = 0.0f;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-i" && i + 1 < argc) {
      input_file = argv[++i];
    } else if (arg == "-o" && i + 1 < argc) {
      output_file = argv[++i];
    } else if (arg == "-d" && i + 1 < argc) {
      enable_downsample = true;
      leaf_size = std::atof(argv[++i]);
    } else if (arg == "-tx" && i + 1 < argc) {
      tx = std::atof(argv[++i]);
    } else if (arg == "-ty" && i + 1 < argc) {
      ty = std::atof(argv[++i]);
    } else if (arg == "-tz" && i + 1 < argc) {
      tz = std::atof(argv[++i]);
    } else if (arg == "-rx" && i + 1 < argc) {
      rx_deg = std::atof(argv[++i]);
    } else if (arg == "-ry" && i + 1 < argc) {
      ry_deg = std::atof(argv[++i]);
    } else if (arg == "-rz" && i + 1 < argc) {
      rz_deg = std::atof(argv[++i]);
    } else if (arg == "-h") {
      printUsage();
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      printUsage();
      return 1;
    }
  }

  if (input_file.empty() || output_file.empty()) {
    std::cerr << "Error: Input and output files are required" << std::endl;
    printUsage();
    return 1;
  }

  LOG(INFO) << "Input file: " << input_file;
  LOG(INFO) << "Output file: " << output_file;
  LOG(INFO) << "Downsampling enabled: "
            << (enable_downsample ? "true" : "false");
  if (enable_downsample) {
    LOG(INFO) << "Leaf size: " << leaf_size;
  }
  LOG(INFO) << "Translation: tx=" << tx << ", ty=" << ty << ", tz=" << tz;
  LOG(INFO) << "Rotation (degrees): rx=" << rx_deg << ", ry=" << ry_deg
            << ", rz=" << rz_deg;

  float rx_rad = rx_deg * M_PI / 180.0f;
  float ry_rad = ry_deg * M_PI / 180.0f;
  float rz_rad = rz_deg * M_PI / 180.0f;

  Eigen::Affine3f transMat =
      pcl::getTransformation(tx, ty, tz, rx_rad, ry_rad, rz_rad);

  pcl::PointCloud<pcl::PointXYZI>::Ptr read_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr write_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  reader.read(input_file, *read_cloud);
  LOG(INFO) << "Loaded " << read_cloud->size() << " points";

  if (enable_downsample) {
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
    LOG(INFO) << "Downsampling with leaf size: "
              << downSizeFilter.getLeafSize();
    downSizeFilter.setInputCloud(read_cloud);
    downSizeFilter.filter(*filtered_cloud);
    LOG(INFO) << "After downsampling: " << filtered_cloud->size() << " points";
  } else {
    *filtered_cloud = *read_cloud;
  }

  pcl::transformPointCloud(*filtered_cloud, *write_cloud, transMat);
  write_cloud->width = write_cloud->points.size();
  write_cloud->height = 1;

  writer.write(output_file, *write_cloud, true);
  LOG(INFO) << "Transformation completed, saved " << write_cloud->size()
            << " points to " << output_file;
  LOG(INFO) << "Done";

  return 0;
}