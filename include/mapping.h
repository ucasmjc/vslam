#ifndef MAPPING_H_
#define MAPPING_H_

#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include "yaml-cpp/yaml.h"

#include "fileProcess.h"

void mapping(double pose[][7]);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr myTransformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, Eigen::Affine3d& transCur);

#endif