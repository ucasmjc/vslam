#ifndef FILEPROCESS_H_
#define FILEPROCESS_H_

#include <fstream>
#include <ctime> 
#include <algorithm>
#include <iostream>
#include <ctime>
#include <cmath>
#include <string>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
int visualpcd(string path);
void readDataFromIMUOdometry(string path, int fileNum, double pose[][7]);
void drawOdometryTrajectory(string path, double pose[][7], int pointNum);
void calCloudFromBEV(string filename, string savePath);
void readVelocityData(string path, int fileNum, double velocity[][4]);
void readIMUData(string path, int fileNum, double imu[][9]);
void readLidarOdodetryData(string path, double lidar_odom[][7]);
void readData(string path, double lidar_odom[][12]);
#endif