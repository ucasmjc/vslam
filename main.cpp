#include "mapping.h"
#include "fileProcess.h"
#include "calcOdometry.h"
#include "yaml-cpp/yaml.h"
#include "loopclosing.h"
#include<vector>

using namespace std;

int main(int argc, char** argv){
	string lidar_odom_path = "/home/ucasmjc/Desktop/myslam/lidar_odom.txt";
	int get_pointCloud_flag=0;
	int mapping_flag=1;
	int num=1718;
	double lidar_odom[num][7];
	readLidarOdodetryData(lidar_odom_path,lidar_odom);
	if(get_pointCloud_flag){
		string rootPath = "/home/ucasmjc/Desktop/myslam/avm_seg/";
		string savePath = "/home/ucasmjc/Desktop/myslam/cloud/";
		for(int i=0;i<num;i++){
			stringstream buf;
        	buf << setfill('0') << setw(6) << i;
        	string strfileNum;
        	strfileNum = buf.str();
        	// string filePath = rootPath + to_string(i) + ".jpg";
        	string filePath = rootPath + strfileNum + ".png";
        	// string pcdSavedPath = savePath + to_string(i) + ".pcd";
        	string pcdSavedPath = savePath + strfileNum + ".pcd";
        	calCloudFromBEV(filePath,pcdSavedPath);
   		}
	}
	if(mapping_flag) mapping(lidar_odom);
	//correctloop(0£¬1551£¬140);
	return 0;
}