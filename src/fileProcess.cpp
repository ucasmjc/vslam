#include "fileProcess.h"



int visualpcd(string path)
{
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);

    // Create PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");

    // Add point cloud to viewer
    viewer.addPointCloud(cloud, "cloud");

    // Spin viewer
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(0.001);
		sleep(0.1);
    }

    return 0;
}
using namespace std;

void readDataFromIMUOdometry(string path, int fileNum, double pose[][7]){
    for(int i=0;i<fileNum;i++){
        ifstream infile;
        stringstream buf;
        buf << setfill('0') << setw(6) << i;
        string strfileNum;
        strfileNum = buf.str();
        string filePath = path + strfileNum + ".txt";
        infile.open(filePath, ios::in);
        if(!infile){
            cout << "file does not exist!!" << endl;
			cout << filePath << endl;
            exit(1);
        }
        char line[1000];
		int j=0;
		while (infile.getline(line,100,' '))
		{	
			string num = line;
			double a;
			stringstream ss;
			ss << num;
			ss >> a;
			pose[i][j] = a;
			j++;
		}
	   infile.close();
    }
}

void drawOdometryTrajectory(string outputPath, double pose[][7], int pointNum){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(int i=0;i<pointNum;i++){
		pcl::PointXYZRGB po;
		po.x = pose[i][0];
		po.y = pose[i][1];
		po.z = pose[i][2];
		po.r=255;
		po.g=0;
		po.b=0;
		trajectory->push_back(po);
	}
	pcl::io::savePCDFileBinary(outputPath, *trajectory);
    cout << "Trajectory has been drawn in "<< outputPath << endl;
}

void calCloudFromBEV(string filename, string savePath){
    cv::Mat bevImage = cv::imread(filename);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bevCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    int row=bevImage.rows;
    int col=bevImage.cols;
    int pink_flag = 0;
    int red_flag = 0;
    for(int i=0; i<row; i++){

        const uchar* p=bevImage.ptr<uchar>(i);
        for(int j=0;j<col;j++){
            int b=p[3*j];
            int g=p[3*j+1];
            int r=p[3*j+2];
            

            // if (((r>160 && r<=200) && (g>=120 && g<170) && (b>=120 && b<170)) ||
            // ((r>=90 && r<120) && (g>80 && g<=120) && (b<170 &&b>=140))){
            //     pcl::PointXYZRGB po;
			// 	// 原点(1968.5,1696)
            //     po.x = double(i)-double(1968.5);;
            //     po.y = double(j)-double(1696);
            //     po.z = double(0);
            //     po.r=r;
            //     po.g=g;
            //     po.b=b;
            //     bevCloud->push_back(po);
            // }
            
            if ((r>=200 && r<255) && g<=50 && b>=200){
                // 粉色
                // if(pink_flag<5){
                //     pink_flag++;
                // }else{
                    pcl::PointXYZRGB po;
				    // 原点(1968.5,1696)
                    // po.y = double(i)-double(1968.5);
                    // po.x = double(j)-double(1696);
                    po.x = -(double(i)-double(1961.5));
                    po.y = -(double(j)-double(1696));
                    po.z = double(0);
                    po.r=r;
                    po.g=g;
                    po.b=b;
                    bevCloud->push_back(po);
                    pink_flag=0;
                // }    
            }

            if (((r>30 && r<=50) && (g>=0 && g<20) && (b>=100 && b<130)) //深蓝色
            || ((r>=150 && r<170) && (g>90 && g<=120) && (b<120 &&b>=90)) // 棕色
            || ((r>=140 && r<170) && (g>140 && g<=170) && (b<200 &&b>=180)) // 灰色
            ){
                pcl::PointXYZRGB po;
				// 原点(1968.5,1696)
                // po.y = double(i)-double(1968.5);
                // po.x = double(j)-double(1696);
                po.x = -(double(i)-double(1961.5));
                po.y = -(double(j)-double(1696));
                po.z = double(0);
                po.r=r;
                po.g=g;
                po.b=b;
                bevCloud->push_back(po);
            }
            // if ((r>=100 && r<150) && g<=50 && b<=50){
            //     // if(red_flag<20){
            //     //     red_flag++;
            //     // }else{
            //         pcl::PointXYZRGB po;
			// 	    // 原点(1968.5,1696)
            //         po.x = double(i)-double(1968.5);
            //         po.y = double(j)-double(1696);
            //         po.z = double(0);
            //         po.r=r;
            //         po.g=g;
            //         po.b=b;
            //         bevCloud->push_back(po);
            //         red_flag=0;
            //     // }    
            // }
        }//for(int j=0;j<col;j++){
    }//for(int i=0; i<row; i++){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredBevCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::RandomSample<pcl::PointXYZRGB> sor;
    sor.setInputCloud(bevCloud);
    sor.setSample(3000);
    sor.filter(*filteredBevCloud);

    // 保存点云
    filteredBevCloud->height = 1;
    filteredBevCloud->width = filteredBevCloud->points.size();
    cout << "point cloud size = " << filteredBevCloud->points.size() << endl;
    filteredBevCloud->is_dense = false;
    pcl::io::savePCDFile(savePath, *filteredBevCloud);
    bevCloud->points.clear();
    filteredBevCloud->points.clear();
    cout << "Point cloud "+ savePath + " saved" << endl;
}

void readVelocityData(string path, int fileNum, double velocity[][4]){
    for(int i=0;i<fileNum;i++){
        ifstream infile;
        stringstream buf;
        buf << setfill('0') << setw(6) << i;
        string strfileNum;
        strfileNum = buf.str();
        string filePath = path + strfileNum + ".txt";
        infile.open(filePath, ios::in);
        if(!infile){
            cout << "file does not exist!!" << endl;
			cout << filePath << endl;
            exit(1);
        }
        char line[1000];
		int j=0;
		while (infile.getline(line,100,' '))
		{	
			string num = line;
			double a;
			stringstream ss;
			ss << num;
			ss >> a;
			velocity[i][j] = a;
			j++;
		}
	   infile.close();
    }
}

void readIMUData(string path, int fileNum, double imu[][9]){
    for(int i=0;i<fileNum;i++){
        ifstream infile;
        stringstream buf;
        buf << setfill('0') << setw(6) << i;
        string strfileNum;
        strfileNum = buf.str();
        string filePath = path + strfileNum + ".txt";
        infile.open(filePath, ios::in);
        if(!infile){
            cout << "file does not exist!!" << endl;
			cout << filePath << endl;
            exit(1);
        }
        char line[1000];
		int j=0;
		while (infile.getline(line,100,' '))
		{	
			string num = line;
			double a;
			stringstream ss;
			ss << num;
			ss >> a;
			imu[i][j] = a;
			j++;
		}
	   infile.close();
    }
}
void readData(string path, double lidar_odom[][6]){
    ifstream infile;
    vector<std::vector<double>> data;
    infile.open(path, ios::in);
    if(!infile){
        cout << "file does not exist!!" << endl;
        cout << path << endl;
        exit(1);
    }
    if (infile.is_open()) {
        std::string line;
        while (std::getline(infile, line)) {
            std::vector<double> row;
            std::stringstream ss(line);
            double num;
            while (ss >> num) {
                row.push_back(num);
            }
            data.push_back(row);
        }
    }
    infile.close();
    const int rows = data.size();
    const int cols = data[0].size();
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++){
            lidar_odom[i][j] = data[i][j];
        }
    }

//     const int rows = data.size();
//     const int cols = data[0].size();
//     double** dataArray = new double*[rows];
//     for (int i = 0; i < rows; i++) {
//         dataArray[i] = new double[cols];
//         for (int j = 0; j < cols; j++) {
//             dataArray[i][j] = data[i][j];
//         }
//     }
//     for (const auto& row : data) {
//         for (const auto& num : row) {
//             cout << setprecision(6) << fixed;
//             std::cout << num << " ";
//         }
//         std::cout << std::endl;
//     }

//     for (int i = 0; i < rows; i++) {
//         for (int j = 0; j < cols; j++) {
//             std::cout << dataArray[i][j] << " ";
//         }
//         std::cout << std::endl;
//     }

//     // 释放动态分配的内存
//     for (int i = 0; i < rows; i++) {
//         delete[] dataArray[i];
//     }
//     delete[] dataArray;

}
void readLidarOdodetryData(string path, double lidar_odom[][7]){
    ifstream infile;
    vector<std::vector<double>> data;
    infile.open(path, ios::in);
    if(!infile){
        cout << "file does not exist!!" << endl;
        cout << path << endl;
        exit(1);
    }
    if (infile.is_open()) {
        std::string line;
        while (std::getline(infile, line)) {
            std::vector<double> row;
            std::stringstream ss(line);
            double num;
            while (ss >> num) {
                row.push_back(num);
            }
            data.push_back(row);
        }
    }
    infile.close();
    const int rows = data.size();
    const int cols = data[0].size();
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++){
            lidar_odom[i][j] = data[i][j];
        }
    }

//     const int rows = data.size();
//     const int cols = data[0].size();
//     double** dataArray = new double*[rows];
//     for (int i = 0; i < rows; i++) {
//         dataArray[i] = new double[cols];
//         for (int j = 0; j < cols; j++) {
//             dataArray[i][j] = data[i][j];
//         }
//     }
//     for (const auto& row : data) {
//         for (const auto& num : row) {
//             cout << setprecision(6) << fixed;
//             std::cout << num << " ";
//         }
//         std::cout << std::endl;
//     }

//     for (int i = 0; i < rows; i++) {
//         for (int j = 0; j < cols; j++) {
//             std::cout << dataArray[i][j] << " ";
//         }
//         std::cout << std::endl;
//     }

//     // 释放动态分配的内存
//     for (int i = 0; i < rows; i++) {
//         delete[] dataArray[i];
//     }
//     delete[] dataArray;

}
