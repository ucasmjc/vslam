#include "fileProcess.h"
#include "loopclosing.h"
using namespace std;
struct keyframe{
    int id;
	Sophus::SE3d uncorrect;
	Sophus::SE3d correct;
};
//位姿图优化
void OptimizeEssentialGraph(
    keyframe *vpkfs, int cusid,int loopid,int con,int num)
{
    // Setup optimizer
    // Step 1：构造优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    // 指定线性方程求解器使用Eigen的块求解器
    // 7表示位姿是sim3  3表示三维点坐标维度
    std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>());
    // 构造线性求解器
    std::unique_ptr<g2o::BlockSolver_7_3>  solver_ptr (new g2o::BlockSolver_7_3( std::move(linearSolver) ));
    // 使用LM算法进行非线性迭代
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    // 第一次迭代的初始lambda值，如未指定会自动计算一个合适的值
    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);


    // 记录所有优化前关键帧的位姿，优先使用在闭环时通过Sim3传播调整过的Sim3位姿
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(num + 1); // 存放每一帧优化前的sim3
   
    // Set KeyFrame vertices
    // Step 2：将地图中所有关键帧的pose作为顶点添加到优化器
    // 遍历全局地图中的所有的关键帧
    for (int i = 0; i < num; i++)
    {
        keyframe pKF = vpkfs[i];
        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();


        if (pKF.correct.matrix() == Eigen::Matrix4d::Identity())
        {
            Sophus::SE3d Tiw = pKF.uncorrect.cast<double>();
            g2o::Sim3 Siw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            vScw[i] = Siw;
            VSim3->setEstimate(Siw);
        }
        else
        {
            Sophus::SE3d Tiw = pKF.correct.cast<double>();
			Sophus::SE3d Twi = Tiw.inverse();
			Sophus::SE3d uncorrectTiw = pKF.uncorrect.cast<double>();
            g2o::Sim3 Siw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
			g2o::Sim3 uncorrectSiw(uncorrectTiw.unit_quaternion(), uncorrectTiw.translation(), 1.0);
            vScw[i] = uncorrectSiw;
            VSim3->setEstimate(Siw);
        }
        // 固定
        if (pKF.id==cusid||pKF.id==loopid)
            VSim3->setFixed(true);

        VSim3->setId(i);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = false;// 和当前系统的传感器有关，如果是RGBD或者是双目为true

        optimizer.addVertex(VSim3);
    }
	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();
	const g2o::Sim3 Scw = vScw[num-1];
    const g2o::Sim3 Swc = Scw.inverse();
	//第一种边。当前帧和其共识帧之间
	for(int i=con;i<num-1;i++){
		keyframe pkf=vpkfs[i];
		const g2o::Sim3 Siw = vScw[i];
        const g2o::Sim3 Sic = Siw*Swc;  
		g2o::EdgeSim3 *e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(i)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(num-1)));
        e->setMeasurement(Sic);

        // 信息矩阵是单位阵,说明这类新增加的边对总误差的贡献也都是一样大的
        e->information() = matLambda;

        optimizer.addEdge(e);
	}
	//第二种边，每一个关键帧与上一帧
	for(int i = 0; i < num-1; i++){
		keyframe pkfi=vpkfs[i];
		keyframe pkfj=vpkfs[i+1];
		const g2o::Sim3 Siw = vScw[i]; 
		const g2o::Sim3 Sjw = vScw[i+1];
		const g2o::Sim3 Swi = Swi.inverse();
        const g2o::Sim3 Sji = Sjw*Swi;  
		g2o::EdgeSim3 *e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(i+1)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(i)));
        e->setMeasurement(Sji);
        e->information() = matLambda;

        optimizer.addEdge(e);
	}
	//第三种边，当前帧与回环帧之间
	keyframe pkfc=vpkfs[num-1];
	Sophus::SE3d correctTwc = pkfc.correct.cast<double>();
    g2o::Sim3 correctSwc(correctTwc.unit_quaternion(), correctTwc.translation(), 1.0);
	const g2o::Sim3 Slw = vScw[0]; 
    const g2o::Sim3 Slc = Slw*correctSwc;  
	g2o::EdgeSim3 *e = new g2o::EdgeSim3();
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(num-1)));
    e->setMeasurement(Slc);
    e->information() = matLambda;
    optimizer.addEdge(e);
//开始优化
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
//用优化后的位姿赋值
	for (size_t i = 0; i < num; i++)
    {
        keyframe pKFi = vpkfs[i];

        g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(i));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        double s = CorrectedSiw.scale();

        Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(), CorrectedSiw.translation().cast<float>() / s);
        pKFi.correct=Tiw.cast<double>();
    }
}

//cusid为当前关键帧的index(即0--1718中的一个数)，loopid为匹配到的回环帧的index，con为当前帧的共视帧的位次（0-141的一个数）。
//不过，在本函数中，当前帧默认为最后一个关键帧，回环帧默认为第一帧。
void correctloop(int cusid,int loopid,int con){
	YAML::Node config = YAML::LoadFile("../params.yaml");

	string result = config["icp_result"].as<string>();//此处为每一个关键帧的局部点云所在的路径
	int icp_maximum_iterations_mapping = config["icp_maximum_iterations_mapping"].as<int>();
	float voxel_downsample_leaf_size = config["voxel_downsample_leaf_size"].as<float>();
	float icp_max_correspondence_distance = config["icp_max_correspondence_distance"].as<float>();
	double icp_transformation_epsilon = config["icp_transformation_epsilon"].as<double>();
	double icp_euclidean_fitness_epsilon = config["icp_euclidean_fitness_epsilon"].as<double>();
    int num=142;//假设共有142个关键帧
    string KF_cloud = config["icp_result"].as<string>();
    double pose[142][6];
    readData("../test.txt",pose);//这是每个关键帧的位姿，每个位姿6位，前三位是位移，后三位是欧拉角。由process函数生成
	keyframe kfs[num];
	int ids[num];
	ifstream idfile;
    idfile.open("../index.txt", ios::in);//这是关键帧的index(0-1718)
    if (idfile.is_open()) {
        std::string line;
        std::getline(idfile, line);
        std::stringstream ss(line);
        double num1;
		int j=0;
        while (ss >> num1) {
            ids[j]=num1;
			j++;
        }
        }
	for(int i=0;i<num;i++){
		Eigen::Vector3d t(pose[i][0], pose[i][1], pose[i][2]);
		Eigen::AngleAxisd rot_x(pose[i][3], Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd rot_y(pose[i][4], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z(pose[i][5], Eigen::Vector3d::UnitZ());
		Eigen::Matrix3d R = (rot_z * rot_y * rot_x).toRotationMatrix();
		kfs[i].id=ids[i];
		Sophus::SE3d T(R, t);
		kfs[i].uncorrect=T;
	}
	//先将回环帧与当前帧匹配
	string loop_path=result+"local"+to_string(loopid)+".pcd";
	string cus_path=result+"local"+to_string(cusid)+".pcd";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr loop_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cus_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(loop_path, *loop_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(cus_path, *cus_cloud);
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transSrcToGlobal(new pcl::PointCloud<pcl::PointXYZRGB>());
	icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
    icp.setMaximumIterations(icp_maximum_iterations_mapping);
    icp.setTransformationEpsilon(icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
	icp.setInputSource(cus_cloud);
	icp.setInputTarget(loop_cloud);
	icp.align(*transSrcToGlobal);
	Sophus::SE3d Twc;
	Sophus::SE3d loopTwc;
	if (icp.hasConverged() == false) {
		cout << "bad loop!" << endl;
		return;
    }
	else{
		Eigen::Affine3d relative;
		relative = icp.getFinalTransformation().cast<double>();
		Sophus::SE3d se3 = Sophus::SE3d::fitToSE3(relative.matrix());

    	Twc = kfs[-1].uncorrect;
		Sophus::SE3d Tcw = Twc.inverse();
		loopTwc=(se3*Twc).cast<double>();
		kfs[-1].correct=loopTwc;
		cout << "good loop"<< endl;
	}
	//更新当前帧的共识帧
	for(int i=con;i<num-1;i++){
		Sophus::SE3d Twi = kfs[i].uncorrect;
		Sophus::SE3d Tiw = Twi.inverse();
		Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
		kfs[i].correct=Tic*loopTwc;
	}
	double currentX=0;
    double currentY=0;
    double currentZ=0;
    double currentRoll=0;
    double currentPitch=0;
    double currentYaw=0;
	//位姿图优化
	OptimizeEssentialGraph(kfs, cusid,loopid,con,num);
	ofstream outfile1("../result.txt");
    for(int m=0;m<num;m++){
		Eigen::Affine3d affine = Eigen::Affine3d(kfs[m].correct.matrix());
        pcl::getTranslationAndEulerAngles(affine,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
        outfile1<<currentX<<" "<<currentY<<" "<<currentZ<<" "<<currentRoll<<" "<<currentPitch<<" "<<currentYaw<< "\r\n";
    }
    outfile1.close();

}


void processing(double pose[][7]){
    double lastPoseX=0;
    double lastPoseY=0;
    double lastPoseZ=0;
    double lastPoseRoll=0;
    double lastPosePitch=0;
    double lastPoseYaw=0;

    double currentX=0;
    double currentY=0;
    double currentZ=0;
    double currentRoll=0;
    double currentPitch=0;
    double currentYaw=0;
    int keyframe;
	YAML::Node config = YAML::LoadFile("../params.yaml");
	
	std::vector<double> imu2lidar_matrix = config["imu2lidar_matrix"].as<vector<double>>();
	string root_source_cloud = config["root_source_cloud"].as<string>();
	string init_global_cloud = config["init_global_cloud"].as<string>();
	string transSrcCloud_path = config["transSrcCloud_path"].as<string>();
	string result = config["icp_result"].as<string>();

	int pointcloud_file_num = config["pointcloud_file_num"].as<int>();
	int random_downsample_num = config["random_downsample_num"].as<int>();
	int icp_maximum_iterations_mapping = config["icp_maximum_iterations_mapping"].as<int>();
	float voxel_downsample_leaf_size = config["voxel_downsample_leaf_size"].as<float>();
	float icp_max_correspondence_distance = config["icp_max_correspondence_distance"].as<float>();
	double icp_transformation_epsilon = config["icp_transformation_epsilon"].as<double>();
	double icp_euclidean_fitness_epsilon = config["icp_euclidean_fitness_epsilon"].as<double>();
	double key_frame_dis = config["key_frame_dis"].as<double>();
	double quaterniondNum[pointcloud_file_num][4];
	double translation3dNum[pointcloud_file_num][3];
	
	for(int i=0;i<pointcloud_file_num;i++){
		translation3dNum[i][0] = pose[i][0];
		translation3dNum[i][1] = pose[i][1];
		translation3dNum[i][2] = pose[i][2];	
		
		for(int k=0;k<=3;k++){
			quaterniondNum[i][k] = pose[i][k+3];
		}
	}

	Eigen::Affine3d imu2lidar;
	imu2lidar(0,0) = imu2lidar_matrix[0];
	imu2lidar(0,1) = imu2lidar_matrix[1];
	imu2lidar(0,2) = imu2lidar_matrix[2];
	imu2lidar(1,0) = imu2lidar_matrix[3];
	imu2lidar(1,1) = imu2lidar_matrix[4];
	imu2lidar(1,2) = imu2lidar_matrix[5];
	imu2lidar(2,0) = imu2lidar_matrix[6];
	imu2lidar(2,1) = imu2lidar_matrix[7];
	imu2lidar(2,2) = imu2lidar_matrix[8]; 
	imu2lidar(0,3) = imu2lidar_matrix[9];
	imu2lidar(1,3) = imu2lidar_matrix[10];
	imu2lidar(2,3) = imu2lidar_matrix[11];

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpGloCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(init_global_cloud, *global_cloud);

    Eigen::Affine3d outres1[150];
    int index[150];
    int length=0;
	int cloudSize = global_cloud->size();
	tmpGloCloud->resize(cloudSize);

	for (int i = 0; i < cloudSize; ++i)
	{
		const auto &pointFrom = global_cloud->points[i];
		tmpGloCloud->points[i].x = double(pointFrom.x*0.004);//为啥*0.004
		tmpGloCloud->points[i].y = double(pointFrom.y*0.004);
		tmpGloCloud->points[i].z = double(pointFrom.z*0.004);
		tmpGloCloud->points[i].r = pointFrom.r;
		tmpGloCloud->points[i].g = pointFrom.g;
		tmpGloCloud->points[i].b = pointFrom.b;

	}
	*global_cloud = *tmpGloCloud;
    pcl::io::savePCDFileBinary(result + "local" + to_string(0) + ".pcd", *global_cloud);
	Eigen::Quaterniond q_init;
	q_init.x() = quaterniondNum[0][0];
	q_init.y() = quaterniondNum[0][1];
	q_init.z() = quaterniondNum[0][2];
	q_init.w() = quaterniondNum[0][3];
	double t_x_init = translation3dNum[0][0];
	double t_y_init = translation3dNum[0][1];
	double t_z_init = translation3dNum[0][2];

	Eigen::Translation3d t_init(t_x_init,t_y_init,t_z_init);
	Eigen::Affine3d scr2global_init = t_init * q_init.toRotationMatrix()*imu2lidar;//将imu数据映射到全局坐标系
	Eigen::Affine3d inverseMatrix = scr2global_init.inverse();
    outres1[length]=scr2global_init;
    index[length]=0;
    length++;

	clock_t start,end;
	start = clock();

	for(int j=1;j<pointcloud_file_num;j++){
		stringstream buf;
        buf << setfill('0') << setw(6) << j;//填充0成为长为6的字符串
        string strfileNum;
        strfileNum = buf.str();
		// string source_cloud_path = root_source_cloud + to_string(j) + ".pcd";
		string source_cloud_path = root_source_cloud + strfileNum + ".pcd";
		string global_cloud_path = result + "glo" + to_string(j) + ".pcd";
        string local_cloud_path = result + "local" + to_string(j) + ".pcd";
		pcl::io::loadPCDFile<pcl::PointXYZRGB>(source_cloud_path, *source_cloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredSource_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    	pcl::RandomSample<pcl::PointXYZRGB> sor;
    	sor.setInputCloud(source_cloud);
    	sor.setSample(10000);
    	sor.filter(*filteredSource_cloud);
		*source_cloud = *filteredSource_cloud;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpSrcCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloudSize = source_cloud->size();
		
		tmpSrcCloud->resize(cloudSize);
		for (int i = 0; i < cloudSize; ++i)
		{
			const auto &pointFrom = source_cloud->points[i];
			tmpSrcCloud->points[i].x = double(pointFrom.x*0.004);
			tmpSrcCloud->points[i].y = double(pointFrom.y*0.004);
			tmpSrcCloud->points[i].z = double(pointFrom.z*0.004);
			tmpSrcCloud->points[i].r = pointFrom.r;
			tmpSrcCloud->points[i].g = pointFrom.g;
			tmpSrcCloud->points[i].b = pointFrom.b;
		}
		*source_cloud = *tmpSrcCloud;

		Eigen::Quaterniond q;
		q.x() = quaterniondNum[j][0];
		q.y() = quaterniondNum[j][1];
		q.z() = quaterniondNum[j][2];
		q.w() = quaterniondNum[j][3];
		double t_x = translation3dNum[j][0];
		double t_y = translation3dNum[j][1];
		double t_z = translation3dNum[j][2];
		Eigen::Translation3d t(t_x,t_y,t_z);
		Eigen::Affine3d scr2global = t * q.toRotationMatrix();
		// 直接用lidar的数据，试一下不这样转换
		//scr2global = scr2global*imu2lidar;
		// scr2global = inverseMatrix * scr2global;
		*source_cloud =*myTransformPointCloud(source_cloud,scr2global);

		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transSrcToGlobal(new pcl::PointCloud<pcl::PointXYZRGB>());
		icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
    	icp.setMaximumIterations(icp_maximum_iterations_mapping);
    	icp.setTransformationEpsilon(icp_transformation_epsilon);
    	icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
		icp.setInputSource(source_cloud);
		icp.setInputTarget(global_cloud);
		icp.align(*transSrcToGlobal);

		string tmp_transSrcCloud_path = transSrcCloud_path + to_string(j) + ".pcd";
		if (icp.hasConverged() == false) {
			cout << "ICP failed, the score is " << icp.getFitnessScore() << endl;
			cout << "Frame No." << j << " failed to finish ICP" << endl;
			keyframe++;
			if(keyframe>5){
				// 5次icp失败就用一次imu的
				*global_cloud = *global_cloud + *source_cloud;
				pcl::io::savePCDFileBinary(local_cloud_path, *global_cloud);
				
				// pcl::io::savePCDFileBinary(tmp_transSrcCloud_path, *source_cloud);
				
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
				cout << global_cloud_path << ": done (imu)!" << endl;
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
				keyframe=0;
			}
        }//if (icp.hasConverged() == false) {
		else{
			Eigen::Affine3d transWorldCurrent1;
			transWorldCurrent1 = icp.getFinalTransformation().cast<double>();
			Eigen::Affine3d transWorldCurrent=transWorldCurrent1*scr2global;
			pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
			double dis=(currentX-lastPoseX)*(currentX-lastPoseX)+(currentY-lastPoseY)*(currentY-lastPoseY);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<pcl::PointXYZRGB>());
			*currentFeatureCloudInWorld =*myTransformPointCloud(source_cloud, transWorldCurrent1);
			// pcl::io::savePCDFileBinary(tmp_transSrcCloud_path, *currentFeatureCloudInWorld);
			cout << "mapping success:" << j << endl;
			if((dis>key_frame_dis)){
				*global_cloud = *global_cloud + *currentFeatureCloudInWorld;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_downsampling(new pcl::PointCloud<pcl::PointXYZRGB>());
       	    	pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
				downSizeFilter.setInputCloud(global_cloud);
        		downSizeFilter.setLeafSize(voxel_downsample_leaf_size, voxel_downsample_leaf_size, voxel_downsample_leaf_size);
        		downSizeFilter.filter(*global_downsampling);
        		*global_cloud=*global_downsampling;
				pcl::io::savePCDFileBinary(local_cloud_path, *currentFeatureCloudInWorld);
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
				cout << local_cloud_path << ":done!" << endl;
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;   
                outres1[length]=transWorldCurrent;
                index[length]=j;
                length++;
				keyframe=0;
				lastPoseX=currentX;
        		lastPoseY=currentY;
			}//if((dis>odomKeyFramDisThresh)){
		
		}//else

	}//for(int j=0; j<=pointcloud_file_num;j++){
    ofstream outfile("../test.txt");
    pcl::getTranslationAndEulerAngles(outres1[0],currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
    outfile<<currentX<<" "<<currentY<<" "<<currentZ<<" "<<currentRoll<<" "<<currentPitch<<" "<<currentYaw<< "\r\n";
    for(int m=1;m<length;m++){
        pcl::getTranslationAndEulerAngles(outres1[m],currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
        outfile<<currentX<<" "<<currentY<<" "<<currentZ<<" "<<currentRoll<<" "<<currentPitch<<" "<<currentYaw<< "\r\n";
    }
    outfile.close();
    ofstream indexfile("../index.txt");
    for(int m=0;m<length;m++){
        indexfile<<index[m]<<" ";
    }
    indexfile.close();
    pcl::io::savePCDFileBinary(result + "glo" + ".pcd", *global_cloud);
	end = clock();

	cout << "time cost "<< (double)(end - start)/ CLOCKS_PER_SEC << "s"<< endl;	
}//mapping 

