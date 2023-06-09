#include <algorithm>
#include <fstream>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include "yaml-cpp/yaml.h"
#include "fileProcess.h"
#include "mapping.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include "sophus/geometry.hpp"
#include "sophus/se3.hpp"
void processing(double pose[][7]);
void correctloop(int cusid,int loopid,int con);
