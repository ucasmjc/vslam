#ifndef CALC_ODOMETRY_H
#define CALC_ODOMETRY_H

#include <cmath>
#include "fileProcess.h"
#include "calcOdometry.h"

struct velocities
{
    float linear_x;
    float linear_y;
    float angular_z;
};

class CalcOdometry
{
private:
    const double wheel_radius = 0.32985;
    const double wheel_distance_x = 2.785;
    const double wheel_distance_y = 1.578;
    const int total_wheels = 2;

    double dt = 0.01;
    double wheel_x_ = 0.0;
    double wheel_y_ = 0.0;
    double wheel_th_ = 0.0;

    double imu_x_ = 0.0;
    double imu_y_ = 0.0;
    double imu_vx_ = 0.0;
    double imu_vy_ = 0.0;
    double imu_th_ = 0.0;

    Eigen::Quaterniond q_init;
    Eigen::Translation3d t_init;
    Eigen::Affine3d inverseMatrix;
    bool gnssInitFlag = false;

public:
    CalcOdometry();
    ~CalcOdometry();

    void calcWheelOdometry(double velocityData[4], double wheelOdom[7]);
    void calcImuOdometry(double imuData[9], double imuOdom[7]);

    /// @brief fuse gnss, wheel,lidar odometry by ekf
    Eigen::VectorXd calcFuselOdometry(double gnss[7],
                                      double velocityData[4],
                                      double lidar[7]);

    velocities GetVelocities(float vel_front_left, float vel_front_right, float vel_rear_left, float vel_rear_right);
    std::vector<double> EulerToQuaternion(double roll, double pitch, double yaw);
    std::vector<double> QuaternionToEuler(double qw, double qx, double qy, double qz);
};

#endif //CALC_ODOMETRY_H