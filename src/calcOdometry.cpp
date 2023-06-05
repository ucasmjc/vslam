#include "calcOdometry.h"

CalcOdometry::~CalcOdometry(){}


std::vector<double> CalcOdometry::EulerToQuaternion(double roll, double pitch, double yaw)
{
    double coeff = static_cast<double>(0.5);
    double r = roll * coeff;
    double p = pitch * coeff;
    double y = yaw * coeff;

    double sr = std::sin(r);
    double sp = std::sin(p);
    double sy = std::sin(y);

    double cr = std::cos(r);
    double cp = std::cos(p);
    double cy = std::cos(y);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    return {qx, qy, qz, qw};
}

std::vector<double> CalcOdometry::QuaternionToEuler(double qw, double qx, double qy, double qz)
{
    double euler_yaw;
    double euler_pitch;
    double euler_roll;

    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    euler_roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        euler_pitch = std::copysign(M_PI / 2, sinp);
    else
        euler_pitch = std::asin(sinp);

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    euler_yaw = std::atan2(siny_cosp, cosy_cosp);

    return {euler_roll, euler_pitch, euler_yaw};
}