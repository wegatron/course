#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char * argv[])
{
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    q1.normalize();
    //std::cout << q1.coeffs() << std::endl;
    Eigen::Vector3d t1(0.7, 1.1, 0.2);
    Eigen::Matrix4d rt1_2world; rt1_2world.setIdentity();
    rt1_2world.block<3,3>(0,0) = q1.matrix();
    rt1_2world.block<3,1>(0, 3) = t1;

    //std::cout << rt1_2world << std::endl;

    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);
    Eigen::Matrix4d rt2_2world; rt2_2world.setIdentity();
    rt2_2world.block<3,3>(0,0) = q2.matrix();
    rt2_2world.block<3,1>(0, 3) = t2;

    Eigen::Vector4d p1(0.5, -0.1, 0.2, 1.0);
    Eigen::Vector4d p2 = rt2_2world.inverse() * rt1_2world * p1;
    // 答案的算法
    // Eigen::Vector4d p2 = rt2_2world * rt1_2world.inverse() * p1;
    std::cout << p2.block<3,1>(0,0).transpose() << std::endl;
    return 0;
}
