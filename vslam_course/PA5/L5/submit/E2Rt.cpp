//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double sv = (svd.singularValues()[0] + svd.singularValues()[1])*0.5;
    Eigen::DiagonalMatrix<double, 3> sigma;
    sigma.diagonal() << sv, sv, 0;
    //std::cout << svd.singularValues() << std::endl;
    //std::cout << "matrixU:\n" << svd.matrixU() << std::endl;
    //std::cout << "matrixV:\n" << svd.matrixV() << std::endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Eigen::Matrix3d W;
    W << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    Eigen::Matrix3d Wt = W.transpose();

    Matrix3d t_wedge1 = svd.matrixU()*W*sigma*svd.matrixU().transpose();
    Matrix3d t_wedge2 = svd.matrixU()*Wt*sigma*svd.matrixU().transpose();

    Matrix3d R1 = svd.matrixU() * W * svd.matrixV().transpose();
    Matrix3d R2 = svd.matrixU() * Wt * svd.matrixV().transpose();
    if(R1.determinant() < 0) R1 = -R1;
    if(R2.determinant() < 0) R2 = -R2;
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}