//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix<double, 2, 3> K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    std::ifstream ifs_2d(p2d_file);
    double tmp[3];
    while(!ifs_2d.eof())
    {
        ifs_2d>>tmp[0]>>tmp[1];
        p2d.emplace_back(tmp[0], tmp[1]);
    }

    std::ifstream ifs_3d(p3d_file);
    while(!ifs_3d.eof())
    {
        ifs_3d>>tmp[0]>>tmp[1]>>tmp[2];
        p3d.emplace_back(tmp[0], tmp[1], tmp[2]);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Eigen::Vector3d tp = T_esti * p3d[i];
            Eigen::Vector2d e = p2d[i] - 1.0/tp.z()*K*tp;
            cost += 0.5*e.squaredNorm();
	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            Eigen::Matrix<double, 3, 6> pp;
            pp.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
            pp.block<3,3>(0,3) = -Sophus::SO3d::hat(tp);
            Eigen::Matrix<double, 2, 3> ep;
            ep << fx/tp.z(), 0, -fx*tp.x()/(tp.z()*tp.z()),
               0, fy/tp.z(), -fy*tp.y()/(tp.z()*tp.z());
            J = -ep*pp;
	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx
        Vector6d dx;

        // START YOUR CODE HERE 
        Eigen::LDLT<Eigen::Matrix<double, 6,6>> ldlt(H);
        dx = ldlt.solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3d::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
