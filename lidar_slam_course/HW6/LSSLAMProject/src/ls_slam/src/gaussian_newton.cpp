#include "gaussian_newton.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <iostream>

namespace pa6_gn {

//位姿-->转换矩阵
    Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x) {
        Eigen::Matrix3d trans;
        trans << cos(x(2)), -sin(x(2)), x(0),
                sin(x(2)), cos(x(2)), x(1),
                0, 0, 1;

        return trans;
    }

    Eigen::Matrix2d angle2rmat(double yaw) {
        Eigen::Matrix2d rmat;
        rmat << cos(yaw), -sin(yaw),
                sin(yaw), cos(yaw);

        return rmat;
    }


//转换矩阵－－＞位姿
    Eigen::Vector3d TransToPose(Eigen::Matrix3d trans) {
        Eigen::Vector3d pose;
        pose(0) = trans(0, 2);
        pose(1) = trans(1, 2);
        pose(2) = atan2(trans(1, 0), trans(0, 0));

        return pose;
    }

//计算整个pose-graph的误差
    double ComputeError(std::vector<Eigen::Vector3d> &Vertexs,
                        std::vector<Edge> &Edges) {
        double sumError = 0;
        for (int i = 0; i < Edges.size(); i++) {
            Edge tmpEdge = Edges[i];
            Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
            Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
            Eigen::Vector3d z = tmpEdge.measurement;
            Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

            Eigen::Matrix3d Xi = PoseToTrans(xi);
            Eigen::Matrix3d Xj = PoseToTrans(xj);
            Eigen::Matrix3d Z = PoseToTrans(z);

            Eigen::Matrix3d Ei = Z.inverse() * Xi.inverse() * Xj;

            Eigen::Vector3d ei = TransToPose(Ei);
#if 0
            Eigen::Matrix2d Ri;
            Ri << cos(xi(2)),-sin(xi(2)),
                  sin(xi(2)), cos(xi(2));
            Eigen::Vector3d ei_me;
            ei_me.block<2,1>(0,0)
                    = Z.block<2,2>(0,0).transpose() * (Ri.transpose() * (xj.block<2,1>(0,0) - xi.block<2,1>(0,0))
                    - z.block<2,1>(0,0));
                    ei_me[2] = xj[2]-xi[2]-z[2];
            if(ei_me[2] > M_PI) ei_me[2] -= 2*M_PI;
            else if(ei_me[2] < -M_PI) ei_me[2] += 2*M_PI;
    //        std::cout << "--------------" << std::endl;
    //        std::cout << "invers Z:\n" << Z.inverse() << std::endl;
    //        std::cout << "inverse Xi:\n" << Xi.inverse() << std::endl;
    //        std::cout << "Xj:\n" << Xj << std::endl;
            std::cout << "diff ei norm:" << (ei - ei_me).squaredNorm() << std::endl;
    //        std::cout << "diff ei:" << (ei-ei_me).transpose() << std::endl;
    //        std::cout << "ei:" << ei.transpose() << std::endl;
    //        std::cout << "ei_me:" << ei_me.transpose() << std::endl;
#endif
            sumError += ei.transpose() * infoMatrix * ei;
        }
        return sumError;
    }


/**
 * @brief CalcJacobianAndError
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bi    相对于xj的Jacobian矩阵
 */
    void CalcJacobianAndError(Eigen::Vector3d xi, Eigen::Vector3d xj, Eigen::Vector3d z,
                              Eigen::Vector3d &ei, Eigen::Matrix3d &Ai, Eigen::Matrix3d &Bi) {
        //TODO--Start
        const Eigen::Matrix2d Ri = angle2rmat(xi[2]);
        const Eigen::Matrix2d Rj = angle2rmat(xj[2]);
        const Eigen::Matrix2d Rz = angle2rmat(z[2]);
        const Eigen::Vector2d ti = xi.block<2, 1>(0, 0);
        const Eigen::Vector2d tj = xj.block<2, 1>(0, 0);
        const Eigen::Vector2d tz = z.block<2, 1>(0, 0);

        ei[2] = xj[2] - xi[2] - z[2];
        if (ei[2] > M_PI) ei[2] -= 2 * M_PI;
        else if (ei[2] < -M_PI) ei[2] += 2 * M_PI;

        ei.block<2, 1>(0, 0) =
                Rz.transpose() * (Ri.transpose() * (tj - ti) - tz);
        Eigen::Matrix2d rmat_df_theta;
        rmat_df_theta << -sin(xi[2]), -cos(xi[2]),
                cos(xi[2]), -sin(xi[2]);
        Ai.block<2, 2>(0, 0) = -Rz.transpose() * Ri.transpose();
        Ai.block<2, 1>(0, 2) = Rz.transpose() * rmat_df_theta.transpose() * (tj - ti);
        Ai(2, 0) = 0;
        Ai(2, 1) = 0;
        Ai(2, 2) = -1;

        Bi.setZero();
        Bi(2, 2) = 1;
        Bi.block<2, 2>(0, 0) = Rz.transpose() * Ri.transpose();
        //TODO--end
    }

/**
 * @brief LinearizeAndSolve
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
    Eigen::VectorXd LinearizeAndSolve(std::vector<Eigen::Vector3d> &Vertexs,
                                      std::vector<Edge> &Edges) {
        //申请内存
        Eigen::MatrixXd H(Vertexs.size() * 3, Vertexs.size() * 3);
        Eigen::VectorXd b(Vertexs.size() * 3);

        H.setZero();
        b.setZero();

        //固定第一帧
        Eigen::Matrix3d I;
        I.setIdentity();
        H.block(0, 0, 3, 3) += I;

        //构造H矩阵　＆ b向量
        for (int i = 0; i < Edges.size(); i++) {
            //提取信息
            Edge tmpEdge = Edges[i];
            Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
            Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
            Eigen::Vector3d z = tmpEdge.measurement;
            Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

            //计算误差和对应的Jacobian
            Eigen::Vector3d ei;
            Eigen::Matrix3d Ai;
            Eigen::Matrix3d Bi;
            CalcJacobianAndError(xi, xj, z, ei, Ai, Bi);
            //TODO--Start
            Eigen::Matrix3d tmp = Ai.transpose() * infoMatrix * Bi;
            H.block<3, 3>(tmpEdge.xi * 3, tmpEdge.xj * 3) += tmp;
            H.block<3, 3>(tmpEdge.xj * 3, tmpEdge.xi * 3) += tmp.transpose();
            H.block<3, 3>(tmpEdge.xi * 3, tmpEdge.xi * 3) += Ai.transpose() * infoMatrix * Ai;
            H.block<3, 3>(tmpEdge.xj * 3, tmpEdge.xj * 3) += Bi.transpose() * infoMatrix * Bi;

            b.block<3, 1>(tmpEdge.xi * 3, 0) -= Ai.transpose() * infoMatrix * ei;
            b.block<3, 1>(tmpEdge.xj * 3, 0) -= Bi.transpose() * infoMatrix * ei;
            //TODO--End
        }

        //求解
        //TODO--Start
        //Eigen::LDLT<decltype(H)> solver(H);
        //Eigen::FullPivLU<decltype(H)> solver(H);
        Eigen::HouseholderQR<decltype(H)> solver(H);
        // if(solver.info() != Eigen::Success) {
        //     std::cerr << "[ERROR] error in ldlt decomposition of H!!" << std::endl;
        //     throw std::logic_error("ldlt decomposition");
        // }
        Eigen::VectorXd dx = solver.solve(b);
        //TODO-End
        return dx;
    }
}











