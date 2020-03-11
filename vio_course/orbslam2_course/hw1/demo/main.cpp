#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/eigen.hpp>

#include "two_view_geometry.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

static float ld_scale = 1.0;

void createLandmarks(std::vector<Eigen::Vector3d> &points)
{
    float scale = 5;
    const double k = 0.5;

    points.clear();
#if 0
    for (float y = -1.0f; y <= 1.0f; y+=0.2)
    {
        Eigen::Vector3d pt;
        pt[0] = y > 0 ? k*y : -k*y;
        pt[1] = y;
        for (float z = -1.0f; z <= 1.0f; z+=0.2)
        {
            pt[2] = z;
            points.push_back(pt * scale);
        }
    }
#else

    std::mt19937 gen{12345};
    std::normal_distribution<double> d_x{0.0, ld_scale * 4.0};
    std::normal_distribution<double> d_y{0.0, ld_scale * 10.0};
    std::normal_distribution<double> d_z{0.0, ld_scale * 10.0};
    for (int i = 0; i < 200; i ++)
    {
        Eigen::Vector3d pt;
        pt[0] = std::round(d_x(gen));
        pt[1] = std::round(d_y(gen));
        pt[2] = std::round(d_z(gen));
        points.push_back(pt);
    }

#endif
}

void createCameraPose(std::vector<Eigen::Matrix4d> &v_Twc, const Eigen::Vector3d &point_focus)
{
    float x_offset = 20;
    float y_offset = 0;
    float z_offset = -5;
    float scale = 10;
    static const Eigen::Vector3d b_cam_z(0, 0, 1);
    static const Eigen::Matrix3d R_w_base = (Eigen::Matrix3d() << 0, 0, -1, 1, 0, 0, 0, -1, 0).finished();// row major
    // std::cout << (R_w_base * Eigen::Vector3d(0,0,1)).transpose() << std::endl;

    v_Twc.clear();
    for (float angle = 0; angle < 4*360; angle+=15)
    {
        float theta = angle * 3.14159 / 180.0f;
        Eigen::Vector3d pt;
        pt[0] = 0.5 * cos(theta);
        pt[1] = sin(theta);
        pt[2] = theta / 20;

        pt = scale * pt;
        pt[0] += x_offset;
        pt[1] += y_offset;
        pt[2] += z_offset;

        Eigen::Vector3d b_cam_z_cur = R_w_base.transpose() * (point_focus - pt);
        Eigen::Matrix3d R_cur_base(Eigen::Quaterniond::FromTwoVectors(b_cam_z_cur, b_cam_z));
        // std::cout << pt.transpose() << ", " << (R_cur_base * b_cam_z_cur).transpose() << std::endl;
        Eigen::Matrix3d Rwc(R_w_base * R_cur_base.transpose());

        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = Rwc;
        Twc.block(0, 3, 3, 1) = pt;
        v_Twc.push_back(Twc);
    }
}

static float pixel_sigma = 4.0;

void detectFeatures(const Eigen::Matrix4d &Twc, const Eigen::Matrix3d &K,
                    const std::vector<Eigen::Vector3d> &landmarks,
                    std::vector<cv::Point2f> &features,
                    std::vector<bool> &valid,
                    bool add_noise = true)
{
    std::mt19937 gen{12345};

    std::normal_distribution<> d{0.0, pixel_sigma};

    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
    Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
    Eigen::Matrix3d Rcw = Rwc.transpose();
    Eigen::Vector3d tcw = -Rcw * twc;

    valid.assign(landmarks.size(), false);
    for (size_t l = 0; l < landmarks.size(); ++l)
    {
        Eigen::Vector3d wP = landmarks[l];
        Eigen::Vector3d cP = Rcw * wP + tcw;

        if(cP[2] < 0) continue;

        float noise_u = add_noise ? std::round(d(gen)) : 0.0f;
        float noise_v = add_noise ? std::round(d(gen)) : 0.0f;

        Eigen::Vector3d ft = K * cP;
        features[l].x = static_cast<int>(ft[0]/ft[2] + 0.5 + noise_u);
        features[l].y = static_cast<int>(ft[1]/ft[2] + 0.5 + noise_v);
        valid[l] = true;
    }

}

void saveTrajectoryTUM(const std::string &file_name, const std::vector<Eigen::Matrix4d> &v_Twc)
{
    std::ofstream f;
    f.open(file_name.c_str());
    f << std::fixed;

    for(size_t i = 0; i < v_Twc.size(); i++)
    {
        const Eigen::Matrix4d Twc = v_Twc[i];
        const Eigen::Vector3d t = Twc.block(0, 3, 3, 1);
        const Eigen::Matrix3d R = Twc.block(0, 0, 3, 3);
        const Eigen::Quaterniond q = Eigen::Quaterniond(R);

        f << std::setprecision(6) << i << " "
          << std::setprecision(9) << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
    std::cout << "save traj to " << file_name << " done." << std::endl;
}

int main()
{
    cv::viz::Viz3d window("window");
    cv::viz::WCoordinateSystem world_coord(1.0), camera_coord(0.5);
    window.showWidget("Coordinate", world_coord);

    // cv::viz::WLine x_axis(cv::Point3f(0.0f,0.0f,0.0f), cv::Point3f(0.0f,0.0f,2.0f));
    // x_axis.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    // window.showWidget("Line Widget", x_axis);

    //cv::Affine3d cam_pose = cv::viz::makeCameraPose(cv::Vec3f(0, 20, 20), cv::Vec3f(0,0,0), cv::Vec3f(0,1,0));
    //window.setViewerPose(cam_pose);
    //vis.showWidget("World",world_coor);
    window.showWidget("Camera", camera_coord);
    pixel_sigma = 1.0;
    ld_scale = 0.0;
    for(int times=0; times<21; ++times) {
        ld_scale += 0.2;
        std::vector<Eigen::Vector3d> landmarks;
        std::vector<Eigen::Matrix4d> v_Twc;
        createLandmarks(landmarks);
        createCameraPose(v_Twc, Eigen::Vector3d(0,0,0));
        const size_t pose_num = v_Twc.size();

        cv::Mat point_cloud = cv::Mat::zeros(landmarks.size(), 1, CV_32FC3);
        for (size_t i = 0; i < landmarks.size(); i++)
        {
            point_cloud.at<cv::Vec3f>(i)[0] = landmarks[i][0];
            point_cloud.at<cv::Vec3f>(i)[1] = landmarks[i][1];
            point_cloud.at<cv::Vec3f>(i)[2] = landmarks[i][2];
        }
        cv::viz::WCloud cloud(point_cloud);
        window.showWidget("cloud", cloud);

#if 1
        /* show camera gt trajectory */
        for (size_t i = 0; i < pose_num-1; i++)
        {
            Eigen::Vector3d twc0 = v_Twc[i].block(0, 3, 3, 1);
            Eigen::Vector3d twc1 = v_Twc[i+1].block(0, 3, 3, 1);
            cv::Point3d pose_begin(twc0[0], twc0[1], twc0[2]);
            cv::Point3d pose_end(twc1[0], twc1[1], twc1[2]);
            cv::viz::WLine trag_line(pose_begin, pose_end, cv::viz::Color::green());
            window.showWidget("gt_trag_"+std::to_string(i), trag_line);
        }

        static const Eigen::Vector3d cam_z_dir(0, 0, 1);
        for (size_t i = 0; i < pose_num; i++)
        {
            Eigen::Matrix3d Rwc = v_Twc[i].block(0, 0, 3, 3);
            Eigen::Vector3d twc = v_Twc[i].block(0, 3, 3, 1);
            Eigen::Vector3d w_cam_z_dir = Rwc * cam_z_dir;
            cv::Point3d obvs_dir(w_cam_z_dir[0], w_cam_z_dir[1], w_cam_z_dir[2]);
            cv::Point3d obvs_begin(twc[0], twc[1], twc[2]);
            cv::viz::WLine obvs_line(obvs_begin, obvs_begin+obvs_dir, cv::viz::Color::blue());
            window.showWidget("gt_cam_z_"+std::to_string(i), obvs_line);
        }
#endif

        cv::Mat cv_K = (cv::Mat_<double>(3, 3) << 480, 0, 320, 0, 480, 240, 0, 0, 1);
        Eigen::Matrix3d K;
        cv::cv2eigen(cv_K, K);

        const size_t ld_number = landmarks.size();

        std::vector<cv::Point2f> point_last_raw(ld_number);
        std::vector<bool> valid_last(ld_number, false);

        detectFeatures(v_Twc[0], K, landmarks, point_last_raw, valid_last);
        std::vector<cv::Point2f> point_last; point_last.reserve(ld_number);

        /* start odomerty */
        std::vector<Eigen::Matrix4d> pose_est;
        pose_est.reserve(v_Twc.size());

        Eigen::Matrix4d Twc_last = v_Twc[0];
        pose_est.push_back(Twc_last);

        std::vector<cv::Point2f> point_curr; point_curr.reserve(ld_number);
        std::vector<cv::Point2f> point_curr_raw(ld_number);
        std::vector<bool> valid_curr(ld_number, false);

        for (size_t i = 1; i < pose_num; i++)
        {
            /* get scale form gt */
            double t_scale = 1.0;
            {
                Eigen::Matrix4d gt_Twc_last = v_Twc[i-1];
                Eigen::Matrix4d gt_Twc_curr = v_Twc[i];
                Eigen::Matrix4d gt_T_cur_last = gt_Twc_curr.inverse() * gt_Twc_last;
                t_scale = gt_T_cur_last.block(0, 3, 3, 1).norm();
            }

            /* get features of current frame */
            detectFeatures(v_Twc[i], K, landmarks, point_curr_raw, valid_curr);
            point_curr.clear(); point_last.clear();
            for(size_t i = 0 ; i < ld_number; i++)
            {
                if(valid_curr[i] && valid_last[i]) {
                    point_last.emplace_back(point_last_raw[i]);
                    point_curr.emplace_back(point_curr_raw[i]);
                }
            }

            Eigen::Matrix4d Twc_curr;
            // estimate fundamental matrix between frame i-1 and frame i, then recover pose from fundamental matrix
            {
                Eigen::Matrix3d F;
                Eigen::Matrix3d E;
                Eigen::Matrix3d R;
                Eigen::Vector3d t;
                cv::Mat cv_R, cv_t, cv_E;
                // ....

#if 1
                cv::Mat inliers;
//                cv::Mat cv_F = cv::findFundamentalMat(point_curr, point_last, inliers);
//                cv_E = cv_K.t() * cv_F * cv_K;
                cv_E = cv::findEssentialMat(point_curr, point_last, cv_K, cv::RANSAC, 0.999, 1.0, inliers);
                // std::cout << "cv F:\n" << F << std::endl;
#else
                std::vector<bool> inliers(point_curr.size(), false);
                cv::Mat cv_F = TwoViewGeometry::FindFundamental(point_curr, point_last, inliers);
                cv::cv2eigen(cv_F, F);
#endif
                // std::cout << "my F:\n" <<  F << std::endl;
                Twc_curr = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

#if 1
                cv::recoverPose(cv_E, point_curr, point_last, cv_K, cv_R, cv_t, inliers);
                cv::cv2eigen(cv_R, R);
                cv::cv2eigen(cv_t, t);
#else
                cv::Mat cv_F_flt, cv_K_flt;
                cv_F.convertTo(cv_F_flt, CV_32F);
                cv_K.convertTo(cv_K_flt, CV_32F);
                std::vector<cv::DMatch> vMaches12(point_curr.size());
                std::vector<cv::Point3f> vP3D;
                std::vector<bool> vbTriangulated;
                for(int i=0; i<point_curr.size(); ++i)
                {
                    vMaches12[i].queryIdx = i;
                    vMaches12[i].trainIdx = i;
                }

                if(!TwoViewGeometry::ReconstructF(point_curr, point_last,
                                                   inliers,
                                                   cv_F_flt, cv_K_flt, cv_R, cv_t))
                {
                    std::cerr << "error!!!" << std::endl;
                    continue;
                }

                cv::Mat cv_R_dbl, cv_t_dbl;
                cv_R.convertTo(cv_R_dbl, CV_64F);
                cv_t.convertTo(cv_t_dbl, CV_64F);
                cv::cv2eigen(cv_R_dbl, R);
                cv::cv2eigen(cv_t_dbl, t);
#endif
                point_last = point_curr;

                t = t_scale * t.normalized();
                transform.block<3,3>(0,0) = R;
                transform.block<3,1>(0,3) = t;
                Twc_curr = Twc_last * transform;
            }

            /* show estimated trajectory */
            {
                Eigen::Vector3d twc0 = Twc_last.block(0, 3, 3, 1);
                Eigen::Vector3d twc1 = Twc_curr.block(0, 3, 3, 1);
                cv::Point3d pose_begin(twc0[0], twc0[1], twc0[2]);
                cv::Point3d pose_end(twc1[0], twc1[1], twc1[2]);
                cv::viz::WLine trag_line(pose_begin, pose_end, cv::viz::Color(0,255,255));
                window.showWidget("trag_"+std::to_string(i), trag_line);

                Eigen::Matrix3d Rwc1 = Twc_curr.block(0, 0, 3, 3);
                Eigen::Vector3d w_cam_z_dir = Rwc1 * Eigen::Vector3d(0, 0, 1);
                cv::Point3d obvs_dir(w_cam_z_dir[0], w_cam_z_dir[1], w_cam_z_dir[2]);
                cv::Point3d obvs_begin(twc1[0], twc1[1], twc1[2]);
                cv::viz::WLine obvs_line(obvs_begin, obvs_begin+obvs_dir, cv::viz::Color(255,0,100));
                window.showWidget("cam_z_"+std::to_string(i), obvs_line);
            }

            /* update */
            pose_est.push_back(Twc_curr);
            Twc_last = Twc_curr;
            point_last_raw.swap(point_curr_raw);
            valid_last.swap(valid_curr);
        }

        /* save trajectory for evalution */
        if(times == 0) saveTrajectoryTUM("frame_traj_gt.txt", v_Twc);

        char file_name[100];
        sprintf(file_name,"frame_traj_est_E_%.2f.txt", ld_scale);
        saveTrajectoryTUM(file_name, pose_est);
    }
    //    while(!window.wasStopped())
    //    {
    //        window.spinOnce(1, true);
    //    }
}
