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

#include <from_opencv/dls.h>
#include <from_opencv/upnp.h>
#include "two_view_geometry.h"
#include "pnp_wrapper.h"


EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

void createLandmarks(std::vector<Eigen::Vector3d> &points)
{
    points.clear();

#if 0
    float scale = 5; 
    const double k = 0.5;
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
    std::normal_distribution<double> d_x{0.0, 4.0};
    std::normal_distribution<double> d_y{0.0, 10.0};
    std::normal_distribution<double> d_z{0.0, 10.0};
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

static float pixel_sigma = 1.0;

void detectFeatures(const Eigen::Matrix4d &Twc, const Eigen::Matrix3d &K,
                    const std::vector<Eigen::Vector3d> &landmarks, 
                    std::vector<Eigen::Vector2i> &features, std::vector<bool> &matched, bool add_noise = true)
{
    std::mt19937 gen{12345};
    std::normal_distribution<> d{0.0, pixel_sigma};

    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
    Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
    Eigen::Matrix3d Rcw = Rwc.transpose();
    Eigen::Vector3d tcw = -Rcw * twc;

    features.resize(landmarks.size());
    matched.resize(landmarks.size());
    for (size_t l = 0; l < landmarks.size(); ++l)
    {
        matched[l] = false;
        Eigen::Vector3d wP = landmarks[l];
        Eigen::Vector3d cP = Rcw * wP + tcw;

        if(cP[2] < 0) continue;

        float noise_u = add_noise ? std::round(d(gen)) : 0.0f;
        float noise_v = add_noise ? std::round(d(gen)) : 0.0f;

        Eigen::Vector3d ft = K * cP;
        int u = ft[0]/ft[2] + 0.5 + noise_u;
        int v = ft[1]/ft[2] + 0.5 + noise_v;
        Eigen::Vector2i obs(u, v);
        features[l] = obs;
        matched[l] = true;
        //std::cout << l << " " << obs.transpose() << std::endl;
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

/**
 * create initial map by two consecutive frames
 *
 * @param point_curr feature point in current image [input]
 * @param point_last feature point in last image [input]
 * @param cv_K camera projection matrix [input]
 * @param T_curr_last transformation between current camera to last camera [output]
 * @param map_points map point [output]
 * @param good_matches if feature point match is good between current and last [input]
 * @return init map created success or fail
 */
bool createInitMap(const std::vector<cv::Point2f> &point_curr,
                   const std::vector<cv::Point2f> &point_last,
                   const cv::Mat &cv_K,
                   Eigen::Matrix4d &T_curr_last,
                   std::vector<Eigen::Vector3d> &map_points,
                   std::vector<bool> &good_matches)
{
    static int i = 0; i++;
    Eigen::Matrix3d K;
    cv::cv2eigen(cv_K, K);
    cv::Mat temp_K;
    cv_K.convertTo(temp_K, CV_32F);
    cv::Mat R21, t21;

    std::vector<cv::DMatch> vMatches12;
    vMatches12.reserve(point_curr.size());
    for (size_t n = 0; n < point_curr.size(); n++)
    {
        if (good_matches[n])
        {
            vMatches12.emplace_back(n, n, 0);
        }
    }

    std::vector<bool> vbMatchesInliers;

    cv::Mat inlier_mask;
    // cv::Mat cv_F = cv::findFundamentalMat(point_last, point_curr, cv::FM_RANSAC, 3, 0.99, inlier_mask);
    // cv::Mat cv_E = cv_K.t() * cv_F * cv_K;
    cv::Mat cv_E = cv::findEssentialMat(point_last, point_curr, cv_K, cv::RANSAC, 0.999, 1.0, inlier_mask);

    int num_inlier_F = cv::countNonZero(inlier_mask);
    printf("%d - F inlier: %d(%d)\n", i, num_inlier_F, (int)point_last.size());

    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    cv::recoverPose(cv_E, point_last, point_curr, cv_K, R21, t21, inlier_mask);
    int num_inlier_Pose = cv::countNonZero(inlier_mask);
    printf("%d - RT inlier: %d(%d)\n", i, num_inlier_Pose, num_inlier_F);
    vbMatchesInliers.resize(point_curr.size());
    for (size_t n = 0; n < point_curr.size(); n++) { vbMatchesInliers[n] = inlier_mask.at<uint8_t>(n) != 0; }
    cv::cv2eigen(R21, R);
    cv::cv2eigen(t21, t);

    /* comput Twc_cur = Twc_last * T_last_curr */
    T_curr_last = Eigen::Matrix4d::Identity();
    T_curr_last.block(0, 0, 3, 3) = R;
    T_curr_last.block(0, 3, 3, 1) = t;

    /* create mappoints */
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));

    temp_K.copyTo(P1.rowRange(0,3).colRange(0,3));
    cv::Mat P2(3,4,CV_32F);
    R21.copyTo(P2.rowRange(0,3).colRange(0,3));
    t21.copyTo(P2.rowRange(0,3).col(3));
    P2 = temp_K*P2;
    for (size_t n = 0; n < vMatches12.size(); n++)
    {

        if (!vbMatchesInliers[vMatches12[n].queryIdx])
        {
            continue;
        }

        cv::Mat p3d_c1;
        TwoViewGeometry::Triangulate(point_last[n], point_curr[n], P1, P2, p3d_c1);

        if(!std::isfinite(p3d_c1.at<float>(0)) ||
            !std::isfinite(p3d_c1.at<float>(1)) ||
            !std::isfinite(p3d_c1.at<float>(2)))
        {
            vbMatchesInliers[vMatches12[n].queryIdx]=false;
            continue;
        }

        Eigen::Vector3d &mpt = map_points[vMatches12[n].queryIdx];
        mpt[0] = p3d_c1.at<float>(0);
        mpt[1] = p3d_c1.at<float>(1);
        mpt[2] = p3d_c1.at<float>(2);
    }
    good_matches = vbMatchesInliers;
    return true;
}

int main(int argc, char * argv[])
{
    int itr_times = 5;
    float th = 1.0f;
    float cf = 0.99f;
    int method = cv::SOLVEPNP_DLS;
    if(argc == 6) {
        itr_times = std::atoi(argv[1]);
        th = std::atof(argv[2]);
        cf = std::atof(argv[3]);
        method = std::atoi(argv[4]);
        pixel_sigma = std::atof(argv[5]);
    }
    std::cout << "pixel_sigma=" << pixel_sigma << std::endl;
    std::cout << "method=" << method << std::endl;
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

    std::vector<Eigen::Vector3d> landmarks;
    std::vector<Eigen::Matrix4d> v_Twc;
    createLandmarks(landmarks);
    createCameraPose(v_Twc, Eigen::Vector3d(0,0,0));
    const size_t pose_num = v_Twc.size();

    /* show landmarks */
    {
        cv::Mat point_cloud = cv::Mat::zeros(landmarks.size(), 1, CV_32FC3);
        for (size_t i = 0; i < landmarks.size(); i++)
        {
            point_cloud.at<cv::Vec3f>(i)[0] = landmarks[i][0];
            point_cloud.at<cv::Vec3f>(i)[1] = landmarks[i][1];
            point_cloud.at<cv::Vec3f>(i)[2] = landmarks[i][2];
        }
        cv::viz::WCloud cloud(point_cloud);
        window.showWidget("cloud", cloud);
    }

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
    //cv::Mat cv_K = (cv::Mat_<double>(3, 3) << 1, 0, 1, 0, 480, 240, 0, 0, 1);
    Eigen::Matrix3d K;
    cv::cv2eigen(cv_K, K);

    std::vector<cv::Point2f> point_last;
    std::vector<Eigen::Vector2i> features_curr, features_last;
    std::vector<bool> features_matched_cur, features_matched_last;
    detectFeatures(v_Twc[0], K, landmarks, features_last, features_matched_last);

    /* start odomerty */
    std::vector<Eigen::Matrix4d> pose_est;
    std::vector<Eigen::Matrix4d> pose_gt;
    pose_est.reserve(v_Twc.size());
    pose_gt.reserve(v_Twc.size());

    Eigen::Matrix4d Twc_last = v_Twc[0];
    pose_est.push_back(Twc_last);
    pose_gt.push_back(Twc_last);

    bool init_flag = false;
    std::vector<Eigen::Vector3d> map_points;
    map_points.resize(landmarks.size());
    int tc = 0;
    for (size_t i = 1; i < pose_num; i++)
    {
        /* get features of current frame */
        detectFeatures(v_Twc[i], K, landmarks, features_curr, features_matched_cur);

        /* get matched features */
        std::vector<cv::Point2f> point_curr;
        std::vector<cv::Point2f> point_last;
        std::vector<bool> good_matches(landmarks.size(), false);
        point_curr.reserve(landmarks.size());
        point_last.reserve(landmarks.size());
        for(size_t i = 0 ; i < features_curr.size() ; i++)
        {
            point_curr.push_back(cv::Point2f(features_curr[i][0], features_curr[i][1]));
            point_last.push_back(cv::Point2f(features_last[i][0], features_last[i][1]));
            if (!features_matched_cur[i] || !features_matched_last[i])
                continue;

            good_matches[i] = true;
        }

        Eigen::Matrix4d Twc_curr;

        /* initial map create */
        if (!init_flag)
        {
#if 0
            /* create initial map from groundtruth */
            Twc_curr = v_Twc[i];
            map_points = landmarks;
            init_flag = true;
#else
            /* create initial map by solve F matrix */
            Eigen::Matrix4d T_curr_last;
            init_flag = createInitMap(point_curr, point_last, cv_K, T_curr_last, map_points, good_matches);

            double t_scale = 1.0;
            {
                Eigen::Matrix4d gt_Twc_last = Twc_last;
                Eigen::Matrix4d gt_Twc_curr = v_Twc[i];
                Eigen::Matrix4d gt_T_cur_last = gt_Twc_curr.inverse() * gt_Twc_last;
                t_scale = gt_T_cur_last.block(0, 3, 3, 1).norm();
            }
            T_curr_last.block(0, 3, 3, 1) *= t_scale;
            Twc_curr = Twc_last * T_curr_last.inverse();

            // convert to world coordinate
            for (size_t n = 0; n < map_points.size(); n++)
            {
                if (!good_matches[n]) continue;
                Eigen::Vector3d &mpt = map_points[n];
                mpt *= t_scale;
                mpt = Twc_last.block(0, 0, 3, 3) * mpt + Twc_last.block(0, 3, 3, 1);
            }

#endif
            if (!init_flag)
            {
                continue;
            }
        }
        /* solve pose by pnp */
        else
        {
            std::vector<cv::Point3f> obj_pts;
            std::vector<cv::Point2f> img_pts;
            for (size_t n = 0; n < good_matches.size(); n++)
            {
                if (!good_matches[n]) { continue; }
                Eigen::Vector3d &mpt = map_points[n];
                obj_pts.emplace_back(mpt[0], mpt[1], mpt[2]);
                img_pts.push_back(point_curr[n]);
            }

            // TODO homework
            cv::Mat r(3, 1, CV_64F), t(3, 1, CV_64F);
            auto tp0 = std::chrono::high_resolution_clock::now();
            cv::Mat inliers;
            bool suc = false;
            //bool suc = cv::solvePnPRansac(obj_pts, img_pts, cv_K, cv::Mat(), r, t, false, itr_times, th, cf, inliers, method);
            //std::cout << "cv:" << inliers.t() << std::endl;
            if(method == cv::SOLVEPNP_P3P || method == cv::SOLVEPNP_EPNP || method==cv::SOLVEPNP_UPNP || method == cv::SOLVEPNP_DLS)
                suc = solve_pnp_ransac(obj_pts, img_pts, cv_K, cv::Mat(), r, t, false, itr_times, th, cf, inliers, method);
            else
            {
                cv::Mat opoints(obj_pts);
                cv::Mat ipoints(img_pts);
                cv::Mat rmat;
                if(method == cv::SOLVEPNP_DLS) {
                    dls dls_pnp(opoints, ipoints);
                    suc = dls_pnp.compute_pose(rmat, t);
                } else if(method == cv::SOLVEPNP_UPNP){
                    upnp upnp_pnp(cv_K, opoints, ipoints);
                    double fu = upnp_pnp.compute_pose(rmat, t);
                    suc = true;
                } else {
                    throw std::logic_error("unsupported pnp method!");
                }
                if(suc) Rodrigues(rmat, r);
            }
            //std::cout << "my:" << inliers << std::endl;
            auto tp1 = std::chrono::high_resolution_clock::now();
            tc += std::chrono::duration_cast<std::chrono::milliseconds>(tp1-tp0).count();
            if(!suc)
            {
                std::cout << "error!!!" << std::endl;
                continue;
            }


            Eigen::Vector3d rv(r.at<double>(0), r.at<double>(1), r.at<double>(2));
            Eigen::AngleAxisd  an(rv.norm(), rv.normalized());
            Twc_curr.setIdentity();
            Twc_curr.block<3,3>(0,0) = an.matrix();
            Twc_curr.block<3,1>(0,3) << t.at<double>(0), t.at<double>(1), t.at<double>(2);
            Twc_curr = Twc_curr.inverse().eval();
        }

        /* show estimated mappoints */
        {
            int N = std::count(good_matches.begin(), good_matches.end(), true);
            cv::Mat point_cloud = cv::Mat::zeros(N, 1, CV_32FC3);
            for (size_t k = 0, n = 0; k < good_matches.size(); k++)
            {
                if (!good_matches[k]) { continue; }
                point_cloud.at<cv::Vec3f>(n)[0] = map_points[k][0];
                point_cloud.at<cv::Vec3f>(n)[1] = map_points[k][1];
                point_cloud.at<cv::Vec3f>(n)[2] = map_points[k][2];
                n++;
            }
            cv::viz::WCloud cloud(point_cloud, cv::viz::Color::orange());
            // window.removeWidget("mappoints");
            window.showWidget("mappoints", cloud);
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
        pose_gt.push_back(v_Twc[i]);
        Twc_last = Twc_curr;
        features_last = features_curr;
        features_matched_last = features_matched_cur;
        window.spinOnce(1, true);
    }

    std::cout << "average tc:" << tc*1.0/(pose_num-1) << std::endl;
    /* save trajectory for evalution */
    saveTrajectoryTUM("frame_traj_gt.txt", pose_gt);
    saveTrajectoryTUM("frame_traj_est.txt", pose_est);

//    while(!window.wasStopped())
//    {
//        window.spinOnce(1, true);
//    }
}
