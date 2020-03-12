#include <iostream> 
#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/eigen.hpp>

#include "define.h"
#include "utils.h"
#include "two_view_geometry.h"
#include "optimizer.h"

void detectFeatures(const Eigen::Matrix4d &Twc, const Eigen::Matrix3d &K,
                    const std::vector<Eigen::Vector3d> &landmarks, const std::vector<Eigen::Vector3d> &normals,
                    std::vector<Eigen::Vector2i> &features, std::vector<int32_t> &matched, bool add_noise = true)
{
    assert(landmarks.size() == normals.size());
    std::mt19937 gen{12345};
    const float pixel_sigma = 1.0;
    std::normal_distribution<> d{0.0, pixel_sigma};

    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
    Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
    Eigen::Matrix3d Rcw = Rwc.transpose();
    Eigen::Vector3d tcw = -Rcw * twc;

    features.resize(landmarks.size());
    matched = std::vector<int32_t> (landmarks.size(), -1);
    for (size_t l = 0; l < landmarks.size(); ++l)
    {
        matched[l] = -1;
        Eigen::Vector3d wP = landmarks[l];
        Eigen::Vector3d cP = Rcw * wP + tcw;

        if (cP[2] < 0) continue;

#if 1
        /* check landmark in sight by mesh normal */
        Eigen::Vector3d obv_dir = twc - landmarks[l];
        double costheta = obv_dir.dot(normals[l]) / (obv_dir.norm() * normals[l].norm());
        if (costheta < 0.5) continue;
#endif

        float noise_u = add_noise ? std::round(d(gen)) : 0.0f;
        float noise_v = add_noise ? std::round(d(gen)) : 0.0f;

        Eigen::Vector3d ft = K * cP;
        int u = ft[0]/ft[2] + 0.5 + noise_u;
        int v = ft[1]/ft[2] + 0.5 + noise_v;
        Eigen::Vector2i obs(u, v);
        features[l] = obs;
        matched[l] = l;
    }
}

/**
 * \brief feature_match calculate and return match of pair <obvs_id1, obvs_id2>
 */
std::vector<FMatch> feature_match(const Frame &frame1, const Frame &frame2, float outlier_rate = 0.0f)
{
    const std::vector<int32_t> &obvs1 = frame1.fts_obvs_;
    const std::vector<int32_t> &obvs2 = frame2.fts_obvs_;
    std::map<int32_t, int32_t> obvs_map; // map from landmark id to observation feature point id
    for(size_t n = 0 ; n < obvs1.size() ; n++)
    {
        if (obvs1[n] < 0) continue;
        obvs_map.emplace(obvs1[n], n);
    }

    std::vector<FMatch> matches; matches.reserve(obvs_map.size());
    for(size_t n = 0 ; n < obvs2.size() ; n++)
    {
        if (obvs2[n] < 0) continue;
        if (!obvs_map.count(obvs2[n])) continue;
        int32_t idx1 = obvs_map[obvs2[n]];
        int32_t idx2 = n;
        matches.emplace_back(idx1, idx2, false);
    }

    if(matches.empty()) return matches;

    // if add outliers
    size_t outlier_num = outlier_rate * matches.size();
    static std::mt19937 gen{12345};
    std::uniform_int_distribution<> d{0, static_cast<int>(matches.size())-1};

    std::set<size_t> fts_set;
    for (size_t i = 0; i < outlier_num; i++)
    {
        size_t id1 = d(gen);
        if (fts_set.count(id1)) { continue; }
        fts_set.insert(id1);

        size_t id2;
        do {
            id2 = d(gen);
        } while(obvs2[id2] < 0 || id2==matches[id1].second);
        matches[id1].second = id2;
        matches[id1].outlier_gt = true;
    }

    return matches;
}

void outlier_rejection(Frame &frame_last,Frame &frame_curr, LoaclMap &map, std::vector<FMatch> &matches)
{
    const double sigma = 1.0;
    const double thr = sigma * 4;
    std::list<double> rpj_err;
    for (size_t n = 0; n < matches.size(); n++)
    {
        uint32_t idx_curr = matches[n].first;
        uint32_t idx_last = matches[n].second;
        int32_t mpt_idx = frame_last.mpt_track_[idx_last];

        if (mpt_idx < 0) { continue; }
        Eigen::Vector3d &mpt = map.mpts_[mpt_idx];
        // TODO homework
        // ....
        // if (...)
        // {
        //     matches[n].outlier = true;
        // }
    }

    for (size_t n = 0; n < matches.size(); n++)
    {
        if (matches[n].outlier)
        {
            uint32_t idx_curr = matches[n].first;
            frame_curr.mpt_track_[idx_curr] = -1;
        }
    }
}

bool createInitMap(Frame &frame_last, Frame &frame_curr, LoaclMap &map, std::vector<FMatch> &matches)
{
    static int i = 0; i++;

    cv::Mat cv_K;
    Eigen::Matrix3f temp_K = frame_last.K_.cast<float>();
    cv::eigen2cv(temp_K, cv_K);

    cv::Mat R21, t21;

    /* get matched features */
    std::vector<cv::Point2f> point_curr;
    std::vector<cv::Point2f> point_last;
    point_curr.reserve(matches.size());
    point_last.reserve(matches.size());
    for(size_t n = 0 ; n < matches.size() ; n++)
    {  
        uint32_t idx_curr = matches[n].first;
        uint32_t idx_last = matches[n].second;
        point_curr.push_back(cv::Point2f(frame_curr.fts_[idx_curr][0], frame_curr.fts_[idx_curr][1]));
        point_last.push_back(cv::Point2f(frame_last.fts_[idx_last][0], frame_last.fts_[idx_last][1]));
    }

    std::vector<bool> vbMatchesInliers;
    Eigen::Matrix4d T_curr_last;

    cv::Mat inlier_mask;
    cv::Mat cv_E = cv::findEssentialMat(point_last, point_curr, cv_K, cv::RANSAC, 0.999, 1.0, inlier_mask);

    int num_inlier_F = cv::countNonZero(inlier_mask);
    printf("%d - F inlier: %d(%d)\n", i, num_inlier_F, (int)point_curr.size());

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
    T_curr_last.block(0, 3, 3, 1) = t.normalized();

    /* create mappoints */
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));

    cv_K.copyTo(P1.rowRange(0,3).colRange(0,3));
    cv::Mat P2(3,4,CV_32F);
    R21.copyTo(P2.rowRange(0,3).colRange(0,3));
    t21.copyTo(P2.rowRange(0,3).col(3));
    P2 = cv_K*P2;

    map.clear();
    frame_last.mpt_track_ = std::vector<int32_t>(frame_last.N_, -1);
    frame_curr.mpt_track_ = std::vector<int32_t>(frame_curr.N_, -1);
    for (size_t n = 0; n < matches.size(); n++)
    {
        matches[n].outlier = true;
        if (!vbMatchesInliers[n]) { continue; }

        /* if fts in last frame has been matched, then excaped */
        uint32_t idx_curr = matches[n].first;
        uint32_t idx_last = matches[n].second;
        if (frame_last.mpt_track_[idx_last] > 0) { continue; }

        cv::Mat p3d_c1;
        TwoViewGeometry::Triangulate(point_last[n], point_curr[n], P1, P2, p3d_c1);

        if(!std::isfinite(p3d_c1.at<float>(0)) ||
            !std::isfinite(p3d_c1.at<float>(1)) ||
            !std::isfinite(p3d_c1.at<float>(2)))
        {
            continue;
        }

        Eigen::Vector3d mpt;
        mpt[0] = p3d_c1.at<float>(0);
        mpt[1] = p3d_c1.at<float>(1);
        mpt[2] = p3d_c1.at<float>(2);
        int32_t mpt_idx = map.add(mpt);
        frame_curr.mpt_track_[idx_curr] = mpt_idx;
        frame_last.mpt_track_[idx_last] = mpt_idx;
        matches[n].outlier = false;
    }

    frame_last.Twc_ = Eigen::Matrix4d::Identity();
    frame_curr.Twc_ = frame_last.Twc_ * T_curr_last.inverse();
    outlier_rejection(frame_last, frame_curr, map, matches);
    two_view_ba(frame_last, frame_curr, map, matches);
    outlier_rejection(frame_last, frame_curr, map, matches);

    return true;
}

int main()
{
    cv::viz::Viz3d window("window");
    cv::viz::WCoordinateSystem world_coord(1.0), camera_coord(0.5);
    window.showWidget("Coordinate", world_coord);
    window.showWidget("Camera", camera_coord);


    cv::viz::Mesh mesh;
    loadMeshAndPerprocess(mesh, "/data/bun_zipper_res3.ply");

    // window.showWidget("mesh", cv::viz::WMesh(mesh));
    // window.showWidget("normals", cv::viz::WCloudNormals(mesh.cloud, mesh.normals, 1, 1.0f, cv::viz::Color::green()));
    // window.setRenderingProperty("normals", cv::viz::LINE_WIDTH, 2.0);
    std::cout << mesh.normals.size() << std::endl;
    std::cout << mesh.normals.type() << std::endl;
    std::cout << CV_32FC3 << std::endl;
    std::cout << CV_64FC3 << std::endl;

    std::vector<Eigen::Vector3d> landmarks;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Matrix4d> v_Twc;
    createLandmarksFromMesh(mesh, landmarks, normals);
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
        window.showWidget("__cloud", cloud);
        // window.setRenderingProperty("cloud", cv::viz::POINT_SIZE, 3.0);
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
    cv::Mat cv_K_flt; cv_K.convertTo(cv_K_flt, CV_32F);
    //cv::Mat cv_K = (cv::Mat_<double>(3, 3) << 1, 0, 1, 0, 480, 240, 0, 0, 1);
    Eigen::Matrix3d K;
    cv::cv2eigen(cv_K, K);

    std::vector<Frame, Eigen::aligned_allocator<Frame>> frames;
    frames.reserve(v_Twc.size());
    frames.emplace_back(Frame(0, landmarks.size(), K));
    frames.back().Twc_ = v_Twc[0];
    detectFeatures(v_Twc[0], K, landmarks, normals, frames.back().fts_, frames.back().fts_obvs_);

    /* start odomerty */
    std::vector<Eigen::Matrix4d> pose_est;
    std::vector<Eigen::Matrix4d> pose_gt;
    pose_est.reserve(v_Twc.size());
    pose_gt.reserve(v_Twc.size());

    pose_est.push_back(v_Twc[0]);
    pose_gt.push_back(v_Twc[0]);

    bool init_flag = false;
    LoaclMap map(landmarks.size());
    for (size_t i = 1; i < pose_num; i++)
    {
        /* get features of current frame */
        Frame &frame_last = frames.back();
        Frame frame_curr(i, landmarks.size(), K);
        detectFeatures(v_Twc[i], K, landmarks, normals, frame_curr.fts_, frame_curr.fts_obvs_);

        /* get matched features */
        frame_curr.mpt_track_ = std::vector<int32_t>(landmarks.size(), -1);
        std::vector<FMatch> matches = feature_match(frame_curr, frame_last, 0.0);
        assert(!matches.empty());

        std::cout << "[" << std::setw(3) << frame_curr.idx_ << "] match features " << matches.size() << std::endl;

        Eigen::Matrix4d Twc_curr;

        /* initial map create */
        if (!init_flag)
        {
            bool is_success = false;

            /* create initial map by solve F matrix */
            is_success = createInitMap(frame_last, frame_curr, map, matches);
            Eigen::Matrix4d T_curr_last = frame_curr.Twc_.inverse() * frame_last.Twc_; //! frame_last.Twc_ set to Identity in init
            frame_last.Twc_ = v_Twc[0];
            frame_curr.Twc_ = frame_last.Twc_ * T_curr_last.inverse();

            double t_scale = 1.0;
            {
#if 0
                Eigen::Matrix4d gt_Twc_last = frame_last.Twc_;
                Eigen::Matrix4d gt_Twc_curr = v_Twc[i];
                Eigen::Matrix4d gt_T_cur_last = gt_Twc_curr.inverse() * gt_Twc_last;
                t_scale = gt_T_cur_last.block(0, 3, 3, 1).norm();
#else
                double gt_dist = 0.0;
                Eigen::Matrix4d last_Tcw = frame_last.Twc_.inverse();
                for (size_t n = 0; n < landmarks.size(); n++)
                {
                    int32_t mpt_idx = frame_curr.mpt_track_[n];
                    if (mpt_idx < 0) continue;
                    Eigen::Vector3d mpt = landmarks[n];
                    mpt = last_Tcw.block(0, 0, 3, 3) * mpt + last_Tcw.block(0, 3, 3, 1);
                    gt_dist += mpt[2];
                }

                double est_dist = 0.0;
                for (size_t n = 0; n < landmarks.size(); n++)
                {
                    int32_t mpt_idx = frame_curr.mpt_track_[n];
                    if (mpt_idx< 0) continue;
                    Eigen::Vector3d mpt = map.mpts_[mpt_idx];
                    est_dist += mpt[2];
                }

                t_scale = gt_dist / est_dist;
#endif
            }

            T_curr_last.block(0, 3, 3, 1) *= t_scale;
            frame_curr.Twc_ = frame_last.Twc_ * T_curr_last.inverse();

            int mpts_count = 0;
            for (size_t n = 0; n < landmarks.size(); n++)
            {
                int32_t mpt_idx = frame_curr.mpt_track_[n];
                if (mpt_idx < 0) continue;
                Eigen::Vector3d &mpt = map.mpts_[mpt_idx];
                mpt *= t_scale;
                mpt = frame_last.Twc_.block(0, 0, 3, 3) * mpt + frame_last.Twc_.block(0, 3, 3, 1);
                mpts_count++;
            }

            if (!is_success)
            {
                continue;
            }
            init_flag = is_success;
            std::cout << "[" << std::setw(3) << frame_curr.idx_ << "] create init mappoints " << mpts_count << std::endl;
        }
        /* solve pose by pnp */
        else
        {
            std::vector<cv::Point3f> obj_pts;
            std::vector<cv::Point2f> img_pts;
            for (size_t n = 0; n < matches.size(); n++)
            {
                uint32_t idx_curr = matches[n].first;
                uint32_t idx_last = matches[n].second;
                int32_t mpt_idx = frame_last.mpt_track_[idx_last];

                if (mpt_idx < 0) { continue; }
                Eigen::Vector3d &mpt = map.mpts_[mpt_idx];
                obj_pts.emplace_back(mpt[0], mpt[1], mpt[2]);
                img_pts.push_back(cv::Point2f(frame_curr.fts_[idx_curr][0], frame_curr.fts_[idx_curr][1]));
            }

            std::cout << "[" << std::setw(3) << frame_curr.idx_ << "] track landmarks " << obj_pts.size() << std::endl;
            if (obj_pts.size() < 5)
            {
                std::cout << "!!!!!!!!!!! track lost !!!!!!!!!!!!" << std::endl;
                window.spin();
            }

            cv::Mat rvec, tvec;
            cv::solvePnPRansac(obj_pts, img_pts, cv_K, cv::Mat(), rvec, tvec, false,
                               100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

            cv::Mat Rmat(3, 3, CV_64FC1);
            Rodrigues(rvec, Rmat);
            Eigen::Matrix3d R;
            Eigen::Vector3d t;

            cv::cv2eigen(Rmat, R);
            cv::cv2eigen(tvec, t);
            frame_curr.Twc_.block(0, 0, 3, 3) = R.transpose();
            frame_curr.Twc_.block(0, 3, 3, 1) = -R.transpose() * t;

            /* create new mappoints */
            // TODO homework
            cv::Mat cv_P1(3, 4, CV_32F, cv::Scalar(0));
            cv_K_flt.copyTo(cv_P1.colRange(0,3));
            cv::Mat cv_P2(3,4, CV_32F);
            Eigen::Matrix<float,3,4> P2_e = (K * (frame_curr.Twc_.inverse()*frame_last.Twc_).block<3,4>(0,0)).cast<float>();
            cv::eigen2cv(P2_e, cv_P2);

            for (size_t n = 0; n < matches.size(); n++)
            {
                if (matches[n].outlier) { continue; }

                uint32_t idx_curr = matches[n].first;
                uint32_t idx_last = matches[n].second;
                int32_t mpt_idx = -1;
                if (frame_last.mpt_track_[idx_last] >= 0)
                {
                    // use last frame's mpts, not re-triangulate
                    mpt_idx = frame_last.mpt_track_[idx_last];
                    frame_curr.mpt_track_[idx_curr] = mpt_idx;
                    continue;
                }

                cv::Mat p3d_c1;
                // TODO homework
                // solve the mappoint
                cv::Point2f pt1(frame_last.fts_[idx_last][0], frame_last.fts_[idx_last][1]);
                cv::Point2f pt2(frame_curr.fts_[idx_curr][0], frame_curr.fts_[idx_curr][1]);
                TwoViewGeometry::Triangulate(pt1, pt2, cv_P1, cv_P2, p3d_c1);
                // remove the 'continue'
                // continue;

                if(!std::isfinite(p3d_c1.at<float>(0)) ||
                    !std::isfinite(p3d_c1.at<float>(1)) ||
                    !std::isfinite(p3d_c1.at<float>(2)))
                {
                    frame_curr.mpt_track_[idx_curr] = -1;
                    matches[n].outlier = true;
                    continue;
                }

                Eigen::Vector3d mpt = Eigen::Vector3f(p3d_c1.at<float>(0), p3d_c1.at<float>(1), p3d_c1.at<float>(2)).cast<double>();
                mpt = frame_last.Twc_.block(0,0,3,3)*mpt + frame_last.Twc_.block(0,3,3,1);
                mpt_idx = map.add(mpt);

                frame_curr.mpt_track_[idx_curr] = mpt_idx;
                frame_last.mpt_track_[idx_last] = mpt_idx;
            }
            outlier_rejection(frame_last, frame_curr, map, matches);

            std::cout << "[" << std::setw(3) << frame_curr.idx_ << "] inlier landmarks "
                      << std::count_if(frame_curr.mpt_track_.begin(), frame_curr.mpt_track_.end(), [](int32_t idx){return idx >= 0;}) << std::endl;

            two_view_ba(frame_last, frame_curr, map, matches);
        }



        /* show landmarks in sight */
        {
            cv::Mat point_cloud = cv::Mat::zeros(landmarks.size(), 1, CV_32FC3);
            for (size_t k = 0; k < landmarks.size(); k++)
            {
                if (frame_curr.fts_obvs_[k] < 0) { continue; }
                point_cloud.at<cv::Vec3f>(k)[0] = landmarks[k][0];
                point_cloud.at<cv::Vec3f>(k)[1] = landmarks[k][1];
                point_cloud.at<cv::Vec3f>(k)[2] = landmarks[k][2];
            }
            cv::viz::WCloud cloud(point_cloud, cv::viz::Color::yellow());
            window.showWidget("cloud", cloud);
            window.setRenderingProperty("cloud", cv::viz::POINT_SIZE, 3.0);
        }

        /* show estimated mappoints */
        {
            int N = std::count(map.status_.begin(), map.status_.end(), true);
            cv::Mat point_cloud = cv::Mat::zeros(N, 1, CV_32FC3);
            for (size_t k = 0, n = 0; k < landmarks.size(); k++)
            {
                int32_t mpt_idx = frame_curr.mpt_track_[k];
                if (mpt_idx < 0) { continue; }
                point_cloud.at<cv::Vec3f>(n)[0] = map.mpts_[mpt_idx][0];
                point_cloud.at<cv::Vec3f>(n)[1] = map.mpts_[mpt_idx][1];
                point_cloud.at<cv::Vec3f>(n)[2] = map.mpts_[mpt_idx][2];
                n++;
            }
            cv::viz::WCloud cloud(point_cloud, cv::viz::Color::orange());
            // window.removeWidget("mappoints");
            window.showWidget("mappoints", cloud);
        }

        /* show estimated trajectory */
        {
            Eigen::Vector3d twc0 = frame_last.Twc_.block(0, 3, 3, 1);
            Eigen::Vector3d twc1 = frame_curr.Twc_.block(0, 3, 3, 1);
            cv::Point3d pose_begin(twc0[0], twc0[1], twc0[2]);
            cv::Point3d pose_end(twc1[0], twc1[1], twc1[2]);
            cv::viz::WLine trag_line(pose_begin, pose_end, cv::viz::Color(0,255,255));
            window.showWidget("trag_"+std::to_string(i), trag_line);

            Eigen::Matrix3d Rwc1 = frame_curr.Twc_.block(0, 0, 3, 3);
            Eigen::Vector3d w_cam_z_dir = Rwc1 * Eigen::Vector3d(0, 0, 1);
            cv::Point3d obvs_dir(w_cam_z_dir[0], w_cam_z_dir[1], w_cam_z_dir[2]);
            cv::Point3d obvs_begin(twc1[0], twc1[1], twc1[2]);
            cv::viz::WLine obvs_line(obvs_begin, obvs_begin+obvs_dir, cv::viz::Color(255,0,100));
            window.showWidget("cam_z_"+std::to_string(i), obvs_line);
        }

        /* update */
        map.update(frame_curr);
        frames.push_back(frame_curr);
        pose_est.push_back(frame_curr.Twc_);
        pose_gt.push_back(v_Twc[i]);
        window.spinOnce(10, true);
        // window.spin();
    }

    /* save trajectory for evalution */
    saveTrajectoryTUM("frame_traj_gt.txt", pose_gt);
    saveTrajectoryTUM("frame_traj_est.txt", pose_est);

    while(!window.wasStopped())
    {
        window.spinOnce(1, true);
    }
}
