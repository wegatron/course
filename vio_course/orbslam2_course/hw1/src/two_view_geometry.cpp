#include <vector>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include "two_view_geometry.h"

namespace TwoViewGeometry {

    void getsubset_inds(const size_t total_number, const size_t subset_number, vector<size_t> &subset_inds) {
        CV_Assert(total_number > subset_number);
        vector<size_t> inds(total_number);
        for (size_t i = 0; i < total_number; ++i) { inds[i] = i; }

        std::random_device rd;
        std::mt19937 g(rd());
        shuffle(inds.begin(), inds.end(), g);
        subset_inds.resize(subset_number);
        std::copy(inds.begin(), inds.begin() + subset_number, subset_inds.begin());
    }

    int run8points(vector<cv::Point2f> &pts1, vector<cv::Point2f> &pts2, cv::Matx33d &F) {
        if (pts1.size() < 8 || pts2.size() < 8) return 0;

        cv::Matx<double, 9, 9> A = cv::Matx<double, 9, 9>::zeros();
        cv::Vec<double, 9> r;
        for (auto i = 0; i < 8; ++i) {
            r << pts2[i].x * pts1[i].x, pts2[i].x * pts1[i].y, pts2[i].x,
                    pts2[i].y * pts1[i].x, pts2[i].y * pts1[i].y, pts2[i].y,
                    pts1[i].x, pts1[i].y, 1;
            A += r * r.t();
        }

        // eigen decomposition get the smallest eigen vector as result
        cv::Mat evalues, evec;
        if (!cv::eigen(A, evalues, evec)) return 0;
        if (fabs(evalues.at<double>(7)) < DBL_EPSILON) return 0;

        cv::Mat tmpF = evec.row(8).reshape(0, 3);
        cv::Mat u, w, vt;
        cv::SVD::compute(tmpF, w, u, vt, cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        w.at<double>(2) = 0;
        tmpF = u * cv::Mat::diag(w) * vt;

        tmpF.copyTo(F);
        return 1;
    }

    size_t find_inliers(const cv::Matx33d &F,
                        const vector<cv::Point2f> &pts1,
                        const vector<cv::Point2f> &pts2,
                        vector<bool> &inliers,
                        const double pixel_threshold) {
        size_t in_cnt = 0;
        const int n = pts1.size();
        Eigen::Matrix3d eF;
        cv::cv2eigen(F, eF);
        double sq_pixel_th = pixel_threshold * pixel_threshold;
        inliers.resize(n, false);
        for (int i = 0; i < n; ++i) {
            inliers[i] = false;
            Eigen::Vector3d x1(pts1[i].x, pts1[i].y, 1);
            Eigen::Vector3d x2(pts2[i].x, pts2[i].y, 1);

            // error为点到极线的距离
            Eigen::Vector3d v0 = eF * x1;
            double r = x2.dot(v0);
            double sqdis = r * r / (v0[0] * v0[0] + v0[1] * v0[1]);
            if (sqdis > sq_pixel_th) continue;
            Eigen::Vector3d v1 = x2.transpose() * eF;
            r = v1.dot(x1);
            sqdis = r * r / (v1[0] * v1[0] + v1[1] * v1[1]);
            if (sqdis > sq_pixel_th) continue;
            inliers[i] = true;
            ++in_cnt;
        }
        return in_cnt;
    }

    cv::Mat FindFundamental(const vector<cv::Point2f> &vPts1,
                            const vector<cv::Point2f> &vPts2,
                            vector<bool> &inliers,
                            int maxIterations) {
        cv::Mat F21;
        // point normalization and decentralizatlize
        CV_Assert(vPts1.size() == vPts2.size());
        const size_t n = vPts1.size();
        const float invn = 1.0 / n;
        cv::Point2f cpt1(0, 0), cpt2(0, 0);
        for (auto i = 0; i < n; ++i) {
            cpt1 += vPts1[i];
            cpt2 += vPts2[i];
        }
        cpt1 *= invn;
        cpt2 *= invn;

        vector<cv::Point2f> cv_pts1(n), cv_pts2(n);
        double scale1 = 0, scale2 = 0;
        for (auto i = 0; i < n; ++i) {
            cv_pts1[i] = vPts1[i] - cpt1;
            scale1 += norm(cv_pts1[i]);

            cv_pts2[i] = vPts2[i] - cpt2;
            scale2 += norm(cv_pts2[i]);
        }
        scale1 = sqrt(2.0) / (scale1 * invn);
        scale2 = sqrt(2.0) / (scale2 * invn);

        for (auto i = 0; i < n; ++i) {
            cv_pts1[i] = scale1 * cv_pts1[i];
            cv_pts2[i] = scale2 * cv_pts2[i];
        }

        std::vector<size_t> subset_inds(8);
        std::vector<cv::Point2f> selected_points1(8);
        std::vector<cv::Point2f> selected_points2(8);
        cv::Matx33d tmpF, bestF;
        size_t best_inlier_count = 0;
        cv::Matx33d T1(scale1, 0, -scale1 * cpt1.x, 0, scale1, -scale1 * cpt1.y, 0, 0, 1);
        cv::Matx33d T2(scale2, 0, -scale2 * cpt2.x, 0, scale2, -scale2 * cpt2.y, 0, 0, 1);
        std::vector<bool> tmp_inliers;
        for (int itr = 0; itr < maxIterations; ++itr) {
            // get 8 point subset
            getsubset_inds(n, 8, subset_inds);
            for (size_t id = 0; id < 8; ++id) {
                selected_points1[id] = cv_pts1[subset_inds[id]];
                selected_points2[id] = cv_pts2[subset_inds[id]];
            }

            // run 8 point calc F
            if (run8points(selected_points1, selected_points2, tmpF) == 0) continue;

            tmpF = T2.t() * tmpF * T1;

            // find inliers and outliers
            size_t cur_inliers_count = find_inliers(tmpF, vPts1, vPts2, tmp_inliers, 1.0);
            if (cur_inliers_count < best_inlier_count) continue;
            best_inlier_count = cur_inliers_count;

            std::copy(tmpF.val, tmpF.val + 9, bestF.val);
            inliers.swap(tmp_inliers);
        }

        // std::cout << "best inlier_count:" << best_inlier_count << std::endl;
        // return best F
        cv::Mat(bestF).copyTo(F21);
        return F21;
    }


    void Triangulate(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
    {
        cv::Mat A(4,4,CV_32F);

        A.row(0) = pt1.x*P1.row(2)-P1.row(0);
        A.row(1) = pt1.y*P1.row(2)-P1.row(1);
        A.row(2) = pt2.x*P2.row(2)-P2.row(0);
        A.row(3) = pt2.y*P2.row(2)-P2.row(1);

        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }

    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::Point2f> &vPts1, const vector<cv::Point2f> &vPts2,
                vector<bool> &vbMatchesInliers,
                const cv::Mat &K, float th2, float &parallax)
    {
        // Calibration parameters
        const float fx = K.at<float>(0,0);
        const float fy = K.at<float>(1,1);
        const float cx = K.at<float>(0,2);
        const float cy = K.at<float>(1,2);

        vector<float> vCosParallax;
        vCosParallax.reserve(vPts1.size());

        // Camera 1 Projection Matrix K[I|0]
        cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
        K.copyTo(P1.rowRange(0,3).colRange(0,3));

        cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3,4,CV_32F);
        R.copyTo(P2.rowRange(0,3).colRange(0,3));
        t.copyTo(P2.rowRange(0,3).col(3));
        P2 = K*P2;

        cv::Mat O2 = -R.t()*t;

        int nGood=0;

        size_t iend = vPts1.size();
        for(size_t i=0;i<iend;i++)
        {
            if(!vbMatchesInliers[i])
                continue;

            const cv::Point2f &pt1 = vPts1[i];
            const cv::Point2f &pt2 = vPts2[i];
            cv::Mat p3dC1;

            Triangulate(pt1,pt2,P1,P2,p3dC1);

            if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
            {
                continue;
            }

            // Check parallax
            cv::Mat normal1 = p3dC1 - O1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = p3dC1 - O2;
            float dist2 = cv::norm(normal2);

            float cosParallax = normal1.dot(normal2)/(dist1*dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            cv::Mat p3dC2 = R*p3dC1+t;

            if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
                continue;

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1.at<float>(2);
            im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
            im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

            float squareError1 = (im1x-pt1.x)*(im1x-pt1.x)+(im1y-pt1.y)*(im1y-pt1.y);

            if(squareError1>th2)
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2.at<float>(2);
            im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
            im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

            float squareError2 = (im2x-pt2.x)*(im2x-pt2.x)+(im2y-pt2.y)*(im2y-pt2.y);

            if(squareError2>th2)
                continue;

            vCosParallax.push_back(cosParallax);
            nGood++;
        }

        if(nGood>0)
        {
            sort(vCosParallax.begin(),vCosParallax.end());

            size_t idx = min(50,int(vCosParallax.size()-1));
            parallax = acos(vCosParallax[idx])*180/CV_PI;
        }
        else
            parallax=0;

        return nGood;
    }

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
    {
        cv::Mat u,w,vt;
        cv::SVD::compute(E,w,u,vt);

        u.col(2).copyTo(t);
        t=t/cv::norm(t);

        cv::Mat W(3,3,CV_32F,cv::Scalar(0));
        W.at<float>(0,1)=-1;
        W.at<float>(1,0)=1;
        W.at<float>(2,2)=1;

        R1 = u*W*vt;
        if(cv::determinant(R1)<0)
            R1=-R1;

        R2 = u*W.t()*vt;
        if(cv::determinant(R2)<0)
            R2=-R2;
    }

    bool ReconstructF(const vector<cv::Point2f>& vPts1, const vector<cv::Point2f>& vPts2,
                      vector<bool> &vbMatchesInliers,
                      cv::Mat &F21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                      float minParallax, int minTriangulated, float sigma)
    {
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;
        // Compute Essential Matrix from Fundamental Matrix
        cv::Mat E21 = K.t()*F21*K;
#if 0
        cv::Mat R1, R2, t;

        // Recover the 4 motion hypotheses
        DecomposeE(E21,R1,R2,t);
#else
        cv::Mat u, w, vt;
        cv::SVD::compute(E21, w, u, vt);
        cv::Mat t;
        u.col(2).copyTo(t);
        t=t/cv::norm(t);
        cv::Mat tmp_W = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        cv::Mat W;
        tmp_W.convertTo(W, E21.type());
        cv::Mat R1 = u * W * vt;
        if (cv::determinant(R1) < 0) R1 = -R1;
        cv::Mat R2 = u * W.t() * vt;
        if (cv::determinant(R2) < 0) R2 = -R2;
#endif
        cv::Mat t1=t;
        cv::Mat t2=-t;

        // Reconstruct with the 4 hyphoteses and check
        vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
        float parallax1,parallax2, parallax3, parallax4;

        double sigma2 = sigma * sigma;
        int nGood1 = CheckRT(R1,t1,vPts1,vPts2,vbMatchesInliers,K, 4.0*sigma2, parallax1);
        int nGood2 = CheckRT(R2,t1,vPts1,vPts2,vbMatchesInliers,K, 4.0*sigma2, parallax2);
        int nGood3 = CheckRT(R1,t2,vPts1,vPts2,vbMatchesInliers,K, 4.0*sigma2, parallax3);
        int nGood4 = CheckRT(R2,t2,vPts1,vPts2,vbMatchesInliers,K, 4.0*sigma2, parallax4);

        int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

        R21 = cv::Mat();
        t21 = cv::Mat();

        int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

        int nsimilar = 0;
        if(nGood1>0.7*maxGood)
            nsimilar++;
        if(nGood2>0.7*maxGood)
            nsimilar++;
        if(nGood3>0.7*maxGood)
            nsimilar++;
        if(nGood4>0.7*maxGood)
            nsimilar++;

        // If there is not a clear winner or not enough triangulated points reject initialization
        if(maxGood<nMinGood || nsimilar>1)
        {
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if(maxGood==nGood1)
        {
            if(parallax1>minParallax)
            {
                R1.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood2)
        {
            if(parallax2>minParallax)
            {
                R2.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood3)
        {
            if(parallax3>minParallax)
            {
                R1.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood4)
        {
            if(parallax4>minParallax)
            {
                R2.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }

        return false;
    }
}
