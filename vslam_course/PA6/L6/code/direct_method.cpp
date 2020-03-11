#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
boost::format fmt_others("../%06d.png");    // other files

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21
);

// bilinear interpolation
template<typename T>
double GetPixelValue(const cv::Mat &img, float x, float y) {
    T *data = reinterpret_cast<T*>(&img.data[int(y) * img.step[0] + int(x)*img.step[1]]);
    float xx = x - floor(x);
    float yy = y - floor(y);
    return double(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv) {

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;

    for (int i = 1; i < 6; i++) {  // 1~10
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
        //DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
        std::cout << "to img " << i << " res:\n" << std::endl;
        std::cout << T_cur_ref.matrix() << std::endl;
    }
}

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;
    VecVector2d goodRef;

    Eigen::Matrix3d K;
    K << fx, 0, cx,
      0, fy, cy,
      0, 0, 1;
    Eigen::Matrix3d invK = K.inverse();

    cv::Mat img2_gx, img2_gy;
    cv::spatialGradient(img2, img2_gx, img2_gy);
    img2_gx = 0.125 * img2_gx;
    img2_gy = 0.125 * img2_gy;
    for (int iter = 0; iter < iterations; iter++) {
        cost = 0;
        nGood = 0;
        goodProjection.clear();
        goodRef.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {
            // compute the projection in the second image
            // unproject point
            Eigen::Vector3d pt_3d = depth_ref[i] * invK * Eigen::Vector3d(px_ref[i].x(), px_ref[i].y(), 1);
            pt_3d = T21 * pt_3d; // transformed
            if(pt_3d.z() < 0) continue;
            const double z_inv = 1.0/pt_3d[2];
            Eigen::Vector3d uv = z_inv * K * pt_3d; // project back to img2
            // check if good
            if(uv[0]<half_patch_size || uv[0]+half_patch_size >= img2.cols
               || uv[1]<half_patch_size || uv[1]+half_patch_size >= img2.rows) continue;

            nGood++;
            goodProjection.emplace_back(uv[0], uv[1]);
            goodRef.emplace_back(px_ref[i]);
            Eigen::Matrix<double, 2, 3> J_uq;
            J_uq << fx*z_inv, 0 , -z_inv*z_inv*fx*pt_3d[0],
              0, fy*z_inv, -z_inv*z_inv*fy*pt_3d[1];
            Eigen::Matrix<double, 3, 6> J_qxi;
            J_qxi.block<3,3>(0,0).setIdentity();
            J_qxi.block<3,3>(0,3) = -Sophus::SO3d::hat(pt_3d);
            Matrix26d J_uxi = J_uq * J_qxi;
            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    Eigen::Vector2d ref_coord(px_ref[i].x()+x, px_ref[i].y()+y);
                    Eigen::Vector2d re_proj_coord(uv.x()+x, uv.y()+y);
                    double error = GetPixelValue<uchar>(img1, ref_coord[0], ref_coord[1])
                      - GetPixelValue<uchar>(img2, re_proj_coord[0], re_proj_coord[1]);
                    Eigen::Vector2d J_img2(GetPixelValue<int16_t>(img2_gx, re_proj_coord[0], re_proj_coord[1]),
                                           GetPixelValue<int16_t>(img2_gy, re_proj_coord[0], re_proj_coord[1]));
                    // total jacobian
                    Vector6d J = -J_img2.transpose() * J_uxi;
                    H += J * J.transpose();
                    b -= error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        cost /= nGood;
        Vector6d update = H.ldlt().solve(b);

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        T21 = Sophus::SE3d::exp(update) * T21;
        cout << "cost = " << cost << ", good = " << nGood << endl;
        if(update.norm() < 1e-3) break;
    }
    cout << "good projection: " << nGood << endl;
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    int cnt = 0;
    for (auto &px: goodRef) {
      if(++cnt % 10 != 0) continue;
      if(px[0] < 2 || px[0]+2 >=img1_show.cols || px[1]<2 || px[1]>=img1_show.rows) continue;
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, cnt%255, 255-(cnt%255)));
    }

    cnt = 0;
    for (auto &px: goodProjection) {
      if(++cnt % 10 != 0) continue;
      if(px[0] < 2 || px[0]+2 >=img1_show.cols || px[1]<2 || px[1]>=img1_show.rows) continue;
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, cnt%255, 255-(cnt%255)));
    }
   cv::imshow("reference", img1_show);
   cv::imshow("current", img2_show);
   cv::waitKey();
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
    cv::Mat dst1 = img1;
    cv::Mat dst2 = img2;
    for(int i=0; i<pyramids; ++i) {
        pyr1.emplace_back(dst1);
        pyr2.emplace_back(dst2);
        if(i == pyramids-1) break;
        cv::Mat tmp1, tmp2;
        cv::pyrDown(dst1, tmp1, cv::Size(dst1.cols/2, dst1.rows/2));
        cv::pyrDown(dst2, tmp2, cv::Size(dst2.cols/2, dst2.rows/2));
        dst1 = tmp1; dst2 = tmp2;
    }
    // END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
        fx = scales[level] * fxG;
        fy = scales[level] * fyG;
        cx = scales[level] * cxG;
        cy = scales[level] * cyG;
        // END YOUR CODE HERE
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }

}
