#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

// this program shows how to use optical flow

string file_1 = "../1.png";  // first image
string file_2 = "../2.png";  // second image

// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
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

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    vector<KeyPoint> kp2_single;
    vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single, false);

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, false);

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));

    // plot the differences of those functions
    Mat img2_single;
    cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++) {
        if (success_single[i]) {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    // compute gradient using opencv
    cv::Mat img1_gx, img1_gy, img2_gx, img2_gy;
    cv::spatialGradient(img1, img1_gx, img1_gy);
    cv::spatialGradient(img2, img2_gx, img2_gy);

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        std::vector<Eigen::Vector2d> Js;
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            int ind = 0;
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)
                    int cur_x = kp.pt.x+x;
                    int cur_y = kp.pt.y+y;
                    double error = GetPixelValue<uchar>(img1, cur_x, cur_y) - GetPixelValue<uchar>(img2, cur_x+dx, cur_y+dy);
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                      // Forward Jacobian
                      J[0] = -0.125*GetPixelValue<int16_t>(img2_gx, cur_x+dx, cur_y+dy);
                      J[1] = -0.125*GetPixelValue<int16_t>(img2_gy, cur_x+dx, cur_y+dy);
                      // J[0] = 0.5*(GetPixelValue<uchar>(img2, cur_x+dx+1, cur_y+dy) - GetPixelValue<uchar>(img2, cur_x+dx-1, cur_y+dy));
                      // J[1] = 0.5*(GetPixelValue<uchar>(img2, cur_x+dx, cur_y+dy+1) - GetPixelValue<uchar>(img2, cur_x+dx, cur_y+dy-1));
                    } else if(iter == 0){
                      // Inverse Jacobian
                      // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                      J[0] = -0.125*GetPixelValue<int16_t>(img1_gx, cur_x, cur_y);
                      J[1] = -0.125*GetPixelValue<int16_t>(img1_gy, cur_x, cur_y);
                      // J[0] = GetPixelValue<uchar>(img1, cur_x+0.5, cur_y) - GetPixelValue<uchar>(img1, cur_x-0.5, cur_y);
                      // J[1] = GetPixelValue<uchar>(img1, cur_x, cur_y+0.5) - GetPixelValue<uchar>(img1, cur_x, cur_y-0.5);
                      Js.emplace_back(J); // store J
                    }

                    // compute H, b and set cost;
                    if(!inverse || iter == 0)
                      H += J * J.transpose();
                    b -= J * error;
                    cost += error * error;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }

            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;

            if(update.norm() < 0.01) break;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    Mat ds_1 = img1;
    Mat ds_2 = img2;
    for (int i = 0; i < pyramids; i++) {
      pyr1.emplace_back(ds_1);
      pyr2.emplace_back(ds_2);
      if(i == pyramids-1) break;
      Mat tmp1, tmp2;
      cv::pyrDown(ds_1, tmp1, cv::Size(ds_1.cols/2, ds_1.rows/2));
      cv::pyrDown(ds_2, tmp2, cv::Size(ds_2.cols/2, ds_2.rows/2));
      ds_1 = tmp1; ds_2 = tmp2;
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE
    vector<KeyPoint> mkp1(kp1.size());
    vector<KeyPoint> mkp2;
    for(int i=0; i<pyramids; ++i) {
      for(int j=0; j<kp1.size(); ++j) {
        mkp1[j].pt = scales[3-i] * kp1[j].pt;
      }
      OpticalFlowSingleLevel(pyr1[3-i], pyr2[3-i], mkp1, mkp2, success, inverse);
      if(i == pyramids-1) break;
      for(auto &kp : mkp2) {
        kp.pt *= 2;
      }
    }
    kp2 = mkp2;
    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}
