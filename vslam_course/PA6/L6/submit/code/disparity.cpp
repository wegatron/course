//
// Created by 高翔 on 2017/12/15.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// 文件路径，如果不对，请调整
string left_file = "../left.png";
string right_file = "../right.png";
string disparity_file = "../disparity.png";

// 在panglin中画图，已写好，无需调整
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud, int pivot);

int main(int argc, char **argv) {

    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 间距
    double d = 0.573;

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Mat disparity = cv::imread(disparity_file, 0); // disparty 为CV_8U,单位为像素

    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // TODO 根据双目模型计算点云
    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {

            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // start your code here (~6 lines)
            // 根据双目模型计算 point 的位置
            double nxl = (u - cx)/fx;
            int ur = u - disparity.at<uchar>(v, u);
            if(ur < 0) continue;
            double nxr = (ur - cx)/fx;
            double z = d/(nxl - nxr);
            double ny = (v-cy)/fy;
            pointcloud.emplace_back(nxl*z, ny*z, z, left.at<uchar>(v, u) / 255.0);
            // end your code here
        }

    // calculate disparity by optical flow
    // key points, using GFTT here.
    vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(left, kp1);
    // then test multi-level LK
    vector<cv::Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(left, right, pt1, pt2, status, error, cv::Size(8, 8));
    cv::Mat diff(disparity.rows, disparity.cols, CV_16U, cv::Scalar_<uchar>(128));
    double sum_diff = 0;
    int valid_n = 0;
    int pivot = pointcloud.size();
    //vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud2;
    for(int i=0; i<status.size(); ++i)
    {
        if(!status[i]) continue;
        int dis = pt1[i].x - pt2[i].x;
        if(dis < 0) continue;
        double diff_val = std::abs(disparity.at<uchar>(pt1[i].y, pt1[i].x) - dis);
        sum_diff += diff_val; ++valid_n;
        double z = d*fx / dis;
        //pointcloud2.emplace_back(z*(pt1[i].x-cx)/fx, z*(pt1[i].y-cy)/fy, z, left.at<uchar>(pt1[i].y, pt1[i].x)/255.0);
        pointcloud.emplace_back(z*(pt1[i].x-cx)/fx, z*(pt1[i].y-cy)/fy, z, 255.0);
        if(diff_val < 3) diff.at<uchar>(pt1[i].y, pt1[i].x) = 255;
        else diff.at<uchar>(pt1[i].y, pt1[i].x) = 0;
    }
    std::cout << "average diff pix:" << sum_diff/valid_n << std::endl;
    std::cout << "average diff depth:" << d*fx/(sum_diff/valid_n) << std::endl;
    //cv::imshow("diff", diff);
    //cv::waitKey(0);
    // 画出点云
    showPointCloud(pointcloud, pivot);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud, int pivot) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for(int i=0; i<pivot; ++i) {
            const auto &p = pointcloud[i];
            glPointSize(1.0f);
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }

        for(int i=pivot; i<pointcloud.size(); ++i) {
            const auto &p = pointcloud[i];
            glPointSize(4.0f);
            glColor3f(1.0, 0, 0);
            glVertex3d(p[0], p[1], p[2]);
        }

        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
