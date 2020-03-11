#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <thread>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "../compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses,
                    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &poses_e);

int main(int argc, char **argv) {
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> gposes;

    std::ifstream ifs(trajectory_file);
    double value[8];
    while(!ifs.eof())
    {
      ifs >> value[0]  >> value[1] >> value[2] >> value[3] >> value[4] >> value[5] >> value[6] >> value[7];
      poses.emplace_back(Eigen::Quaterniond(value[4], value[5], value[6], value[7]),
                           Eigen::Vector3d(value[1], value[2], value[3]));
      ifs >> value[0]  >> value[1] >> value[2] >> value[3] >> value[4] >> value[5] >> value[6] >> value[7];
      gposes.emplace_back(Eigen::Quaterniond(value[4], value[5], value[6], value[7]),
                           Eigen::Vector3d(value[1], value[2], value[3]));
    }

    // ICP
    const int n = poses.size();
    Eigen::Vector3d cp = Eigen::Vector3d::Zero();
    Eigen::Vector3d cp_g = Eigen::Vector3d::Zero();
    for(int i=0; i<n; ++i) {
      cp += poses[i].translation();
      cp_g += gposes[i].translation();
    }

    cp = 1.0/n * cp;
    cp_g = 1.0/n * cp_g;

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i=0; i<n; ++i) {
      Eigen::Vector3d q_t = gposes[i].translation() - cp_g;
      Eigen::Vector3d q = poses[i].translation() - cp;
      W += q_t * q.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * (svd.matrixV().transpose());
    if(R.determinant() < 0) R = -R;

    Eigen::Vector3d t = cp_g - R * cp;

    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> poses_e;
    for(int i=0; i<n; ++i) {
      poses_e.emplace_back(R * poses[i].translation() + t);
    }

    // draw trajectory in pangolin
    DrawTrajectory(gposes, poses_e);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses,
                    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &poses_e) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            glVertex3d(poses_e[i][0], poses_e[i][1], poses_e[i][2]);
            glVertex3d(poses_e[i+1][0], poses_e[i+1][1], poses_e[i+1][2]);
            glEnd();
        }
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //usleep(5000);   // sleep 5 ms
    }

}
