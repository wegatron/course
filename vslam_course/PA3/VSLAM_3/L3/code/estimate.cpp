#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
    std::ifstream estimate_file("./estimated.txt");
    std::ifstream ground_truth_file("./groundtruth.txt");

    double value[8];
    double diff_sum = 0.0;
    int cnt = 0;
    while(!estimate_file.eof())
    {
        estimate_file >> value[0]  >> value[1] >> value[2] >> value[3] >> value[4] >> value[5] >> value[6] >> value[7];
        Sophus::SE3d estimate_pos(Eigen::Quaterniond(value[4], value[5], value[6], value[7]),
                           Eigen::Vector3d(value[1], value[2], value[3]));

        ground_truth_file >> value[0]  >> value[1] >> value[2] >> value[3] >> value[4] >> value[5] >> value[6] >> value[7];
        Sophus::SE3d gth_pos(Eigen::Quaterniond(value[4], value[5], value[6], value[7]),
                                  Eigen::Vector3d(value[1], value[2], value[3]));

        //std::cout << estimate_pos.matrix() << std::endl;
        //std::cout << gth_pos.matrix() << std::endl;
        Sophus::SE3d diff(estimate_pos.matrix().inverse() * gth_pos.matrix());

        auto diff_v = diff.log();
        diff_sum += diff_v.squaredNorm();
        ++cnt;
    }
    std::cout << sqrt(diff_sum / cnt) << std::endl;
    return 0;
}