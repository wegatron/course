#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#include <vector>
#include "shared.h"
#include <pcl/common/copy_point.h>

void set_sample_p(std::vector<gr::Point3D> * sample_p);

void set_sample_q(std::vector<gr::Point3D> * sample_q);

std::vector<gr::Point3D> * sample_p();

std::vector<gr::Point3D> * sample_q();

void set_base_4pcs(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> & pcs);

void set_congruent_4pcs(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> & pcs);

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> base_4pcs();

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> congruent_4pcs();

int cur_id();

void set_cur_id(int id);

void set_cur_pos(const Eigen::Vector3f &pos);

Eigen::Vector3f cur_pos();

void addTransform(const Eigen::Affine3f &rt);

const std::vector<Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1>>>& getTransforms();

void set_ground_truth(Eigen::Affine3f &rt);

const Eigen::Affine3f& ground_truth();

#endif //DATA_INTERFACE_H