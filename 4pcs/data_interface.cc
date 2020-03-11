#include "data_interface.h"

static std::vector<gr::Point3D> * sample_p_ = nullptr;
static std::vector<gr::Point3D> * sample_q_ = nullptr;
static std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> base_4pcs_;
static std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> congruent_4pcs_;
static int cur_id_ = 0;

void set_sample_p(std::vector<gr::Point3D>* sample_p)
{
	sample_p_ = sample_p;
}

void set_sample_q(std::vector<gr::Point3D>* sample_q)
{
	sample_q_ = sample_q;
}

std::vector<gr::Point3D>* sample_p()
{
	return sample_p_;
}

std::vector<gr::Point3D>* sample_q()
{
	return sample_q_;
}

void set_base_4pcs(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pcs)
{
	base_4pcs_ = pcs;
}

void set_congruent_4pcs(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pcs)
{
	congruent_4pcs_ = pcs;
}

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> base_4pcs()
{
	return base_4pcs_;
}

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> congruent_4pcs()
{
	return congruent_4pcs_;
}

int cur_id()
{
	return cur_id_;
}

void set_cur_id(int id)
{
	cur_id_ = id;
}

static Eigen::Vector3f cur_pos_;

void set_cur_pos(const Eigen::Vector3f& pos)
{
	cur_pos_ = pos;
}

Eigen::Vector3f cur_pos()
{
	return cur_pos_;
}


static std::vector<Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1>>> rts;
void addTransform(const Eigen::Affine3f& rt)
{
	Eigen::Matrix<float, 6, 1> pt;
	pt.block<3,1>(0,0) = rt.translation();
	Eigen::AngleAxisf aa(rt.rotation());
	pt.block<3,1>(3,0) = aa.axis() * aa.angle();
	rts.emplace_back(pt);
}

const std::vector<Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1>>>& getTransforms()
{
	return rts;
}

static Eigen::Affine3f ground_truth_rt_;

void set_ground_truth(Eigen::Affine3f& rt)
{
	ground_truth_rt_ = rt;
}

const Eigen::Affine3f& ground_truth()
{
	return ground_truth_rt_;
}


