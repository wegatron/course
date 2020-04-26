#ifndef WEGATRON_OCTREE_H
#define WEGATRON_OCTREE_H
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <list>

namespace np = boost::python::numpy;

struct octant
{
    float extent = 0;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    int l_ind = -1; //!< left index
    int r_ind = -1; //!< right index
    bool is_leaf = true;
    std::unique_ptr<octant> children[8] = {nullptr};
};

class octree
{
public:
    octree(np::ndarray db_npt, int leaf_size, float min_extent);
    boost::python::tuple search_knn(np::ndarray query_pt, int k);
private:
    void build_sub_tree(std::list<octant*> &job);

    int leaf_size_;
    float min_extent_;
    std::shared_ptr<octant> root_; //!< using shared_ptr, for assignment
    float * pts_data_; //!< points
    std::vector<size_t> map_ori_inds; //!< map to index in origional order
};

#endif // WEGATRON_OCTREE_H
