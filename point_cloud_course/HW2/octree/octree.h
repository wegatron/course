#ifndef WEGATRON_OCTREE_H
#define WEGATRON_OCTREE_H
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <list>
#include <queue>


namespace np = boost::python::numpy;

struct result_set_knn
{
    result_set_knn(size_t k, const Eigen::Vector3f &qpt)
        : capacity(k), query_pt(qpt),
          worest_radius(std::numeric_limits<float>::infinity()) {
    }

    void add_point(size_t ind, float dis)
    {
        res.push(std::make_pair(dis, ind));
        if(res.size() > capacity) {
            res.pop();
            worest_radius = res.top().first;
        }
    }

    size_t capacity;
    float worest_radius;
    Eigen::Vector3f query_pt;
    std::priority_queue<std::pair<float, size_t>> res; //!< max heap by first element
};

struct octant
{
    float extent = 0;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    int l_ind = -1; //!< left index
    int r_ind = -1; //!< right index
    bool is_leaf = true;
    std::unique_ptr<octant> children[8] = {nullptr};

    /**
     * @brief 是否在子树内
     */
    bool inside(const Eigen::Vector3f &query_pt, const float radius) const
    {
        return (center - query_pt).norm() + radius < extent;
    }

    /**
     * @brief 是否与子树有交集
     */
    bool overlap(const Eigen::Vector3f &query_pt, const float radius) const
    {
        float max_pt[3] = {center[0]+extent, center[1]+extent, center[2]+extent};
        float min_pt[3] = {center[0]-extent, center[1]-extent, center[2]-extent};
        float sq_dis = 0; // 最近点离查询点的距离平方和
        if(query_pt[0] > max_pt[0]) sq_dis += (query_pt[0] - max_pt[0])*(query_pt[0] - max_pt[0]);
        else if(query_pt[0] < min_pt[0]) sq_dis += (query_pt[0] - min_pt[0])*(query_pt[0] - min_pt[0]);

        if(query_pt[1] > max_pt[1]) sq_dis += (query_pt[1] - max_pt[1])*(query_pt[1] - max_pt[1]);
        else if(query_pt[1] < min_pt[1]) sq_dis += (query_pt[1] - min_pt[1])*(query_pt[1] - min_pt[1]);

        if(query_pt[2] > max_pt[2]) sq_dis += (query_pt[2] - max_pt[2])*(query_pt[2] - max_pt[2]);
        else if(query_pt[2] < min_pt[2]) sq_dis += (query_pt[2] - min_pt[2])*(query_pt[2] - min_pt[2]);

        return sq_dis < radius * radius;
    }

    // for debug
    void validate(float * const pts) const;
};

class octree
{
public:
    octree(const np::ndarray &db_npt, int leaf_size, float min_extent);
    boost::python::tuple search_knn(const np::ndarray &query_pt, int k);
    void do_nothint(const np::ndarray &qpt) {}
private:
    void build_sub_tree(std::list<octant*> &job);

    int leaf_size_;
    float min_extent_;
    std::shared_ptr<octant> root_; //!< using shared_ptr, for assignment
    std::vector<float> pts_data_; //!< points
    std::vector<size_t> map_ori_inds_; //!< map to index in origional order
};

#endif // WEGATRON_OCTREE_H
