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
          worest_radius(std::numeric_limits<float>::infinity()),
          worest_radius2(std::numeric_limits<float>::infinity()){}

    void add_point(size_t ind, float dis)
    {
        res.push(std::make_pair(dis, ind));
        if(res.size() > capacity) {
            res.pop();
            worest_radius = res.top().first;
            worest_radius2 = worest_radius * worest_radius;
        }
    }

    size_t capacity;
    float worest_radius;
    float worest_radius2;
    Eigen::Vector3f query_pt;
    std::priority_queue<std::pair<float, size_t>> res; //!< max heap by first element
};

struct result_set_radius
{
    result_set_radius(float r, const Eigen::Vector3f &qpt)
        :radius(r), radius2(r*r), query_pt(qpt) {}

    void add_point(size_t ind, float dis)
    {
        res.emplace_back(dis, ind);
    }
    float radius;
    float radius2;
    Eigen::Vector3f query_pt;
    std::vector<std::pair<float, size_t>> res;
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
    bool overlap(const Eigen::Vector3f &query_pt, const float radius2) const
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

        return sq_dis < radius2;
    }

    /**
     * @brief 子树被搜索区域包含
     */
    bool contains(const Eigen::Vector3f &query_pt, const float raidus) const
    {
        Eigen::Vector3f diff(fabs(query_pt[0] - center[0]) + extent,
                             fabs(query_pt[1] - center[1]) + extent,
                             fabs(query_pt[2] - center[2]) + extent);
        return raidus*raidus > diff.squaredNorm();
    }

    // for debug
    void validate(float * const pts) const;
};

class octree
{
public:
    octree(const np::ndarray &db_npt, int leaf_size, float min_extent);
    boost::python::tuple search_knn(const np::ndarray &query_pt, int k);
    boost::python::tuple search_radius(const np::ndarray &query_pt, float radius);
private:
    void build_sub_tree(std::list<octant*> &job);

    int leaf_size_;
    float min_extent_;
    std::shared_ptr<octant> root_; //!< using shared_ptr, for assignment
    std::vector<float> pts_data_; //!< points
    std::vector<size_t> map_ori_inds_; //!< map to index in origional order
};

#endif // WEGATRON_OCTREE_H
