#include "octree.h"
#include <vector>
#include <list>

octree::octree(const np::ndarray &db_npt, int leaf_size, float min_extent)
    :leaf_size_(leaf_size), min_extent_(min_extent), root_(nullptr)
{
    const size_t pt_num = db_npt.shape(0);
    std::cout << "!!!point num=" << pt_num << " leaf_size=" << leaf_size << " min_extent=" << min_extent << std::endl;
    pts_data_.resize(pt_num*3);
    memcpy(pts_data_.data(), db_npt.get_data(), 3*pt_num*sizeof(float));
    //reinterpret_cast<float*>(db_npt.get_data())

    map_ori_inds_.resize(pt_num);
    for(size_t i=0; i<pt_num; ++i) map_ori_inds_[i] = i;



    float * const root_data_ptr = pts_data_.data();
    Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts(root_data_ptr, 3, pt_num);
    auto rw = pts.rowwise();
    Eigen::Vector3f max_pt = rw.maxCoeff();
    Eigen::Vector3f min_pt = rw.minCoeff();

//    std::cout << "pts in c++:" << std::endl;
//    std::cout << pts << std::endl;
//    std::cout << "min_pt=" << min_pt.transpose() << std::endl;
//    std::cout << "max_pt=" << max_pt.transpose() << std::endl;

    std::unique_ptr<octant> tmp_root(new octant);
    tmp_root->l_ind = 0; tmp_root->r_ind = pt_num;
    tmp_root->center = 0.5*(max_pt + min_pt);
    tmp_root->extent = (max_pt - min_pt).norm() * 0.5f;
    tmp_root->is_leaf = tmp_root->extent < min_extent_ || pt_num < leaf_size_;
    if(tmp_root->is_leaf) { root_ = std::move(tmp_root); return; }

    std::list<octant*> jobs_queue;
    jobs_queue.emplace_back(tmp_root.get());
    while(!jobs_queue.empty())
    {
        build_sub_tree(jobs_queue);
    }

    root_ = std::move(tmp_root);
}

void octree::build_sub_tree(std::list<octant*> &job_queue)
{
    const auto sub_root = job_queue.front(); job_queue.pop_front();
    sub_root->validate(pts_data_.data());
    const int pt_num = sub_root->r_ind - sub_root->l_ind;
    assert(pt_num >0);

    sub_root->is_leaf = sub_root->extent < min_extent_ || pt_num < leaf_size_;
    if(sub_root->is_leaf) return;

    // split for children rerange pts
    std::vector<float> buffers[8];
    for(int i=0; i<8; ++i) buffers[i].reserve(pt_num*3);

    std::vector<size_t> map_inds_buffer[8];
    for(int i=0; i<8; ++i) map_inds_buffer[i].reserve(pt_num);

    float * const root_data_ptr = pts_data_.data() + sub_root->l_ind*3;
    float * ptr = root_data_ptr;
    for(int i=sub_root->l_ind; i<sub_root->r_ind; ++i)
    {
        uint8_t code = 0;
        if(ptr[0] > sub_root->center[0]) code |= 1;
        if(ptr[1] > sub_root->center[1]) code |= 2;
        if(ptr[2] > sub_root->center[2]) code |= 4;
        buffers[code].emplace_back(ptr[0]);
        buffers[code].emplace_back(ptr[1]);
        buffers[code].emplace_back(ptr[2]);
        map_inds_buffer[code].emplace_back(map_ori_inds_[i]); // 在原始数据上的index
        ptr += 3;
    }

    ptr = root_data_ptr;
    // Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts(root_data_ptr, 3, pt_num);
    // std::cout << "points' check before:\n" << pts << std::endl;
    size_t * inds_ptr = map_ori_inds_.data() + sub_root->l_ind;
    for(int i=0; i<8; ++i) {
        if(buffers[i].size() == 0) continue;
        memcpy(ptr, buffers[i].data(), sizeof(float)*buffers[i].size());
        ptr += buffers[i].size();

        memcpy(inds_ptr, map_inds_buffer[i].data(), sizeof(size_t)*map_inds_buffer[i].size());
        inds_ptr += map_inds_buffer[i].size();
    }

    // check
    //std::cout << "point's check:\n" << pts << std::endl;

    // add sub tree jobs
    int offset = sub_root->l_ind;
    for(int i=0; i<8; ++i) {
        if(buffers[i].size() == 0) continue;
        auto child = new octant;
        child->extent = sub_root->extent * 0.5f;
        child->center[0] = i&1 ? sub_root->center[0] + child->extent : sub_root->center[0] - child->extent;
        child->center[1] = i&2 ? sub_root->center[1] + child->extent : sub_root->center[1] - child->extent;
        child->center[2] = i&4 ? sub_root->center[2] + child->extent : sub_root->center[2] - child->extent;

        child->l_ind = offset;
        child->r_ind = child->l_ind + buffers[i].size()/3;
        offset = child->r_ind;
        child->validate(pts_data_.data());
        job_queue.emplace_back(child);
        sub_root->children[i].reset(child);
    }
}

boost::python::tuple octree::search_knn(const np::ndarray &query_pt, int k)
{
    float * qpt = reinterpret_cast<float*>(query_pt.get_data());
    std::cout << "qpt in c++:" << qpt[0] << " " << qpt[1] << " " << qpt[2] << std::endl;
    result_set_knn res_knn(k, Eigen::Vector3f(qpt[0], qpt[1], qpt[2]));
    std::list<const octant*> job_queue;
    job_queue.emplace_back(root_.get());
    while(!job_queue.empty()) {
        const octant * sub_root = job_queue.back(); job_queue.pop_back();
        // TODO remove for debug
        sub_root->validate(pts_data_.data());
        if(sub_root == nullptr /*|| !sub_root->overlap(res_knn.query_pt, res_knn.worest_radius)*/) continue;
        // 若radius < worest_radius的点都在当前子树内, 那么只要搜索当前子树即可, 将其他子树的搜索任务清除
        if(sub_root->inside(res_knn.query_pt, res_knn.worest_radius)) job_queue.clear();
        if(sub_root->is_leaf)
        {
            float * pts = pts_data_.data() + sub_root->l_ind * 3;
            for(int i=sub_root->l_ind; i<sub_root->r_ind; ++i)
            {
                Eigen::Vector3f tmp_pt(pts);
                res_knn.add_point(i, (tmp_pt-res_knn.query_pt).norm());
                pts += 3;
            }
            continue;
        }

        // search children
        unsigned char code = 0;
        if(res_knn.query_pt[0] > sub_root->center[0]) code |= 1;
        if(res_knn.query_pt[1] > sub_root->center[1]) code |= 2;
        if(res_knn.query_pt[2] > sub_root->center[2]) code |= 4;

        for(int i=0; i<8; ++i) {
            if(i == code || sub_root->children[i] == nullptr) continue;
            job_queue.emplace_back(sub_root->children[i].get());
        }
        if(sub_root->children[code] != nullptr) job_queue.emplace_back(sub_root->children[code].get());
    }

    // transform result output
    np::ndarray dist = np::empty(boost::python::make_tuple(k), np::dtype::get_builtin<float>());
    np::ndarray inds = np::empty(boost::python::make_tuple(k), np::dtype::get_builtin<int>());
    float * dist_data = reinterpret_cast<float*>(dist.get_data());
    int * inds_data = reinterpret_cast<int*>(inds.get_data());
    for(int i=0; i<k; ++i) {
        auto info = res_knn.res.top();
        res_knn.res.pop();
        dist_data[k-i-1] = info.first;
        inds_data[k-i-1] = map_ori_inds_[info.second];
    }
    return make_tuple(inds, dist);
}

void octant::validate(float * const pts_data) const
{
    float * pts = pts_data + l_ind * 3;
    for(int i=l_ind; i<r_ind; ++i)
    {
        Eigen::Vector3f tmp_pt(pts);
        assert(pts[0] <= center[0] + extent + 1e-3);
        assert(pts[1] <= center[1] + extent + 1e-3);
        assert(pts[2] <= center[2] + extent + 1e-3);

        assert(pts[0] + 1e-3 >= center[0] - extent);
        assert(pts[1] + 1e-3 >= center[1] - extent);
        assert(pts[2] + 1e-3 >= center[2] - extent);
        pts += 3;
    }
}


BOOST_PYTHON_MODULE(wegatron_octree) {
    using namespace boost::python;
    np::initialize(); // must initialize before using numpy
    class_<octree>("octree",
        init<const boost::python::numpy::ndarray &, int, double>())
        .def("search_knn", &octree::search_knn)
        .def("do_nothing", &octree::do_nothint);
}
