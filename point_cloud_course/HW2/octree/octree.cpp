#include "octree.h"
#include <vector>
#include <list>

octree::octree(np::ndarray &db_npt, int leaf_size, float min_extent)
    :leaf_size_(leaf_size), min_extent_(min_extent), root_(nullptr),
      pts_data_(reinterpret_cast<float*>(db_npt.get_data())) // 这里db_npt的数据已经被拷贝一份了
{
    const size_t pt_num = db_npt.shape(1);
    std::cout << "!!!point num=" << pt_num << " leaf_size=" << leaf_size << " min_extent=" << min_extent << std::endl;

    map_ori_inds_.resize(pt_num);
    for(size_t i=0; i<pt_num; ++i) map_ori_inds_[i] = i;

    std::unique_ptr<octant> tmp_root(new octant);
    tmp_root->l_ind = 0; tmp_root->r_ind = pt_num;
    std::list<octant*> jobs_queue;
    jobs_queue.emplace_back(tmp_root);
    while(!jobs_queue.empty())
    {
        build_sub_tree(jobs_queue);
        jobs_queue.pop_front();
    }

    root_ = std::move(tmp_root);
}

void octree::build_sub_tree(std::list<octant*> &job_queue)
{
    const auto sub_root = job_queue.front();
    const int pt_num = sub_root->r_ind - sub_root->l_ind;
    assert(pt_num >0);
    // calculate center, extent
    float * const root_data_ptr = pts_data_ + sub_root->l_ind*3;
    Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts(root_data_ptr, 3, pt_num);
    auto rw = pts.rowwise();
    Eigen::Vector3f max_pt = rw.maxCoeff();
    Eigen::Vector3f min_pt = rw.minCoeff();
    sub_root->center = 0.5*(max_pt + min_pt);
    sub_root->extent = (max_pt - min_pt).norm() * 0.5f;
    sub_root->is_leaf = sub_root->extent < min_extent_ || pt_num < leaf_size_;
    if(sub_root->is_leaf) return;

    // split for children rerange pts
    std::vector<float> buffers[8];
    for(int i=0; i<8; ++i) buffers[i].reserve(pt_num*3);

    std::vector<size_t> map_inds_buffer[8];
    for(int i=0; i<8; ++i) map_inds_buffer[i].reserve(pt_num);

    float *ptr = root_data_ptr;
    uint8_t code = 0;
    for(int i=sub_root->l_ind; i<sub_root->r_ind; ++i)
    {
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
    size_t * inds_ptr = map_ori_inds_.data() + sub_root->l_ind;
    for(int i=0; i<8; ++i) {
        if(buffers[i].size() == 0) continue;
        memcpy(ptr, buffers[i].data(), sizeof(float)*buffers[i].size());
        ptr += buffers[i].size();

        memcpy(inds_ptr, map_inds_buffer[i].data(), sizeof(size_t)*map_inds_buffer[i].size());
        inds_ptr += map_inds_buffer[i].size();
    }

    // add sub tree jobs
    int offset = sub_root->l_ind;
    for(int i=0; i<8; ++i) {
        if(buffers[i].size() == 0) continue;
        auto child = new octant;
        child->l_ind = offset;
        child->r_ind = child->r_ind + buffers[i].size()/3;
        offset = child->r_ind;
        job_queue.emplace_back(child);
        sub_root->children[i].reset(child);
    }

    job_queue.pop_front();
}

boost::python::tuple octree::search_knn(np::ndarray &query_pt, int k)
{
    np::ndarray dist = np::empty(boost::python::make_tuple(6), np::dtype::get_builtin<float>());
    np::ndarray inds = np::empty(boost::python::make_tuple(6), np::dtype::get_builtin<int>());

    // TODO
    float * dist_data = reinterpret_cast<float*>(dist.get_data());
    int * inds_data = reinterpret_cast<int*>(inds.get_data());
    for(int i=0; i<6; ++i) {
        inds_data[i] = i;
        dist_data[i] = i * 10;
    }
    return make_tuple(inds, dist);
}


BOOST_PYTHON_MODULE(wegatron_octree) {
    using namespace boost::python;
    np::initialize(); // must initialize before using numpy
    class_<octree>("octree",
        init<const boost::python::numpy::ndarray &, int, double>())
        .def("search_knn", &octree::search_knn);
}
