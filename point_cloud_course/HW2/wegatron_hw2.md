# 第二次课作业
1. 补充了python代码
    __修改以下bug__
    * velodyne 读取点云维度转置问题
    ```python
    def read_velodyne_bin(path):
        ...
        return np.asarray(pc_list, dtype=np.float32)  # 这里修改了一下, 去掉多余转置
    ```
    * jiaxin提到的center的bug


    __加入可视化(从论坛中抄写的)__
    ```python
    visualize_result(db_np, knn_res_cpp, radius_nn_res_cpp)
    ```

    __加入了与bruteforce对比的check代码__
    ```python
    nn_ind = nn_result_set.nearest_nn_index()
    radius_ind = result_set.nearest_radius_index()
    # if not np.all(radius_ind == nn_idx[:result_set.count]):
    #     print('check radius nn False!')
    # if not np.all(nn_ind == nn_idx[:nn_result_set.count]):
    #     print('check knn False'   
    if not np.all(cpp_inds_radius == nn_idx[:result_set.count]):
        # 这里可能由于浮点数误差
        print('cpp check radius nn not eq')
        index = np.arange(0, cpp_inds_radius.shape[0])
        not_eq = index[cpp_inds_radius != nn_idx[:result_set.count]]
        print(not_eq)
        print(cpp_dist_radius[not_eq] - nn_dist[not_eq])
    if not np.all(cpp_inds_knn == nn_idx[:cpp_inds_knn.shape[0]]):
        print('cpp check knn False')
    ```

2. 实现了c++版本的octree, 并做了一些改进
    这里使用了boost::python将c++函数导出到python, 并传递numpy::array对象.
    改进点如下:
    * 使用任务队列, 去掉递归
    ```c++
    octree::octree(const np::ndarray &db_npt, int leaf_size, float min_extent)
    :leaf_size_(leaf_size), min_extent_(min_extent), root_(nullptr)
    {
        ...
        std::list<octant*> jobs_queue;
        jobs_queue.emplace_back(tmp_root.get());
        while(!jobs_queue.empty())
        {
            build_sub_tree(jobs_queue);
        }
    }

    void octree::build_sub_tree(std::list<octant*> &job_queue)
    {
        ...
        // add sub tree jobs
        int offset = sub_root->l_ind;
        for(int i=0; i<8; ++i) {
            if(buffers[i].size() == 0) continue;
            auto child = new octant;
            child->extent = sub_root->extent * 0.5f;
            child->center[0] = i&1 ? sub_root->center[0] + child->extent : sub_root->center [0] - child->extent;
            child->center[1] = i&2 ? sub_root->center[1] + child->extent : sub_root->center [1] - child->extent;
            child->center[2] = i&4 ? sub_root->center[2] + child->extent : sub_root->center [2] - child->extent;

            child->l_ind = offset;
            child->r_ind = child->l_ind + buffers[i].size()/3;
            offset = child->r_ind;
            //child->validate(pts_data_.data());
            job_queue.emplace_back(child);
            sub_root->children[i].reset(child);
        }
    }
    ```
    * 点云数据重新排列, 访问更加连续, 并且节点只需存index range
    ```c++
    void octree::build_sub_tree(std::list<octant*> &job_queue)
    {
        ...
        // split for children rerange pts
        std::vector<float> buffers[8];
        for(int i=0; i<8; ++i) buffers[i].reserve(pt_num*3);

        std::vector<size_t> map_inds_buffer[8];// 记录原始index
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
        // Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts(root_data_ptr, 3,    pt_num);
        // std::cout << "points' check before:\n" << pts << std::endl;
        size_t * inds_ptr = map_ori_inds_.data() + sub_root->l_ind;
        for(int i=0; i<8; ++i) {
            if(buffers[i].size() == 0) continue;
            memcpy(ptr, buffers[i].data(), sizeof(float)*buffers[i].size());
            ptr += buffers[i].size();

            memcpy(inds_ptr, map_inds_buffer[i].data(), sizeof(size_t)*map_inds_buffer[i].  size());
            inds_ptr += map_inds_buffer[i].size();
        }
        ...
    }
        
    ```
    * 使用优先级队列提高插入效率
    ```c++
    void add_point(size_t ind, float dis)
    {
        res.push(std::make_pair(dis, ind));
        if(res.size() > capacity) {
            res.pop();
            worest_radius = res.top().first;
            worest_radius2 = worest_radius * worest_radius;
        }
    }
    ```
    * 参考论坛简化overlap函数
    ```c++
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
    ```

3. 结果:
图中黑色点为knn搜索的50个点, 蓝色为radius=40搜索得到的点. bentch mark与brute force差两个数量级.
![result](result.png)
    ```
    octree --------------
    !!!point num=121015 leaf_size=200 min_extent=10
    !!!point num=121151 leaf_size=200 min_extent=10
    !!!point num=121483 leaf_size=200 min_extent=10
    !!!point num=121604 leaf_size=200 min_extent=10
    !!!point num=121779 leaf_size=200 min_extent=10
    !!!point num=121948 leaf_size=200 min_extent=10
    !!!point num=121848 leaf_size=200 min_extent=10
    !!!point num=121997 leaf_size=200 min_extent=10
    cpp check radius nn not eq
    [2307 2308]
    [0. 0.]
    !!!point num=122227 leaf_size=200 min_extent=10
    !!!point num=122402 leaf_size=200 min_extent=10
    cpp check radius nn not eq
    [178 179 448 449]
    [0.0000000e+00 0.0000000e+00 1.9073486e-06 0.0000000e+00]
    !!!point num=122486 leaf_size=200 min_extent=10
    Octree: build 1508.478, knn 68.372, radius 264.795,  cpp_build 10.922, cpp_knn 0.023,   cpp_radius 0.095, brute 9.774
    ```