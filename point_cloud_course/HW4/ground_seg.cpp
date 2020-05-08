#include "ground_seg.h"
#include <math.h>
#include <iomanip>
#include <zsw_vtk_io.h>


void gaussian_process_ground_seg::gen_polar_bin_grid(
     pcl::PointCloud<pcl::PointXYZ>::Ptr &pts)
{
    // gen polar bin grid
    float bsize_rad_inv = params_.num_bins_a/360.0;
    float bsize_lin_inv = params_.num_bins_l/params_.rmax;

    float rad_off = 180.0 * bsize_rad_inv;
    float rad_tr = 180.0 * bsize_rad_inv / M_PI;
    std::vector<std::vector<range_height>> rh_buffer(params_.num_bins_a * params_.num_bins_l);
    const int num_pts = pts->size();
    for (auto i=0; i<num_pts; ++i) {
        const auto &pt = pts->points[i];
        float tmp_r = sqrt(pt.x*pt.x + pt.y*pt.y);
        if(tmp_r > params_.rmax) continue;
        //int rad_ind = (180 + atan2(pt.y, pt.x) * 180 / M_PI) / bsize_rad;
        int rad_ind = static_cast<int>(rad_off + atan2(pt.y, pt.x) * rad_tr);
        int lin_ind = static_cast<int>(tmp_r * bsize_lin_inv);
        rh_buffer[rad_ind*params_.num_bins_l + lin_ind].push_back({pt.z, tmp_r, i, false});
    }

    int valid_pt_cnt = 0;
    for(int i=0; i<params_.num_bins_a; ++i)
    {
        for(int j=0; j<params_.num_bins_l; ++j)
        {
            auto &cur_cell = rh_buffer[i*params_.num_bins_l + j];
            if(cur_cell.size() < 5) continue;
            rh_pts_[i].insert(rh_pts_[i].end(), cur_cell.begin(), cur_cell.end());
            valid_pt_cnt += cur_cell.size();
        }
    }
    std::cout << "valid pt num:" << valid_pt_cnt << std::endl;
}

void gaussian_process_ground_seg::extract_seed(const int ind,
                                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &pts,
                                               std::vector<range_height> &current_model,
                                               std::vector<range_height> &sig_pts)
{
    current_model.clear();
    sig_pts.clear();
    // sort and find candicate seed groud points
    auto &pb = rh_pts_[ind];
    std::sort(pb.begin(), pb.end(),
              [](const range_height &a, const range_height &b){return a.height < b.height;});
    size_t num_points = std::min(params_.num_seed_points, pb.size());
    std::vector<range_height> signal_points;
    for(int j=0; j<pb.size(); ++j)
    {
        if(current_model.size()<num_points
           && pb[j].r < params_.max_seed_range
           && pb[j].height < params_.max_seed_height)
        {
            pb[j].is_ground = true;
            current_model.emplace_back(pb[j]);
        } else sig_pts.emplace_back(pb[j]);
    }

    std::cout << "rh pts:" << rh_pts_[ind].size() << std::endl;

    #if 1
    {
        std::stringstream ss;
        ss << "/home/wegatron/tmp/seed" << std::setw(4) << std::setfill('0') << ind << ".vtk";
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_pc(new pcl::PointCloud<pcl::PointXYZ>);
        model_pc->resize(current_model.size());
        for(int i=0; i<current_model.size(); ++i)
        {
            model_pc->points[i] = pts->points[current_model[i].index];
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr sig_pc(new pcl::PointCloud<pcl::PointXYZ>);
        sig_pc->resize(sig_pts.size());
        for(int i=0; i<sig_pts.size(); ++i)
        {
            sig_pc->points[i] = pts->points[sig_pts[i].index];
        }
        zsw::point_clouds2vtk_file(ss.str(), {model_pc, sig_pc});
    }
    #endif

}

std::vector<uint8_t> gaussian_process_ground_seg::segment(pcl::PointCloud<pcl::PointXYZ>::Ptr &pts)
{
    const auto num_pts = pts->size();
    std::vector<uint8_t> labels(num_pts, false);
    if(num_pts <= 10) return labels;
    gen_polar_bin_grid(pts);
    std::vector<range_height> current_model;
    std::vector<range_height> sig_pts;
    for(int i=0; i<params_.num_bins_a; ++i) {
        extract_seed(i, pts, current_model, sig_pts);
        if(current_model.size() <= 2) continue;
        // insac
    }

    return labels;
#if 0
    // insac
    const float len_scale = 1.0/(params_.p_l * params_.p_l);
    while(true)
    {
        // calculate K_ff^{-1}, K_fy, K_yy
        const int model_size = current_model.size();
        const int sig_size = sig_pts.size();
        Eigen::VectorXf train_x(model_size);
        Eigen::VectorXf x(sig_size);
        Eigen::VectorXf train_y(model_size);
        for(int j=0; j<model_size; ++j)
        {
            train_x[j] = current_model[j].r;
            train_y[j] = current_model[j].height;
        }

        for(int j=0; j<sig_size; ++j)
        {
            x[j] = sig_pts[j].r;
        }

        Eigen::VectorXf sqr_train_x = train_x.array() * train_x.array();
        Eigen::VectorXf sqr_x = x.array() * x.array();

        Eigen::MatrixXf K_ff(model_size, model_size);
        Eigen::MatrixXf inv_K_ff(model_size, model_size);
        {
            Eigen::MatrixXf k_0(model_size, model_size);
            Eigen::MatrixXf k_1(model_size, model_size);
            for(int j=0; j<model_size; ++j) {
                k_0.row(j).setConstant(sqr_train_x[j]);
                k_1.col(j).setConstant(sqr_train_x[j]);
            }

            K_ff = -len_scale * (k_0 + k_1 - 2*train_x * train_x.transpose());
            K_ff = params_.p_sf * exp(K_ff.array()).matrix()
                + params_.p_sn * Eigen::MatrixXf::Identity(model_size, model_size);
            inv_K_ff = K_ff.inverse();
        }

        Eigen::MatrixXf K_yy(sig_size, sig_size);
        {
            Eigen::MatrixXf k_0(sig_size, sig_size);
            Eigen::MatrixXf k_1(sig_size, sig_size);
            for(int j=0; j<sig_size; ++j) {
                k_0.row(j).setConstant(sqr_x[j]);
                k_1.col(j).setConstant(sqr_x[j]);
            }
            K_yy = -len_scale*(k_0 + k_1 - 2*sqr_x * sqr_x.transpose());
            K_yy = params_.p_sf * exp(K_yy.array());
        }

        Eigen::MatrixXf K_fy(model_size, sig_size);
        {
            Eigen::MatrixXf k_0(model_size, sig_size);
            Eigen::MatrixXf k_1(model_size, sig_size);
            for(int j=0; j<model_size; ++j)
                k_0.row(j).setConstant(sqr_train_x[j]);

            for(int j=0; j<sig_size; ++j)
                k_1.col(j).setConstant(sqr_x[j]);

            K_fy = -len_scale*(k_0 + k_1 - 2*sqr_train_x * sqr_x.transpose());
            K_fy = params_.p_sf * exp(K_fy.array());
        }

        // calc y and cov
        Eigen::VectorXf y = K_fy.transpose() * inv_K_ff * train_y;
        Eigen::MatrixXf cov =K_yy - K_fy.transpose() * inv_K_ff * K_fy;

        // evaluate the model
    }
    return labels;
#endif
}
