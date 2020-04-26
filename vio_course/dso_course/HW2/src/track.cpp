#include "track.hpp"
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include <iostream>

static constexpr float setting_huber_th = 9;

hw::Tracker::Tracker(
    int         top_level,   
    int         bottom_level,
    size_t      num_iter_max,     
    PatternType pattern ) :
    num_level_max_(top_level),
    num_level_min_(bottom_level),
    pattern_(pattern),
    num_iter_max_(num_iter_max)
{
    num_iter_ = 0;
    epsilon_ = 0.00000001;
    stop_ = false;
    num_obser_ = 0;
};

size_t hw::Tracker::run(Frame::Ptr &frame_ref, Frame::Ptr &frame_cur)
{
    if(frame_ref->features.empty())
    {
        std::cout<<"[error]: There is nothing in features!"<<std::endl;
    }

    patch_ref_ = cv::Mat::zeros(frame_ref->features.size(), pattern_.size(), CV_32F);
    patch_dI_.resize(frame_ref->features.size()*pattern_.size(), Eigen::NoChange);
    visible_ftr_.resize(frame_ref->features.size(), false);
    jacobian_.resize(frame_ref->features.size()*pattern_.size(), Eigen::NoChange);
    residual_.resize(frame_ref->features.size()*pattern_.size(), Eigen::NoChange);

    num_ftr_active_ = 0;

    // Get the optimizing state
    Sophus::SE3d T_r_w = frame_ref->getPose();
    Sophus::SE3d T_c_w = frame_cur->getPose();
    Sophus::SE3d T_cur_ref = T_c_w * T_r_w.inverse();

    // size_t all_obser;
    // Frames to aligne
    frame_cur_ = frame_cur;
    frame_ref_ = frame_ref;

    for(level_cur_=num_level_max_; level_cur_ >= num_level_min_; level_cur_--)
    {
        preCompute();  // get reference patch

        runOptimize(T_cur_ref); // first compute Jacobian and residual, then solve

        // all_obser += num_ftr_active_; // all pixels we used
    }

    // update result
    T_c_w = T_cur_ref*T_r_w;
    frame_cur->setPose(T_c_w);

    return num_ftr_active_;

}


void hw::Tracker::preCompute()
{
    const cv::Mat img_ref_pyr = frame_ref_->getImage(level_cur_);
    patch_dI_.setZero();
    size_t num_ftr = frame_ref_->features.size();
    const double scale = 1.f/(1<<level_cur_); // current level
    int stride = img_ref_pyr.cols; // step

    auto iter_vis_ftr = visible_ftr_.begin();

    for(size_t i_ftr=0; i_ftr<num_ftr; ++i_ftr, ++iter_vis_ftr)
    {
        //! feature on level_cur
        const int ftr_x = frame_ref_->features[i_ftr].pt.x; // feature coordiante
        const int ftr_y = frame_ref_->features[i_ftr].pt.y;
        const float ftr_pyr_x = ftr_x*scale; // current level feature coordiante
        const float ftr_pyr_y = ftr_y*scale;

        bool is_in_frame = true;
        //* patch pointer on head
        float* data_patch_ref = reinterpret_cast<float*>(patch_ref_.data)+i_ftr*pattern_.size();
        //* img_ref_pyr pointer on feature
        // float* data_img_ref = reinterpret_cast<float*> (img_ref_pyr.data)+ftr_pyr_y_i*stride+ftr_pyr_x_i;
        int pattern_count = 0;

        for(auto iter_pattern : pattern_)
        {
            const float x_img_pattern = ftr_pyr_x + iter_pattern.first;
            const float y_img_pattern = ftr_pyr_y + iter_pattern.second;
            // Is in the image of current level
            if( x_img_pattern < 1 || y_img_pattern < 1 ||
                x_img_pattern > img_ref_pyr.cols - 2 ||
                y_img_pattern > img_ref_pyr.rows - 2 )
            {
                is_in_frame = false;
                break;
            }

            // reference patch
            data_patch_ref[pattern_count] = utils::interpolate_uint8((img_ref_pyr.data), x_img_pattern, y_img_pattern, stride);
            ++pattern_count;
        }

        if(is_in_frame)
        {
            *iter_vis_ftr = true;
        }
    }
}

//! NOTE myself: need to make sure residual match Jacobians
void hw::Tracker::computeResidual(const Sophus::SE3d& state)
{
    residual_.setZero();
    jacobian_.setZero();
    patch_dI_.setZero();
    Sophus::SE3d T_cur_ref = state;
    // cv::Mat img_cur_pyr = frame_cur_->getImage(level_cur_);
    const cv::Mat img_cur_pyr = frame_cur_->getImage(level_cur_);
    size_t num_ftr = frame_ref_->features.size();
    int num_pattern = pattern_.size();
    int stride = img_cur_pyr.cols;
    const double scale = 1.f/(1<<level_cur_);
    auto iter_vis_ftr = visible_ftr_.begin();

    const auto fx = Cam::cx() * scale;
    const auto fy = Cam::cy() * scale;
    for(size_t i_ftr=0; i_ftr<num_ftr; ++i_ftr, ++iter_vis_ftr)
    {
        if(!*iter_vis_ftr)
            continue;

        bool is_in_frame = true;

        Eigen::VectorXd residual_pattern = Eigen::VectorXd::Zero(num_pattern);
        size_t pattern_count = 0;
        Eigen::Array<double, 8, 1> hw = Eigen::Array<double, 8, 1>::Zero();
        float* data_patch_ref = reinterpret_cast<float*>(patch_ref_.data) + i_ftr*num_pattern;

        const int ftr_ref_x = frame_ref_->features[i_ftr].pt.x;
        const int ftr_ref_y = frame_ref_->features[i_ftr].pt.y;
        const double ftr_depth_ref = frame_ref_->getDepth(ftr_ref_x, ftr_ref_y);

        // project to current
        Eigen::Vector3d point_ref(Cam::pixel2unitPlane(ftr_ref_x, ftr_ref_y)*ftr_depth_ref);
        Eigen::Vector3d point_cur(T_cur_ref*point_ref);
        Eigen::Vector2d ftr_cur = Cam::project(point_cur)*scale;

        for(auto iter_pattern : pattern_)
        {
            const float ftr_cur_x = ftr_cur.x()+iter_pattern.first;
            const float ftr_cur_y = ftr_cur.y()+iter_pattern.second;

            // Is in the image of current level
            if( ftr_cur_x < 1 || ftr_cur_y < 1 ||
                ftr_cur_x > img_cur_pyr.cols - 2 ||
                ftr_cur_y > img_cur_pyr.rows - 2 )
            {
                is_in_frame = false;
                break;
            }

            float pattern_value = utils::interpolate_uint8((img_cur_pyr.data), ftr_cur_x, ftr_cur_y, stride);

            // derive patch
            float dx = 0.5*(utils::interpolate_uint8((img_cur_pyr.data), ftr_cur_x+1, ftr_cur_y, stride) -
                            utils::interpolate_uint8((img_cur_pyr.data), ftr_cur_x-1, ftr_cur_y, stride));
            float dy = 0.5*(utils::interpolate_uint8((img_cur_pyr.data), ftr_cur_x, ftr_cur_y+1, stride) -
                            utils::interpolate_uint8((img_cur_pyr.data), ftr_cur_x, ftr_cur_y-1, stride));
            patch_dI_.row(i_ftr*pattern_.size()+pattern_count) = Eigen::Vector2d(dx, dy);

            // TODO calculate residual res = ref - cur may need to add huber
            auto res = pattern_value-data_patch_ref[pattern_count];
            auto fabs_res = fabs(res);
            hw[pattern_count] = fabs_res < setting_huber_th ? 1 : sqrt(setting_huber_th/fabs_res);
            residual_pattern[pattern_count] = hw[pattern_count] * res;
            ++pattern_count;
        }

        if(is_in_frame)
        {
            residual_.segment(i_ftr*num_pattern, num_pattern) = residual_pattern;
            num_ftr_active_++;
        }
        else
        {
            *iter_vis_ftr = false;
            continue;
        }
        // TODO calculate Jacobian
        Eigen::Matrix<double, 8, 6> ref_mat;
        {
            const double Z_inv = 1.0/point_cur.z();
            const double Z2_inv = Z_inv * Z_inv;
            const double X = point_cur.x();
            const double Y = point_cur.y();
            Eigen::Matrix<double, 2, 6> J_pixel_xi;
            J_pixel_xi(0, 0) = fx * Z_inv;
            J_pixel_xi(0, 1) = 0;
            J_pixel_xi(0, 2) = -fx * X * Z2_inv;
            J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
            J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
            J_pixel_xi(0, 5) = -fx * Y * Z_inv;

            J_pixel_xi(1, 0) = 0;
            J_pixel_xi(1, 1) = fy * Z_inv;
            J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
            J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
            J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
            J_pixel_xi(1, 5) = fy * X * Z_inv;
            for(int pi=0; pi<8; ++pi) {
                Eigen::Vector2d J_img_pixel = patch_dI_.row(i_ftr * pattern_.size() + pi);
                jacobian_.block<1,6>(num_pattern*i_ftr+pi, 0) = hw[pi]*J_img_pixel.transpose() * J_pixel_xi;
            }
            ref_mat = jacobian_.block<8,6>(num_pattern*i_ftr, 0);
            //std::cout << "ref method:\n" << jacobian_.block<8,6>(num_pattern*i_ftr, 0) << std::endl;
        }
        {
            // jacobian_.block(num_pattern*i_ftr, 0, num_pattern, 6) = ;
            const auto inv_depth = 1.0 / point_cur[2];
            const auto fxdx = fx*patch_dI_.block<8, 1>(num_pattern*i_ftr, 0).array();
            const auto fydy = fy*patch_dI_.block<8, 1>(num_pattern*i_ftr, 1).array();
            const auto u = point_cur.x() * inv_depth;
            const auto v = point_cur.y() * inv_depth;

            jacobian_.block<8,1>(num_pattern*i_ftr, 0).array() = hw*(inv_depth*fxdx);
            jacobian_.block<8,1>(num_pattern*i_ftr, 1).array() = hw*(inv_depth*fydy);
            jacobian_.block<8,1>(num_pattern*i_ftr, 2).array() = hw*(-inv_depth*u*fxdx - inv_depth*v*fydy);
            jacobian_.block<8,1>(num_pattern*i_ftr, 3).array() = hw*(-u*v*fxdx - (1+v*v)*fydy);
            jacobian_.block<8,1>(num_pattern*i_ftr, 4).array() = hw*((1+u*u)*fxdx + u*v*fydy);
            jacobian_.block<8,1>(num_pattern*i_ftr, 5).array() = hw*(-v*fxdx + u*fydy);

            std::cout << "--------" << std::endl;
            std::cout << ref_mat - jacobian_.block<8,6>(num_pattern*i_ftr, 0) << std::endl;
        }
    }
}

bool hw::Tracker::runOptimize(Sophus::SE3d& state)
{
    optimizeGaussNetow(state);

    if(converged_ == true)
        return true;
    else
        return false;
}

void hw::Tracker::optimizeGaussNetow(Sophus::SE3d& state)
{
    reset();
    computeResidual(state);

    while(num_iter_ < num_iter_max_ && !stop_)
    {
        Sophus::SE3d state_new(state);
        // new chi2
        double chi2_new = 0;
        // old chi2
        chi2_ = getChi2();

        // TODO calculate H and b
        hessian_ = jacobian_.transpose() * jacobian_;
        jres_ = -jacobian_.transpose() * residual_;

        if( !solve())
        {
            stop_ = true;
        }
        else
        {
            update(state, state_new);
            // NOTICE: this will change residual
            computeResidual(state_new);
            chi2_new = getChi2();
        }

        num_obser_ = jacobian_.rows();

        if(chi2_ >= chi2_new && !stop_)
        {
            state = state_new;
            chi2_ = chi2_new;
        }
        else
        {
            std::cout << "old_chi2=" << chi2_ << " new_chi2=" << chi2_new << std::endl;
            stop_ = true;
            converged_ = false;
        }

        // converged condition

        if( utils::maxFabs(delta_x_) < epsilon_ && !std::isnan(delta_x_[0]))
        {
            converged_=true;
            stop_ = true;
        }
        ++num_iter_;

        std::cout<<"[message]"<<std::setprecision(4)
            <<"\t NO."<<num_iter_
            <<"\t Obser "<<num_obser_
            <<"\t Chi2 "<<chi2_
            <<"\t delta_x "<<delta_x_.transpose()
            <<"\t Stop "<<std::boolalpha<<stop_
            <<"\t Converged "<<std::boolalpha<<converged_
            <<std::endl;
    }

}


bool hw::Tracker::solve()
{
    delta_x_ = hessian_.ldlt().solve(jres_);
    if((double)std::isnan(delta_x_[0]))
        return false;
    return true;
}

void hw::Tracker::update(const Sophus::SE3d& old_state, Sophus::SE3d& new_state)
{
    // TODO update new state
    new_state = Sophus::SE3d::exp(delta_x_) * old_state;
}
