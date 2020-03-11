#include <fstream>
#include "full_bundle_adj_problem.h"
#include "ref/projection.h"
#include "ref/tools/random.h"

int full_bla_problem::load(const std::string &file_path) {
    std::ifstream ifs(file_path);
    if(!ifs) {
        std::cerr << "[ERROR] unable to load file:" << file_path << std::endl;
        return __LINE__;
    }
    int cam_num = 0;
    int pt_num = 0;
    int edge_num = 0;
    ifs >> cam_num >> pt_num >> edge_num;
    std::cout << "camera=" << cam_num << " pt=" << pt_num << " edge=" << edge_num << std::endl;
    obs_.resize(edge_num);
    for(int i=0; i<edge_num; ++i)
    {
        auto &ob = obs_[i];
        ifs >> ob.cam_id >> ob.pt_id >> ob.ob_pixel[0] >> ob.ob_pixel[1];
    }

    cameras_.resize(cam_num);
    for(int i=0; i<cam_num; ++i)
    {
        ifs >> cameras_[i][0] >> cameras_[i][1] >> cameras_[i][2]
            >> cameras_[i][3] >> cameras_[i][4] >> cameras_[i][5]
            >> cameras_[i][6] >> cameras_[i][7] >> cameras_[i][8];
    }

//    std::ofstream ofs("tmp.txt");
    pts_.resize(pt_num);
    for(int i=0; i<pt_num; ++i) {
        ifs >> pts_[i][0] >> pts_[i][1] >> pts_[i][2];
//        ofs << pts_[i][0] << " " << pts_[i][1] << " " << pts_[i][2] << std::endl;
    }
//    ofs.close();
    ifs.close();
    return 0;
}

void full_bla_problem::cam_rt_test()
{
    // pt in cam 0
    // angle axis to quaternion
    Eigen::Vector3d ax = cameras_[0].block<3,1>(0,0);
    double angle = ax.norm();
    Eigen::Vector3d axis(0,0,1);
    if (angle >= 1e-3) axis = ax / angle;
    Eigen::AngleAxisd r_ax(angle, axis);
    std::cout << "angle:" << angle << std::endl;
    std::cout << "angle axis:" << axis.transpose() << std::endl;
    for(const auto &pt : pts_) {
        Eigen::Vector3d pt_in_camera = r_ax * pt; // R *p + t
        Eigen::Vector2d ref_pt;
        CamProjectionWithDistortion(cameras_[0].data(), pt.data(), ref_pt.data());
        Eigen::Vector2d pt2d = cam_project_with_distortion(cameras_[0], pt);
        std::cout << "pt: " << pt.transpose() << std::endl;
        std::cout << "pt_in_camera: " << pt2d.transpose() << std::endl;
        std::cout << "ref pt: " << ref_pt.transpose() << std::endl;
        std::cout << "diff: " << (pt2d - ref_pt).transpose() << std::endl;
    }
}

void full_bla_problem::build_up()
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3> > BalBlockSolver;

    /// for init the optimizer
    // for dens_schur, for sparse_schur use g2o::LinearSolverCholmod
    std::unique_ptr<g2o::LinearSolver<BalBlockSolver::PoseMatrixType>> linearSolver(new g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType>());
    BalBlockSolver* solver_ptr = new BalBlockSolver(std::move(linearSolver));
    // here we can also use g2o::OptimizationAlgorithmDogleg
    g2o::OptimizationAlgorithmWithHessian* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<BalBlockSolver>(solver_ptr));
    optimizer_.setAlgorithm(solver);

    // vertices of cameras
    const int num_cameras = cameras_.size();
    for(int i=0; i<num_cameras; ++i)
    {
        auto vcam = new vertex_camera;
        vcam->setEstimate(cameras_[i]);
        vcam->setId(i);
        optimizer_.addVertex(vcam);
    }

    // vertices of points
    const int num_pts = pts_.size();
    for(int i=0; i<num_pts; ++i)
    {
        auto vpt = new vertex_point;
        vpt->setEstimate(pts_[i]);
        vpt->setId(i+num_cameras);
        vpt->setMarginalized(true);
        optimizer_.addVertex(vpt);
    }

    // edges
    const int num_ob = obs_.size();
    for(int i=0; i<num_ob; ++i)
    {
        auto bal_edge = new edge_observation;
        // using robust kernel
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(1.0);
        bal_edge->setRobustKernel(rk);
        int cam_id = obs_[i].cam_id;
        int pt_id = obs_[i].pt_id + num_cameras;
        bal_edge->setVertex(0, dynamic_cast<vertex_camera*>(optimizer_.vertex(cam_id)));
        bal_edge->setVertex(1, dynamic_cast<vertex_point*>(optimizer_.vertex(pt_id)));

        //In its physical meaning, information matrix represents how reliable this
        // measurement is. Therefore, the more precisely the measurement is made or the
        // more you trust in this measurement, the larger values in the information matrix
        // you can set.
        bal_edge->setInformation(Eigen::Matrix2d::Identity());
        bal_edge->setMeasurement(obs_[i].ob_pixel);
        optimizer_.addEdge(bal_edge);
    }

}

void full_bla_problem::solve()
{
    optimizer_.initializeOptimization();
    optimizer_.setVerbose(true);
    optimizer_.optimize(10);
}

void full_bla_problem::write_ply(const std::string &file_path)
{
    const int num_cameras = cameras_.size();
    const int num_pts = pts_.size();
    std::ofstream of(file_path);

    // Export extrinsic data (i.e. camera centers) as green points.
    for(int i = 0; i < num_cameras; ++i){
        Eigen::Vector3d ax = cameras_[i].block<3,1>(0,0);
        double theta = ax.norm();
        Eigen::Vector3d axis(0,0,1);
        if(theta > 1e-4) axis = ax.normalized();
        Eigen::AngleAxisd r(-theta, axis);
        Eigen::Vector3d t = cameras_[i].block<3,1>(3,0);
        Eigen::Vector3d center =  r*(-t);
        of << center[0] << ' ' << center[1] << ' ' << center[2]
           << " 0\n";
    }

    // Export the structure (i.e. 3D Points) as white points.
    for(int i = 0; i < pts_.size(); ++i){
        of << pts_[i][0] << ' ' << pts_[i][1] << ' ' << pts_[i][2]
        <<  " 1\n";
    }
    of.close();
}

void full_bla_problem::perturb(const double rotation_sigma, const double translation_sigma, const double point_sigma)
{
    for(auto &pt : pts_) {
        pt[0] += RandNormal() * point_sigma;
        pt[1] += RandNormal() * point_sigma;
        pt[2] += RandNormal() * point_sigma;
    }

    for(auto &ca : cameras_)
    {
        ca(0,0) = RandNormal() * rotation_sigma;
        ca(1,0) = RandNormal() * rotation_sigma;
        ca(2,0) = RandNormal() * rotation_sigma;
        ca(3,0) = RandNormal() * translation_sigma;
        ca(4,0) = RandNormal() * translation_sigma;
        ca(5,0) = RandNormal() * translation_sigma;
    }
}

void full_bla_problem::update()
{
    const int num_camera = cameras_.size();
    for(int i=0; i<num_camera; ++i)
    {
        auto camera_v = dynamic_cast<vertex_camera*>(optimizer_.vertex(i));
        cameras_[i] = camera_v->estimate();
    }

    const int num_pts = pts_.size();
    for(int i=0; i<num_pts; ++i)
    {
        auto pt_v = dynamic_cast<vertex_point*>(optimizer_.vertex(i+num_camera));
        pts_[i] = pt_v->estimate();
    }
}

Eigen::Vector2d cam_project_with_distortion(const Eigen::Matrix<double, 9, 1> &camera, const Eigen::Vector3d &point)
{
    // angle axis to quaternion
    Eigen::Vector3d ax = camera.block<3,1>(0,0);
    double angle = ax.norm();
    Eigen::Vector3d axis(0,0,1);
    if (angle >= 1e-3) axis = ax / angle;
    Eigen::AngleAxisd r_ax(angle, axis);

    Eigen::Vector3d pt_in_camera = r_ax * point + camera.block<3,1>(3,0); // R *p + t

    // here assume image haven't flip yet
    Eigen::Vector2d pixel = -pt_in_camera.block<2,1>(0,0)/pt_in_camera[2]; // normalize

    // apply distortion
    double distortion_scale = 1 + pixel.squaredNorm()*(camera[7] + camera[8]*camera[8]);
    pixel = camera[6] * distortion_scale * pixel;
    return pixel;
}
