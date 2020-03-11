#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <sophus/se3.hpp>
#include <iostream>
#include "random.h"


using namespace Sophus;
using namespace Eigen;
using namespace std;

/// 姿态和内参的结构
struct PoseAndIntrinsics {
    PoseAndIntrinsics() {}

    /// set from given data address
    explicit PoseAndIntrinsics(double *data_addr) {
        rotation = SO3d::exp(Vector3d(data_addr[0], data_addr[1], data_addr[2]));
        translation = Vector3d(data_addr[3], data_addr[4], data_addr[5]);
        focal = data_addr[6];
        k1 = data_addr[7];
        k2 = data_addr[8];
    }

    /// 将估计值放入内存
    void set_to(double *data_addr) {
        auto r = rotation.log();
        for (int i = 0; i < 3; ++i) data_addr[i] = r[i];
        for (int i = 0; i < 3; ++i) data_addr[i + 3] = translation[i];
        data_addr[6] = focal;
        data_addr[7] = k1;
        data_addr[8] = k2;
    }

    SO3d rotation;
    Vector3d translation = Vector3d::Zero();
    double focal = 0;
    double k1 = 0, k2 = 0;
};

/// 位姿加相机内参的顶点，9维，前三维为so3，接下去为t, f, k1, k2
class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoseAndIntrinsics() {}

    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();
    }

    virtual void oplusImpl(const double *update) override {
        _estimate.rotation = SO3d::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Vector3d(update[3], update[4], update[5]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    /// 根据估计值投影一个点
    Vector2d project(const Vector3d &point) {
        Vector3d pc = _estimate.rotation * point + _estimate.translation;
        pc = -pc / pc[2];
        double r2 = pc.squaredNorm();
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
        return Vector2d(_estimate.focal * distortion * pc[0],
                        _estimate.focal * distortion * pc[1]);
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    virtual void setToOriginImpl() override {
        _estimate = Vector3d(0, 0, 0);
    }

    virtual void oplusImpl(const double *update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

class EdgeProjection :
    public g2o::BaseBinaryEdge<2, Vector2d, VertexPoseAndIntrinsics, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override {
        auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];
        auto v1 = (VertexPoint *) _vertices[1];
        auto proj = v0->project(v1->estimate());
        _error = proj - _measurement;
    }

    // void linearizeOplus() override {

    // }

    // use numeric derivatives
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

};

struct observation_t{
  int camera_index;
  int point_index;
  Eigen::Vector2d coord;
};

void solve_ba(std::vector<PoseAndIntrinsics> &cams, std::vector<Eigen::Vector3d> &pts, std::vector<observation_t> obs)
{
    // solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // add vertices
    std::vector<VertexPoseAndIntrinsics*> v_cas(cams.size());
    for(int i=0; i<cams.size(); ++i)
    {
        VertexPoseAndIntrinsics *vp = new VertexPoseAndIntrinsics;
        vp->setEstimate(cams[i]);
        vp->setId(i);
        optimizer.addVertex(vp);
        v_cas[i] = vp;
    }

    std::vector<VertexPoint*> v_points(pts.size());
    for(int i=0; i<v_points.size(); ++i)
    {
        VertexPoint *vp = new VertexPoint;
        vp->setEstimate(pts[i]);
        vp->setMarginalized(true);
        vp->setId(cams.size() + i);
        v_points[i] = vp;
        optimizer.addVertex(vp);
    }

    // add edges
    for(auto &ob : obs)
    {
        EdgeProjection *eg = new EdgeProjection;
        eg->setVertex(0, v_cas[ob.camera_index]);
        eg->setVertex(1, v_points[ob.point_index]);
        eg->setMeasurement(ob.coord);
        eg->setInformation(Matrix2d::Identity());
        eg->setRobustKernel(new g2o::RobustKernelHuber);
        optimizer.addEdge(eg);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(40);

    // get result back
    for(int i=0; i<cams.size(); ++i)
    {
        cams[i] = v_cas[i]->estimate();
    }

    for(int i=0; i<pts.size(); ++i)
    {
        pts[i] = v_points[i]->estimate();
    }
}

void add_noise(std::vector<PoseAndIntrinsics> &cams, std::vector<Eigen::Vector3d> &pts,
        const double sigma_ca, const double sigma_pts)
{
    for(auto &ca : cams) {
        ca.translation[0] += RandNormal() * sigma_ca;
        ca.translation[1] += RandNormal() * sigma_ca;
        ca.translation[2] += RandNormal() * sigma_ca;

        Eigen::Vector3d ax = ca.rotation.log();
        ax[0] += RandNormal() * sigma_ca;
        ax[1] += RandNormal() * sigma_ca;
        ax[2] += RandNormal() * sigma_ca;

        ca.rotation = SO3d::exp(ax);
    }

    for(auto &pt : pts) {
        pt[0] += RandNormal() * sigma_pts;
        pt[1] += RandNormal() * sigma_pts;
        pt[2] += RandNormal() * sigma_pts;
    }
}

int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: bundle_adj bal_data.txt" << endl;
        return 1;
    }

    // load data
    std::ifstream ifs(argv[1]);
    int num_camera;
    int num_points;
    int num_observation;
    ifs >> num_camera >> num_points >> num_observation;
    std::cout << "ca_num=" << num_camera << " num_pts="
              << num_points << " num_obs=" << num_observation << std::endl;
    // observation
    std::vector<observation_t> obs(num_observation);
    for(auto &tmp_ob : obs) {
      ifs >> tmp_ob.camera_index >> tmp_ob.point_index >> tmp_ob.coord[0] >> tmp_ob.coord[1];
      assert(tmp_ob.camera_index < num_camera);
      assert(tmp_ob.point_index < num_points);
    }

    // cameras
    std::vector<PoseAndIntrinsics> cams(num_camera);
    for(auto &ca : cams) {
        Eigen::Vector3d ax;
        ifs >> ax[0] >> ax[1] >> ax[2];
        ifs >> ca.translation[0] >> ca.translation[1] >> ca.translation[2];
        ifs >> ca.focal >> ca.k1 >> ca.k2;
        ca.rotation = SO3d::exp(ax);
    }

    // points
    std::vector<Eigen::Vector3d> pts(num_points);
    for(auto &pt : pts)
    {
        ifs >> pt[0] >> pt[1] >> pt[2];
    }

    // add noise
    add_noise(cams, pts, 0.1, 0.1);

    // output origional result
    std::ofstream ofs("ori.txt");
    for(auto &ca : cams) {
      ofs << ca.translation[0] << " " << ca.translation[1]
          << " " << ca.translation[2] << " " << ca.rotation.log()[0] << " "
          << ca.rotation.log()[1] << " " << ca.rotation.log()[2] << std::endl;
    }
    for(auto &pt : pts)
    {
        ofs << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
    }
    ofs.close();

    // construct and solve
    solve_ba(cams, pts, obs);

    // output result
    ofs.open("result.txt");
    for(auto &pt : pts)
    {
        ofs << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
    }
    ofs.close();
    return 0;
}
