#include <cmath>
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "define.h"
#include "optimizer.h"
#include "convertor.h"

void two_view_ba(Frame &frame_last, Frame &frame_curr, LoaclMap &map, std::vector<FMatch> &matches, int n_iter)
{
    // TODO homework
    // after you complete this funtion, remove the "return"
//    return;

    const double fx = frame_last.K_(0, 0);
    const double fy = frame_last.K_(1, 1);
    const double cx = frame_last.K_(0, 2);
    const double cy = frame_last.K_(1, 2);
    g2o::SparseOptimizer optimizer;
    using BlockSolverType=g2o::BlockSolver_6_3;
    using LinearSolverType=g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer.setAlgorithm(solver);

    // Set vertices
    Eigen::Matrix4d last_Tcw = frame_last.Twc_.inverse();
    Eigen::Matrix4d curr_Tcw = frame_curr.Twc_.inverse();

    const std::vector<Eigen::Vector2i> &features_last = frame_last.fts_;
    const std::vector<Eigen::Vector2i> &features_curr = frame_curr.fts_;

    int frame_id_last = frame_last.idx_;
    int frame_id_curr = frame_curr.idx_;

    g2o::VertexSE3Expmap *v_last = new g2o::VertexSE3Expmap;
    v_last->setEstimate(Converter::toSE3Quat(last_Tcw));
    v_last->setId(0);
    v_last->setFixed(true);
    optimizer.addVertex(v_last);

    g2o::VertexSE3Expmap *v_cur = new g2o::VertexSE3Expmap;
    v_cur->setEstimate(Converter::toSE3Quat(curr_Tcw));
    v_cur->setId(1);
    optimizer.addVertex(v_cur);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    bool bRobust = true;

    int max_frame_id = std::max(frame_id_last, frame_id_curr) + 1;

    // Set MapPoint vertices
    std::vector<g2o::VertexSBAPointXYZ*> pvs(matches.size(), nullptr);
    int ind=0;
    for(size_t i=0; i<matches.size(); i++)
    {
        if(matches[i].outlier)
            continue;

        uint32_t idx_curr = matches[i].first;
        uint32_t idx_last = matches[i].second;
        int32_t idx_mpt = frame_curr.mpt_track_[idx_curr];
        assert(idx_mpt >=0);
        assert(true == map.status_[idx_mpt]);
        assert(idx_mpt == frame_last.mpt_track_[idx_last]);

        Eigen::Vector3d &mpt = map.mpts_[idx_mpt];

        {
            g2o::VertexSBAPointXYZ * pv = new g2o::VertexSBAPointXYZ;
            pv->setEstimate(mpt);
            pv->setId(ind+2);
            pv->setMarginalized(true);
            optimizer.addVertex(pv);
            pvs[i] = pv;

            g2o::EdgeSE3ProjectXYZ * eg_last = new g2o::EdgeSE3ProjectXYZ;
            eg_last->setVertex(0, pv);
            eg_last->setVertex(1, v_last);
            eg_last->setMeasurement(Eigen::Vector2d(frame_last.fts_[idx_last][0], frame_last.fts_[idx_last][1]));
            eg_last->setInformation(Eigen::Matrix2d::Identity());
            eg_last->fx =fx;
            eg_last->fy = fy;
            eg_last->cx = cx;
            eg_last->cy = cy;

            g2o::RobustKernelHuber *rk_last = new g2o::RobustKernelHuber;
            rk_last->setDelta(thHuber2D);
            eg_last->setRobustKernel(rk_last);
            optimizer.addEdge(eg_last);

            g2o::EdgeSE3ProjectXYZ * eg_cur = new g2o::EdgeSE3ProjectXYZ;
            eg_cur->setVertex(0, pv);
            eg_cur->setVertex(1, v_cur);
            eg_cur->setMeasurement(Eigen::Vector2d(frame_curr.fts_[idx_curr][0], frame_curr.fts_[idx_curr][1]));
            eg_cur->setInformation(Eigen::Matrix2d::Identity());
            eg_cur->fx = fx;
            eg_cur->fy = fy;
            eg_cur->cx = cx;
            eg_cur->cy = cy;
            g2o::RobustKernelHuber *rk_cur = new g2o::RobustKernelHuber;
            rk_cur->setDelta(thHuber2D);
            eg_cur->setRobustKernel(rk_cur);
            optimizer.addEdge(eg_cur);
            ++ind;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(n_iter);

    // TODO homework
    // Recover optimized data
    // Frame Pose
    {
        // frame_last.Twc_ = frame_last.Twc_;
        frame_curr.Twc_ = Converter::toEigenMat(v_cur->estimate()).inverse();
    }
    
    // Points
    for(size_t i = 0; i < matches.size(); i++)
    {
        if(matches[i].outlier) { continue; }
        assert(pvs[i] != nullptr);
        uint32_t idx_last = matches[i].second;
        int32_t idx_mpt = frame_last.mpt_track_[idx_last];

        Eigen::Vector3d &mpt = map.mpts_[idx_mpt];

        mpt = pvs[i]->estimate();
    }
}
