#include <cmath>
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "define.h"
#include "optimizer.h"
#include "convertor.h"

void two_view_ba(Frame &frame_last, Frame &frame_curr, LoaclMap &map, std::vector<FMatch> &matches, int n_iter)
{
    // TODO homework
    // after you complete this funtion, remove the "return"
    return;

    const double fx = frame_last.K_(0, 0);
    const double fy = frame_last.K_(1, 1);
    const double cx = frame_last.K_(0, 2);
    const double cy = frame_last.K_(1, 2);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Set vertices
    Eigen::Matrix4d last_Tcw = frame_last.Twc_.inverse();
    Eigen::Matrix4d curr_Tcw = frame_curr.Twc_.inverse();

    const std::vector<Eigen::Vector2i> &features_last = frame_last.fts_;
    const std::vector<Eigen::Vector2i> &features_curr = frame_curr.fts_;

    int frame_id_last = frame_last.idx_;
    int frame_id_curr = frame_curr.idx_;
    // TODO homework
    // add frame pose Vertex to optimizer
    // example: 
    // g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    // ...
    // optimizer.addVertex(vSE3);
    {
       
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    bool bRobust = true;

    int max_frame_id = std::max(frame_id_last, frame_id_curr) + 1;

    // Set MapPoint vertices
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

        // TODO homework
        // add mappoint Vertex to optimizer
        // example: 
        // g2o::VertexSBAPointXYZ * vPoint = new g2o::VertexSBAPointXYZ();
        // ...
        // optimizer.addVertex(vPoint);
        {
        
        }
        
        // TODO homework
        // add edage to optimizer
        // example: 
        // g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
        // ...
        // optimizer.addEdge(e);
        {

        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(n_iter);

    // TODO homework
    // Recover optimized data
    // Frame Pose
    {
        frame_last.Twc_ = frame_last.Twc_;
        frame_curr.Twc_ = frame_curr.Twc_;
    }
    
    // Points
    for(size_t i = 0; i < matches.size(); i++)
    {
        if(matches[i].outlier) { continue; }
        uint32_t idx_last = matches[i].second;
        int32_t idx_mpt = frame_last.mpt_track_[idx_last];

        Eigen::Vector3d &mpt = map.mpts_[idx_mpt];

        mpt = mpt;
    }
}