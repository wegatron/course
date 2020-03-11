//
// Created by Sandra Alfaro on 24/05/18.
//

#include <vector>
#include <atomic>
#include <chrono>

#ifdef OpenGR_USE_OPENMP
#include <omp.h>
#endif

#include "shared.h"
#include "sampling.h"
#include "kdtree.h"
#include "logger.h"

#include "congruentSetExplorationBase.h"
#include <pcl/point_cloud.h>
#include <zsw_vtk_io.h>
#include <iomanip>
#include "demo_io.h"

#ifdef TEST_GLOBAL_TIMINGS
#   include "../utils/timer.h"
#endif


namespace gr {
template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > typename ... OptExts >
  CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::CongruentSetExplorationBase(
          const typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::OptionsType& options
        , const Utils::Logger& logger )
    : MatchBaseType(options, logger)
    , number_of_trials_(0)
    , best_LCP_(0.0)
    #ifdef OpenGR_USE_OPENMP
    , omp_nthread_congruent_(1)
    #endif
//    , options_(options)
{}

template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > class ... OptExts >
CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::~CongruentSetExplorationBase(){}


// The main 4PCS function. Computes the best rigid transformation and transfoms
// Q toward P by this transformation
template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > class ... OptExts >
template <typename Sampler>
typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::Scalar
CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::ComputeTransformation(
        const std::vector<Point3D>& P,
        const std::vector<Point3D>& Q,
        Eigen::Ref<typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::MatrixType> transformation,
        const Sampler& sampler,
        TransformVisitor& v) {
  const Scalar kSmallError = 0.00001;
  const int kMinNumberOfTrials = 4;
  const Scalar kDiameterFraction = 0.3;

#ifdef TEST_GLOBAL_TIMINGS
    kdTreeTime = 0;
    totalTime  = 0;
    verifyTime = 0;
#endif

  if (P.empty() || Q.empty()) return kLargeNumber;

  // RANSAC probability and number of needed trials.
  Scalar first_estimation =
          std::log(kSmallError) / std::log(1.0 - pow(MatchBaseType::options_.getOverlapEstimation(),
                                                     static_cast<Scalar>(kMinNumberOfTrials)));
  // We use a simple heuristic to elevate the probability to a reasonable value
  // given that we don't simply sample from P, but instead, we bound the
  // distance between the points in the base as a fraction of the diameter.
  number_of_trials_ =
          static_cast<int>(first_estimation * (MatchBaseType::P_diameter_ / kDiameterFraction) /
                           MatchBaseType::max_base_diameter_);
  if (number_of_trials_ < kMinNumberOfTrials)
      number_of_trials_ = kMinNumberOfTrials;

  MatchBaseType::template Log<LogLevel::Verbose>( "norm_max_dist: ", MatchBaseType::options_.delta );
  current_trial_ = 0;
  best_LCP_ = 0.0;

  for (int i = 0; i < Traits::size(); ++i) {
      base_[i] = 0;
      current_congruent_[i] = 0;
  }

  MatchBaseType::init(P, Q, sampler);

  //best_LCP_ = Verify(MatchBaseType::transform_);
  MatchBaseType::template Log<LogLevel::Verbose>( "Initial LCP: ", best_LCP_ );

  if (best_LCP_ != Scalar(1.))
    Perform_N_steps(number_of_trials_, transformation, v);



	

  // output sample_p, sample_q and base 4pcs, congruent 4pcs
  pcl::PointCloud<pcl::PointXYZ>::Ptr bp0(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr bp1(new pcl::PointCloud<pcl::PointXYZ>);
  auto sp = sample_p();
  for (const auto & pt : *sp)
  {
	  bp0->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
  }
  auto & b4pcs = base_4pcs();
  for (const auto & pt : b4pcs)
  {
	  bp1->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cp0(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cp1(new pcl::PointCloud<pcl::PointXYZ>);

  auto sq = sample_q();
  for (const auto & pt : *sq)
  {
	  cp0->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
  }
  auto & c4pcs = congruent_4pcs();
  for (const auto & pt : c4pcs)
  {
	  cp1->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
  }

  std::stringstream ss;
  ss << std::setw(4) << std::setfill('0') << cur_id() << ".vtk";

  zsw::point_clouds2vtk_file("f:/data/tmp/4pcs_res/base_" + ss.str(), { bp0, bp1 });
  zsw::point_clouds2vtk_file("f:/data/tmp/4pcs_res/congruent_" + ss.str(), { cp0, cp1 });
  // output rts
  rt2vtk("f:/data/tmp/4pcs_res/rts_"+ss.str(), getTransforms());
#ifdef TEST_GLOBAL_TIMINGS
  MatchBaseType::template Log<LogLevel::Verbose>( "----------- Timings (msec) -------------" );
  MatchBaseType::template Log<LogLevel::Verbose>( " Total computation time  : ", totalTime   );
  MatchBaseType::template Log<LogLevel::Verbose>( " Total verify time       : ", verifyTime  );
  MatchBaseType::template Log<LogLevel::Verbose>( "    Kdtree query         : ", kdTreeTime  );
  MatchBaseType::template Log<LogLevel::Verbose>( "----------------------------------------" );
#endif

  return best_LCP_;
}

// Performs N RANSAC iterations and compute the best transformation. Also,
// transforms the set Q by this optimal transformation.
template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > class ... OptExts >
bool
CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::Perform_N_steps(
        int n,
        Eigen::Ref<typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::MatrixType> transformation,
        TransformVisitor &v) {
  using std::chrono::system_clock;

#ifdef TEST_GLOBAL_TIMINGS
    Timer t (true);
#endif


  // The transformation has been computed between the two point clouds centered
  // at the origin, we need to recompute the translation to apply it to the original clouds
  auto getGlobalTransform = [this](Eigen::Ref<MatrixType> transformation){
    Eigen::Matrix<Scalar, 3, 3> rot, scale;
    Eigen::Transform<Scalar, 3, Eigen::Affine> (MatchBaseType::transform_).computeRotationScaling(&rot, &scale);
    transformation = MatchBaseType::transform_;
    transformation.col(3) = (MatchBaseType::qcentroid1_ + MatchBaseType::centroid_P_ -
            ( rot * scale * (MatchBaseType::qcentroid2_ + MatchBaseType::centroid_Q_))).homogeneous();
  };

  Scalar last_best_LCP = best_LCP_;
  v(0, best_LCP_, transformation);

  bool ok = false;
  std::chrono::time_point<system_clock> t0 = system_clock::now(), end;
  for (int i = current_trial_; i < current_trial_ + n; ++i) {
    ok = TryOneBase(v);

    Scalar fraction_try  = Scalar(i) / Scalar(number_of_trials_);
    Scalar fraction_time =
        std::chrono::duration_cast<std::chrono::seconds>
        (system_clock::now() - t0).count() /
                          MatchBaseType::options_.max_time_seconds;
    Scalar fraction = std::max(fraction_time, fraction_try);

    if (v.needsGlobalTransformation()) {
      getGlobalTransform(transformation);
    } else {
      transformation = MatchBaseType::transform_;
    }

    v(fraction, best_LCP_, transformation);

    // ok means that we already have the desired LCP.
    if (ok || i > number_of_trials_ || fraction >= 0.99 || best_LCP_ == 1.0) break;
  }

  // Need to force global transformation update at the end of the process
  // if not already performed for the visitor during the process
  if (! v.needsGlobalTransformation())
    getGlobalTransform(transformation);

  current_trial_ += n;
#ifdef TEST_GLOBAL_TIMINGS
    totalTime += Scalar(t.elapsed().count()) / Scalar(CLOCKS_PER_SEC);
#endif

  return ok || current_trial_ >= number_of_trials_;
}



template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > class ... OptExts >
bool CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::TryOneBase(
        TransformVisitor &v) {
        CongruentBaseType base;
        Set congruent_quads;
		std::chrono::high_resolution_clock::time_point tp0 = std::chrono::high_resolution_clock::now();
		if (!generateCongruents(base, congruent_quads)) {
			std::cout << "generateCongruents time cost: "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp0).count()
				<< " ms" << std::endl;
			return false;
		}

		auto tp1 = std::chrono::high_resolution_clock::now();
		std::cout << "generateCongruents time cost: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(tp1 - tp0).count()
			<< " ms" << std::endl;

        size_t nb = 0;
		
        bool match = TryCongruentSet(base,congruent_quads,v,nb);
		std::cout << "TryCongruentSet time cost: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp1).count()
			<< " ms" << std::endl;
        //if (nb != 0)
        //  MatchBaseType::Log<LogLevel::Verbose>( "Congruent quads: (", nb, ")    " );

        return match;
}

template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > class ... OptExts >
bool CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::TryCongruentSet(
        typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::CongruentBaseType& base,
        typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::Set& set,
        TransformVisitor &v,
        size_t &nbCongruent) {
    static const Scalar pi = std::acos(-1);

    // get references to the basis coordinate
    Coordinates references;
//    std::cout << "Process congruent set for base: \n";
    for (int i = 0; i!= Traits::size(); ++i) {
        references[i] = MatchBaseType::sampled_P_3D_[base[i]];
//        std::cout << "[" << base[i] << "]: " << references[i].pos().transpose() << "\n";
    }
    const Coordinates& ref = references;
    Scalar targetAngle = (references[1].pos() - references[0].pos()).normalized().dot(
          (references[3].pos() - references[2].pos()).normalized());
//    std::cout << "Target Angle : " << std::acos(targetAngle)*Scalar(180)/pi << std::endl;

    // Centroid of the basis, computed once and using only the three first points
    Eigen::Matrix<Scalar, 3, 1> centroid1 = (ref[0].pos() + ref[1].pos() + ref[2].pos()) / Scalar(3);

//    std::cout << "Congruent set size: " << set.size() <<  std::endl;

    std::atomic<size_t> nbCongruentAto(0);

    Coordinates congruent_candidate;
	size_t valid_congruent_cnt = 0;
#ifdef OpenGR_USE_OPENMP
#pragma omp parallel for num_threads(omp_nthread_congruent_)
#endif
    for (int i = 0; i < int(set.size()); ++i) {
        const auto& congruent_ids = set[i];
        for (int j = 0; j!= Traits::size(); ++j)
            congruent_candidate[j] = MatchBaseType::sampled_Q_3D_[congruent_ids[j]];

		if (fabs(ref[1].z() - ref[2].z() - congruent_candidate[1].z() + congruent_candidate[2].z()) > 0.3)
		{
			continue;
		}

		Eigen::Matrix<Scalar, 4, 4> transform;

        // Centroid of the sets, computed in the loop using only the three first points
        Eigen::Matrix<Scalar, 3, 1> centroid2;


#ifdef STATIC_BASE
        MatchBaseType::Log<LogLevel::Verbose>( "Ids: ");
        for (int j = 0; j!= Traits::size(); ++j)
            MatchBaseType::Log<LogLevel::Verbose>( base[j], "\t");
        MatchBaseType::Log<LogLevel::Verbose>( "     ");
        for (int j = 0; j!= Traits::size(); ++j)
            MatchBaseType::Log<LogLevel::Verbose>( congruent_ids[j], "\t");
#endif

        centroid2 = (congruent_candidate[0].pos() +
                congruent_candidate[1].pos() +
                congruent_candidate[2].pos()) / Scalar(3.);

        Scalar rms = -1;

        const bool ok =
                this->ComputeRigidTransformation(ref,     // input congruent quad
                                           congruent_candidate,// tested congruent quad
                                           centroid1,          // input: basis centroid
                                           centroid2,          // input: candidate quad centroid
                                           transform,          // output: transformation
                                           rms,                // output: rms error of the transformation between the basis and the congruent quad
                                   #ifdef MULTISCALE
                                           true
                                   #else
                                           false
                                   #endif
                                           );             // state: compute scale ratio ?

        if (ok && rms >= Scalar(0.)) {

            // We give more tolerant in computing the best rigid transformation.
            if (rms < distance_factor * MatchBaseType::options_.delta) {

//                std::cout << "congruent candidate: [";
//                for (int j = 0; j!= Traits::size(); ++j)
//                    std::cout << congruent_ids[j] << " ";
//                std::cout << "]";

                nbCongruentAto++;
                // The transformation is computed from the point-clouds centered inn [0,0,0]

                // Verify the rest of the points in Q against P.
				// for ZSW_DEBUG verify
				Eigen::Affine3f rt;
            	rt.matrix() = transform;
				Eigen::Vector3f translate = rt.translation();
				if (Eigen::Vector3f(0, 0, 1).dot(rt.rotation() * Eigen::Vector3f(0, 0, 1)) <0.966) continue;
				addTransform(rt);
				++valid_congruent_cnt;
				//Verify(transform);
				float lcp = 0;

				//auto tp0 = std::chrono::high_resolution_clock::now();
				//float vf = Verify(transform);
				//std::cout << "verify time:" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp0).count()
				//	<< std::endl;
				//lcp = vf;
				//if(aa.angle() < 0.17 && translate.norm()<0.1)
				//{
				//	//float vf = Verify(transform);
				//	lcp = 1.0;
				//	//std::cout << "!!!!!!!!!!!!!!!!!!!!!! fine !!!!!!!!!!!!!" << std::endl;
				//	//std::cout << transform << std::endl;
				//	
				//	
				//	if(vf > 0.1) std::cout << "in:" << vf << std::endl;
				//	//Eigen::Matrix4f ttrt = Eigen::Matrix4f::Identity();
				//	//ttrt.block<3, 1>(0, 3) = translate;
				//	//std::cout << ttrt << std::endl;
				//	////auto id = Eigen::Matrix<Scalar, 4, 4>::Identity();
				//	//std::cout << "identity:" << Verify(ttrt) << std::endl;

				//	// TODO output 4pcs set here
				//}
				//else if (vf > 0.05) std::cout << "out:" << vf << std::endl;
				
                 //Scalar lcp = Verify(transform);
//                std::cout << " " << lcp << " - Angle: ";

                // compute angle between pairs of points: for debug only
//                Scalar angle = (congruent_candidate[1].pos() - congruent_candidate[0].pos()).normalized().dot(
//                      (congruent_candidate[3].pos() - congruent_candidate[2].pos()).normalized());
//                std::cout << std::acos(angle)*Scalar(180)/pi << " (error: "
//                          << (std::acos(targetAngle) - std::acos(angle))*Scalar(180)/pi << ")\n";

                // transformation has been computed between the two point clouds centered
                // at the origin, we need to recompute the translation to apply it to the original clouds
#pragma omp critical
                {
                  auto getGlobalTransform =
                      [this, transform, centroid1, centroid2]
                      (Eigen::Ref<MatrixType> transformation){
                      Eigen::Matrix<Scalar, 3, 3> rot, scale;
                      Eigen::Transform<Scalar, 3, Eigen::Affine> (transform).computeRotationScaling(&rot, &scale);
                      transformation = transform;
                      transformation.col(3) = (centroid1 + MatchBaseType::centroid_P_ -
                                               ( rot * scale * (centroid2 + MatchBaseType::centroid_Q_))).homogeneous();
                    };

                  if (v.needsGlobalTransformation())
                    {
                      Eigen::Matrix<Scalar, 4, 4> transformation = transform;
                      getGlobalTransform(transformation);
                      v(-1, lcp, transformation);
                    }
                  else
                    v(-1, lcp, transform);

                  if (lcp > best_LCP_) {
					  std::cout << "-------------------------------" << std::endl;
					  std::cout << "best lcp=" << lcp << std::endl;
					  std::cout << "transform:\n" << transform << std::endl;

					  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> ref_4pcs;
					  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> c_4pcs;

					  for (int debug_i = 0; debug_i < 4; ++debug_i)
					  {
						  std::cout << "p" << debug_i << ":" << ref[debug_i].x() << " " << ref[debug_i].y() << " " << ref[debug_i].z() << std::endl;
						  ref_4pcs.emplace_back(ref[debug_i].x(), ref[debug_i].y(), ref[debug_i].z());
					  }

					  for (int debug_i = 0; debug_i < 4; ++debug_i)
					  {
						  std::cout << "q" << debug_i << ":" << congruent_candidate[debug_i].x() << " " << congruent_candidate[debug_i].y() << " " << congruent_candidate[debug_i].z() << std::endl;
						  c_4pcs.emplace_back(congruent_candidate[debug_i].x(), congruent_candidate[debug_i].y(), congruent_candidate[debug_i].z());
					  }
					  
					  set_base_4pcs(ref_4pcs);
					  set_congruent_4pcs(c_4pcs);

                      // Retain the best LCP and transformation.
                      for (int j = 0; j!= Traits::size(); ++j)
                        base_[j] = base[j];


                      for (int j = 0; j!= Traits::size(); ++j)
                        current_congruent_[j] = congruent_ids[j];

                      best_LCP_                   = lcp;
                      MatchBaseType::transform_   = transform;
                      MatchBaseType::qcentroid1_  = centroid1;
                      MatchBaseType::qcentroid2_  = centroid2;
                    }

                }
                // Terminate if we have the desired LCP already.
                if (lcp > MatchBaseType::options_.getTerminateThreshold()){
                    continue;
                  }
            }
          } else {

//            std::cout << "skipped candidate: [";
//            for (int j = 0; j!= Traits::size(); ++j)
//                std::cout << congruent_ids[j] << " ";
//            std::cout << "]\n";
          }
    }

    nbCongruent = nbCongruentAto;
	std::cout << "valid congruent:" << valid_congruent_cnt << std::endl;
    // If we reached here we do not have yet the desired LCP.
    return best_LCP_ > MatchBaseType::options_.getTerminateThreshold() /*false*/;
}


// Verify a given transformation by computing the number of points in P at
// distance at most (normalized) delta from some point in Q. In the paper
// we describe randomized verification. We apply deterministic one here with
// early termination. It was found to be fast in practice.
template <typename Traits, typename TransformVisitor,
          typename PairFilteringFunctor,
          template < class, class > class ... OptExts >
typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::Scalar
CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::Verify(
        const Eigen::Ref<const typename CongruentSetExplorationBase<Traits, TransformVisitor, PairFilteringFunctor, OptExts ...>::MatrixType> &mat) const {
    using RangeQuery = typename gr::KdTree<Scalar>::template RangeQuery<>;

#ifdef TEST_GLOBAL_TIMINGS
    Timer t_verify (true);
#endif

    // We allow factor 2 scaling in the normalization.
    const Scalar epsilon = MatchBaseType::options_.delta * 1.5;
#ifdef OPENGR_USE_WEIGHTED_LCP
    std::atomic<float> good_points(0);

    auto kernel = [](Scalar x) {
        return std::pow(std::pow(x,4) - Scalar(1), 2);
    };

    auto computeWeight = [kernel](Scalar sqx, Scalar th) {
        return kernel( std::sqrt(sqx) / th );
    };
#else
    std::atomic_uint good_points(0);
#endif
    const size_t number_of_points = MatchBaseType::sampled_Q_3D_.size();
    const size_t terminate_value = best_LCP_ * number_of_points;

    const Scalar sq_eps = epsilon*epsilon;
#ifdef OPENGR_USE_WEIGHTED_LCP
    const Scalar    eps = std::sqrt(sq_eps);
#endif

    for (size_t i = 0; i < number_of_points; ++i) {

        // Use the kdtree to get the nearest neighbor
#ifdef TEST_GLOBAL_TIMINGS
        Timer t (true);
#endif

        RangeQuery query;
        query.queryPoint = (mat * MatchBaseType::sampled_Q_3D_[i].pos().homogeneous()).template head<3>();
        query.sqdist     = sq_eps;

        auto result = MatchBaseType::kd_tree_.doQueryRestrictedClosestIndex( query );

#ifdef TEST_GLOBAL_TIMINGS
        kdTreeTime += Scalar(t.elapsed().count()) / Scalar(CLOCKS_PER_SEC);
#endif

        if ( result.first != gr::KdTree<Scalar>::invalidIndex() ) {
            //      Point3D& q = sampled_P_3D_[near_neighbor_index[0]];
            //      bool rgb_good =
            //          (p.rgb()[0] >= 0 && q.rgb()[0] >= 0)
            //              ? cv::norm(p.rgb() - q.rgb()) < options_.max_color_distance
            //              : true;
            //      bool norm_good = norm(p.normal()) > 0 && norm(q.normal()) > 0
            //                           ? fabs(p.normal().ddot(q.normal())) >= cos_dist
            //                           : true;
            //      if (rgb_good && norm_good) {
#ifdef OPENGR_USE_WEIGHTED_LCP
            assert (result.second <= query.sqdist);
            good_points = good_points + computeWeight(result.second, eps);
#else
            good_points++;
#endif
            //      }
        }

        // We can terminate if there is no longer chance to get better than the
        // current best LCP.
        if (number_of_points - i + good_points < terminate_value) {
            break;
        }
    }

#ifdef TEST_GLOBAL_TIMINGS
    verifyTime += Scalar(t_verify.elapsed().count()) / Scalar(CLOCKS_PER_SEC);
#endif
    return Scalar(good_points) / Scalar(number_of_points);
}

}
