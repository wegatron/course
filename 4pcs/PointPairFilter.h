//
// Created by Sandra Alfaro on 26/04/18.
//

#ifndef OPENGR_FUNCTORFEATUREPOINTTEST_H
#define OPENGR_FUNCTORFEATUREPOINTTEST_H

#include "shared.h"
#include <vector>

namespace gr {

    /// \brief Functor used in n-pcs algorithm to filters pairs of points according
    ///        to the exploration basis,
    /// \tparam
    /// \implements PairFilterConcept
    ///
    struct DummyPointFilter {
    template < class Derived, class TBase>
    struct Options : public TBase {
      bool dummyFilteringResponse;
      enum { IS_DUMMYPOINTFILTER_OPTIONS = true };
    };
    template <typename WantedOptionsAndMore>
    inline std::pair<bool,bool> operator() (const Point3D& /*p*/,
                                            const Point3D& /*q*/,
                                            typename Point3D::Scalar /*pair_normals_angle*/,
                                            const Point3D& /*b0*/,
                                            const Point3D& /*b1*/,
                                            const WantedOptionsAndMore &options) {
        return std::make_pair(options.dummyFilteringResponse, options.dummyFilteringResponse);
    }
    };

    /// \brief Functor used in n-pcs algorithm to filters pairs of points according
    ///        to the exploration basis. Uses normal, colors and max motion when
    ///        available
    ///
    /// \implements PairFilterConcept
    ///
    struct AdaptivePointFilter {
      template < class Derived, class TBase>
      struct Options : public TBase {
        using Scalar = typename TBase::Scalar;

        /// Maximum normal difference.
        Scalar max_normal_difference = -1;
        /// Maximum color RGB distance between corresponding vertices. Set negative to ignore
        Scalar max_color_distance = -1;

        enum { IS_ADAPTIVEPOINTFILTER_OPTIONS = true };
      };


	  bool normal_consistence(const Point3D::VectorType &normal_p, const Point3D::VectorType &pq,
		  const Point3D::VectorType &normal_q, const Point3D::VectorType &normal_rp, 
		  const Point3D::VectorType &rpq, const Point3D::VectorType &normal_rq, const double threshold)
	  {
		  double angle_0 = atan2(normal_p.cross(pq).norm(), normal_p.dot(pq));
		  double angle_1 = atan2(normal_q.cross(pq).norm(), normal_q.dot(pq));
		  double angle_2 = atan2(normal_p.cross(normal_q).norm(), normal_p.dot(normal_q));

		  double r_angle_0 = atan2(normal_rp.cross(rpq).norm(), normal_rp.dot(rpq));
		  double r_angle_1 = atan2(normal_rq.cross(rpq).norm(), normal_rq.dot(rpq));
		  double r_angle_2 = atan2(normal_rp.cross(normal_rq).norm(), normal_rp.dot(normal_rq));

		  double max_diff = std::max(std::abs(angle_0 - r_angle_0), std::abs(angle_1 - r_angle_1));
		  max_diff = std::max(max_diff, std::abs(angle_2 - r_angle_2));
		  //if(max_diff <= threshold)
		  //{
			 // std::cout << "cur: " << angle_0 << " " << angle_1 << " " << angle_2 << std::endl;
			 // std::cout << "ref: " << r_angle_0 << " " << r_angle_1 << " " << r_angle_2 << std::endl;

		  //    std::cout << "------------------------" << std::endl;
		  //    std::cout << "np=[" << normal_p.transpose() << "]" << std::endl;
		  //    std::cout << "nb0=[" << normal_rp.transpose() << "]" << std::endl;
		  //    std::cout << "pq=[" << pq.transpose() << "]" << std::endl;
		  //    std::cout << "b01=[" << rpq.transpose() << "]" << std::endl;
		  //    std::cout << "nq=[" << normal_q.transpose() << "]" << std::endl;
		  //    std::cout << "nb1=[" << normal_rq.transpose() << "]" << std::endl;
		  //}
		  return max_diff <= threshold;
	  }

        /// Verify that the 2 points found in Q are similar to 2 of the points in the base.
        /// A filter by point feature : normal, distance, translation distance, angle and color.
        /// Return a pair of bool, according of the right addition of the pair (p,q) or (q,p) in the congruent set.
        template <typename WantedOptionsAndMore>
        inline std::pair<bool,bool> operator() (const Point3D& p,
                                                const Point3D& q,
                                                typename Point3D::Scalar pair_normals_angle,
                                                const Point3D& b0,
                                                const Point3D& b1,
                                                const WantedOptionsAndMore &options) {
            static_assert( WantedOptionsAndMore::IS_ADAPTIVEPOINTFILTER_OPTIONS,
                           "Options passed to AdaptivePointFilter must inherit AdaptivePointFilter::Options" );
            using Scalar      = typename Point3D::Scalar;
            using PairsVector = std::vector< std::pair<int, int> >;
            using VectorType  = typename Point3D::VectorType;


            std::pair<bool,bool> res{true, true};

            VectorType segment1 = (b1.pos() - b0.pos()).normalized();
			
			// value z constraint
			double diffz = p.z() - q.z();
			double ref_diffz = b0.z() - b1.z();
			res.first = fabs(diffz - ref_diffz) < 0.3;
			res.second = fabs(diffz + ref_diffz) < 0.3;

			 if (!(res.first || res.second)) return res;
			
            //if ( options.max_normal_difference > 0 &&
            //     q.normal().squaredNorm() > 0 &&
            //     p.normal().squaredNorm() > 0) {
            //    // const Scalar norm_threshold =
            //    //         0.5 * options.max_normal_difference * M_PI / 180.0;
            //    const double first_normal_angle = (q.normal() - p.normal()).norm();
            //    const double second_normal_angle = (q.normal() + p.normal()).norm();
            //    // Take the smaller normal distance.
            //    const Scalar first_norm_distance =
            //            std::min(std::abs(first_normal_angle - pair_normals_angle),
            //                     std::abs(second_normal_angle - pair_normals_angle));
            //    // Verify appropriate angle between normals and distance.
            //
            //    if (first_norm_distance > options.max_normal_difference) return {false, false};
            //}

			 //if (options.max_normal_difference > 0 &&
			 //	q.normal().squaredNorm() > 0 &&
			 //	p.normal().squaredNorm() > 0) {
			 //	// const Scalar norm_threshold =
			 //	//         0.5 * options.max_normal_difference * M_PI / 180.0;
			 //	// const double first_normal_angle = atan2(p.normal().cross(q.normal()).norm(), p.normal().dot(q.normal()));
			 //	const double first_normal_angle = (q.normal() - p.normal()).norm();
			 //	// const double second_normal_angle = (q.normal() + p.normal()).norm();
			 //	// Take the smaller normal distance.
			 //	const Scalar first_norm_distance = std::abs(first_normal_angle - pair_normals_angle);
			 //	// Verify appropriate angle between normals and distance.
			
			 //	if (first_norm_distance > options.max_normal_difference) return { false, false };
			 //}

			  //normal consistence
			 if (options.max_normal_difference > 0 && q.normal().squaredNorm() > 0 && p.normal().squaredNorm() > 0)
			 {
			 	const auto pq = (q.pos() - p.pos()).normalized();
			 	const auto b01 = (b1.pos() - b0.pos()).normalized();
			 	res.first = res.first && normal_consistence(p.normal(), pq, q.normal(), b0.normal(), b01, b1.normal(), options.max_normal_difference);
			 	res.second = res.second && normal_consistence(q.normal(), -pq, p.normal(), b0.normal(), b01, b1.normal(), options.max_normal_difference);
			 	if (!(res.first || res.second)) return res;
			 }


            // Verify restriction on the rotation angle, translation and colors.
            if (options.max_color_distance > 0) {
                const bool use_rgb = (p.rgb()[0] >= 0 && q.rgb()[0] >= 0 &&
                                      b0.rgb()[0] >= 0 &&
                                      b1.rgb()[0] >= 0);
                bool color_good = (p.rgb() - b0.rgb()).norm() <
                                  options.max_color_distance &&
                                  (q.rgb() - b1.rgb()).norm() <
                                  options.max_color_distance;

                if (use_rgb && ! color_good) return {false, false};
            }

            if (options.max_translation_distance > 0) {
                const bool dist_good = (p.pos() - b0.pos()).norm() <
                                       options.max_translation_distance &&
                                       (q.pos() - b1.pos()).norm() <
                                       options.max_translation_distance;
                if (! dist_good) return {false, false};
            }

            // need cleaning here
            if (options.max_angle > 0){
                VectorType segment2 = (q.pos() - p.pos()).normalized();
				res.second = res.second && (std::acos(segment1.dot(segment2)) <= options.max_angle * M_PI / 180.0);
				res.first = res.first && (std::acos(segment1.dot(-segment2)) <= options.max_angle * M_PI / 180.0);
            }
            return res;
        }
    };
}

#endif //OPENGR_FUNCTORFEATUREPOINTTEST_H
