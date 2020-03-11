//
// Created by Sandra Alfaro on 24/05/18.
//

#include <vector>
#include <chrono>
#include <atomic>
#include <Eigen/Geometry>                 // MatrixBase.homogeneous()
#include <Eigen/SVD>
#include <Eigen/Core>                     // Transform.computeRotationScaling()


#ifdef OpenGR_USE_OPENMP
#include <omp.h>
#endif

#include "shared.h"
#include "sampling.h"
#include "kdtree.h"
#include "logger.h"
#include "match4pcsBase.h"

#ifdef TEST_GLOBAL_TIMINGS
#   include "gr/utils/timer.h"
#endif

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyDataWriter.h>

namespace gr {
    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::Match4pcsBase (const OptionsType& options
            , const Utils::Logger& logger)
            : MatchBaseType(options,logger)
            , fun_(MatchBaseType::sampled_Q_3D_,MatchBaseType::base_3D_,MatchBaseType::options_)
    {
    }

    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::~Match4pcsBase() {}

    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    bool Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::TryQuadrilateral(
        typename Point3D::Scalar &invariant1,
        typename Point3D::Scalar &invariant2,
        int &id1, int &id2, int &id3, int &id4) {

        Scalar min_distance = std::numeric_limits<Scalar>::max();
        int best1, best2, best3, best4;
        best1 = best2 = best3 = best4 = -1;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (i == j) continue;
                int k = 0;
                while (k == i || k == j) k++;
                int l = 0;
                while (l == i || l == j || l == k) l++;
                Scalar local_invariant1;
                Scalar local_invariant2;
                // Compute the closest points on both segments, the corresponding
                // invariants and the distance between the closest points.
                Scalar segment_distance = distSegmentToSegment(
                        MatchBaseType::base_3D_[i].pos(), MatchBaseType::base_3D_[j].pos(),
                        MatchBaseType::base_3D_[k].pos(), MatchBaseType::base_3D_[l].pos(),
                        local_invariant1, local_invariant2);
                // Retail the smallest distance and the best order so far.
                if (segment_distance < min_distance) {
                    min_distance = segment_distance;
                    best1 = i;
                    best2 = j;
                    best3 = k;
                    best4 = l;
                    invariant1 = local_invariant1;
                    invariant2 = local_invariant2;
                }
            }
        }

        if(best1 < 0 || best2 < 0 || best3 < 0 || best4 < 0 ) return false;

        Coordinates tmp = MatchBaseType::base_3D_;
        MatchBaseType::base_3D_[0] = tmp[best1];
        MatchBaseType::base_3D_[1] = tmp[best2];
        MatchBaseType::base_3D_[2] = tmp[best3];
        MatchBaseType::base_3D_[3] = tmp[best4];

        CongruentBaseType tmpId = {id1, id2, id3, id4};
        id1 = tmpId[best1];
        id2 = tmpId[best2];
        id3 = tmpId[best3];
        id4 = tmpId[best4];

        return true;
    }

    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    bool Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::SelectQuadrilateral(
        Scalar &invariant1,
        Scalar &invariant2,
        int& base1, int& base2, int& base3, int& base4)  {

        const Scalar kBaseTooSmall (0.2);
        int current_trial = 0;

        // Try fix number of times.
        while (current_trial < MatchBaseType::kNumberOfDiameterTrials) {
            // Select a triangle if possible. otherwise fail.
            if (!MatchBaseType::SelectRandomTriangle(base1, base2, base3)){
                return false;
            }

            const auto& b0 = MatchBaseType::base_3D_[0] = MatchBaseType::sampled_P_3D_[base1];
            const auto& b1 = MatchBaseType::base_3D_[1] = MatchBaseType::sampled_P_3D_[base2];
            const auto& b2 = MatchBaseType::base_3D_[2] = MatchBaseType::sampled_P_3D_[base3];

            // The 4th point will be a one that is close to be planar to the other 3
            // while still not too close to them.
            const double x1 = b0.x();
            const double y1 = b0.y();
            const double z1 = b0.z();
            const double x2 = b1.x();
            const double y2 = b1.y();
            const double z2 = b1.z();
            const double x3 = b2.x();
            const double y3 = b2.y();
            const double z3 = b2.z();

            // Fit a plan.
            Scalar denom = (-x3 * y2 * z1 + x2 * y3 * z1 + x3 * y1 * z2 - x1 * y3 * z2 -
                            x2 * y1 * z3 + x1 * y2 * z3);

            if (denom != 0) {
                Scalar A =
                        (-y2 * z1 + y3 * z1 + y1 * z2 - y3 * z2 - y1 * z3 + y2 * z3) / denom;
                Scalar B =
                        (x2 * z1 - x3 * z1 - x1 * z2 + x3 * z2 + x1 * z3 - x2 * z3) / denom;
                Scalar C =
                        (-x2 * y1 + x3 * y1 + x1 * y2 - x3 * y2 - x1 * y3 + x2 * y3) / denom;
                base4 = -1;
                Scalar best_distance = std::numeric_limits<Scalar>::max();
                // Go over all points in P.
                const Scalar too_small = std::pow(MatchBaseType::max_base_diameter_ * kBaseTooSmall, 2);
                for (unsigned int i = 0; i < MatchBaseType::sampled_P_3D_.size(); ++i) {
                    const auto &p = MatchBaseType::sampled_P_3D_[i];
                    if ((p.pos() - b0.pos()).squaredNorm() >= too_small &&
                        (p.pos() - b1.pos()).squaredNorm() >= too_small &&
                        (p.pos() - b2.pos()).squaredNorm() >= too_small) {
                        // Not too close to any of the first 3.
                        const Scalar distance =
                                std::abs(A * p.x() + B * p.y() + C * p.z() - 1.0);
                        // Search for the most planar.
                        if (distance < best_distance) {
                            best_distance = distance;
                            base4 = int(i);
                        }
                    }
                }
                // If we have a good one we can quit.
                if (base4 != -1) {
                    MatchBaseType::base_3D_[3] = MatchBaseType::sampled_P_3D_[base4];
                    if(TryQuadrilateral(invariant1, invariant2, base1, base2, base3, base4))
                        return true;
                }
            }
            current_trial++;
        }

        // We failed to find good enough base..
        return false;
    }

    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    // Initialize all internal data structures and data members.
    void Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::Initialize(
        const std::vector<Point3D>& P,
        const std::vector<Point3D>& Q) {
        fun_.Initialize(P,Q);
    }


    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    bool Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::generateCongruents (
        CongruentBaseType &base, Set& congruent_quads) {
//      std::cout << "------------------" << std::endl;

      Scalar invariant1, invariant2;
//#define STATIC_BASE

#ifdef STATIC_BASE
  static bool first_time = true;

  if (first_time){
      std::cerr << "Warning: Running with static base" << std::endl;
      base[0] = 0;
      base[1] = 3;
      base[2] = 1;
      base[3] = 4;

      MatchBaseType::base_3D_[0] = MatchBaseType::sampled_P_3D_ [base[0]];
      MatchBaseType::base_3D_[1] = MatchBaseType::sampled_P_3D_ [base[1]];
      MatchBaseType::base_3D_[2] = MatchBaseType::sampled_P_3D_ [base[2]];
      MatchBaseType::base_3D_[3] = MatchBaseType::sampled_P_3D_ [base[3]];
      TryQuadrilateral(invariant1, invariant2, base[0], base[1], base[2], base[3]);

      first_time = false;
  }
  else
      return false;

#else
        if (!SelectQuadrilateral(invariant1, invariant2, base[0], base[1],
                                 base[2], base[3])) {
//            std::cout << "Skipping wrong base" << std::endl;
            return false;
        }
#endif
//        std::cout << "Found a new base !" << std::endl;
        const auto& b0 = MatchBaseType::base_3D_[0];
        const auto& b1 = MatchBaseType::base_3D_[1];
        const auto& b2 = MatchBaseType::base_3D_[2];
        const auto& b3 = MatchBaseType::base_3D_[3];

        // Computes distance between pairs.
        const Scalar distance1 = (b0.pos()- b1.pos()).norm();
        const Scalar distance2 = (b2.pos()- b3.pos()).norm();

        std::vector<std::pair<int, int>> pairs1, pairs2;

        // Compute normal angles.
        const Scalar normal_angle1 = (b0.normal() - b1.normal()).norm();
        const Scalar normal_angle2 = (b2.normal() - b3.normal()).norm();

		fun_.ExtractPairs(distance1, normal_angle1, MatchBaseType::options_.delta, 0, 1, &pairs1);
		if (pairs1.size() < 2000)
		{
			fun_.ExtractPairs(distance1, normal_angle1, MatchBaseType::distance_factor * MatchBaseType::options_.delta, 0, 1, &pairs1);
		}
		fun_.ExtractPairs(distance2, normal_angle2, MatchBaseType::options_.delta, 2, 3, &pairs2);
		if (pairs2.size() < 2000)
		{
			fun_.ExtractPairs(distance2, normal_angle2, MatchBaseType::distance_factor * MatchBaseType::options_.delta, 2, 3, &pairs2);
		}

//        std::cout << "Pair set 1 has " << pairs1.size() << " elements" << std::endl;
//        std::cout << "Pair set 2 has " << pairs2.size() << " elements" << std::endl;

//  Log<LogLevel::Verbose>( "Pair creation ouput: ", pairs1.size(), " - ", pairs2.size());

        if (pairs1.size() == 0 || pairs2.size() == 0) {
            return false;
        }
#ifdef ZSW_DEBUG
		static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
		static bool first_time = true;
		if(first_time)
		{
			first_time = false;
			// sample p points
			{
				vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
				vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
				vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
				for (auto &pt : MatchBaseType::sampled_P_3D_)
				{
					auto id = points_data->InsertNextPoint(pt.x(), pt.y(), pt.z());
					vertices->InsertNextCell(1);
					vertices->InsertCellPoint(id);
				}

				vtk_points_cloud->SetVerts(vertices);
				vtk_points_cloud->SetPoints(points_data);
				poly_data_writer->SetInputData(vtk_points_cloud);
				poly_data_writer->SetFileName("F:/data/tmp/4pcs_res/sample_p.vtk");
				poly_data_writer->SetFileTypeToBinary();
				poly_data_writer->Write();
			}

			// sample q points
			{
				vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
				vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
				vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
				for (auto &pt : MatchBaseType::sampled_Q_3D_)
				{
					auto id = points_data->InsertNextPoint(pt.x(), pt.y(), pt.z());
					vertices->InsertNextCell(1);
					vertices->InsertCellPoint(id);
				}
				vtk_points_cloud->SetVerts(vertices);
				vtk_points_cloud->SetPoints(points_data);
				poly_data_writer->SetInputData(vtk_points_cloud);
				poly_data_writer->SetFileName("F:/data/tmp/4pcs_res/sample_q.vtk");
				poly_data_writer->SetFileTypeToBinary();
				poly_data_writer->Write();
			}
		}

		static int b_index = -1;
		++b_index;
		{
			vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
			vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
			vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
			vtk_normals->SetNumberOfComponents(3);
			auto id = points_data->InsertNextPoint(b0.x(), b0.y(), b0.z());
			vertices->InsertNextCell(1);
			vertices->InsertCellPoint(id);
			vtk_normals->InsertNextTuple3(b0.normal()[0], b0.normal()[1], b0.normal()[2]);

			id = points_data->InsertNextPoint(b1.x(), b1.y(), b1.z());
			vertices->InsertNextCell(1);
			vertices->InsertCellPoint(id);
			vtk_normals->InsertNextTuple3(b1.normal()[0], b1.normal()[1], b1.normal()[2]);

			id = points_data->InsertNextPoint(b2.x(), b2.y(), b2.z());
			vertices->InsertNextCell(1);
			vertices->InsertCellPoint(id);
			vtk_normals->InsertNextTuple3(b2.normal()[0], b2.normal()[1], b2.normal()[2]);
			
			id = points_data->InsertNextPoint(b3.x(), b3.y(), b3.z());
			vertices->InsertNextCell(1);
			vertices->InsertCellPoint(id);
			vtk_normals->InsertNextTuple3(b3.normal()[0], b3.normal()[1], b3.normal()[2]);

			vtk_points_cloud->SetVerts(vertices);
			vtk_points_cloud->SetPoints(points_data);
			vtk_points_cloud->GetCellData()->SetNormals(vtk_normals);

			poly_data_writer->SetInputData(vtk_points_cloud);
			std::stringstream ss;
			ss << "F:/data/tmp/4pcs_res/base_" << setfill('0') << setw(3) << b_index << ".vtk";
			poly_data_writer->SetFileName(ss.str().c_str());
			poly_data_writer->SetFileTypeToBinary();
			poly_data_writer->Write();
		}

#endif
		std::cout << "start FindCongruentQuadrilaterals" << std::endl;
		std::chrono::high_resolution_clock::time_point tp0 = std::chrono::high_resolution_clock::now();
        if (!fun_.FindCongruentQuadrilaterals(invariant1, invariant2,
                                         MatchBaseType::distance_factor * MatchBaseType::options_.delta,
                                         MatchBaseType::distance_factor * MatchBaseType::options_.delta,
                                         pairs1,
                                         pairs2,
                                         &congruent_quads)) {
			std::cout << "FindCongruentQuadrilaterals failed, time cost:"
				<< std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp0).count()
				<< " ms" << std::endl;
            return false;
        }
		std::cout << "FindCongruentQuadrilaterals suc, time cost:"
			<< std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp0).count()
			<< " ms" << std::endl;
		std::cout << "congruent pairs1:" << pairs1.size() << std::endl;
		std::cout << "congruent pairs2:" << pairs2.size() << std::endl;
		std::cout << "congruent_quads:" << congruent_quads.size() << std::endl;

#ifdef ZSW_DEBUG1		
		// TODO ZSW output the 4 points and the congruent
		{
			// for
			int c_index = 0;
			for(auto &cur_array : congruent_quads)
			{
				auto cong_0 = MatchBaseType::sampled_Q_3D_[cur_array[0]];
				auto cong_1 = MatchBaseType::sampled_Q_3D_[cur_array[1]];
				auto cong_2 = MatchBaseType::sampled_Q_3D_[cur_array[2]];
				auto cong_3 = MatchBaseType::sampled_Q_3D_[cur_array[3]];

				vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
				vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
				vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
				vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
				vtk_normals->SetNumberOfComponents(3);
				vtkSmartPointer<vtkFloatArray> index = vtkSmartPointer<vtkFloatArray>::New();
				index->SetName("index");

				auto id = points_data->InsertNextPoint(cong_0.x(), cong_0.y(), cong_0.z());
				vertices->InsertNextCell(1);
				vertices->InsertCellPoint(id);
				vtk_normals->InsertNextTuple3(cong_0.normal()[0], cong_0.normal()[1], cong_0.normal()[2]);

				id = points_data->InsertNextPoint(cong_1.x(), cong_1.y(), cong_1.z());
				vertices->InsertNextCell(1);
				vertices->InsertCellPoint(id);
				vtk_normals->InsertNextTuple3(cong_1.normal()[0], cong_1.normal()[1], cong_1.normal()[2]);

				id = points_data->InsertNextPoint(cong_2.x(), cong_2.y(), cong_2.z());
				vertices->InsertNextCell(1);
				vertices->InsertCellPoint(id);
				vtk_normals->InsertNextTuple3(cong_2.normal()[0], cong_2.normal()[1], cong_2.normal()[2]);
				
				id = points_data->InsertNextPoint(cong_3.x(), cong_3.y(), cong_3.z());
				vertices->InsertNextCell(1);
				vertices->InsertCellPoint(id);
				vtk_normals->InsertNextTuple3(cong_3.normal()[0], cong_3.normal()[1], cong_3.normal()[2]);

				index->InsertNextTuple1(cur_array[0]);
				index->InsertNextTuple1(cur_array[1]);
				index->InsertNextTuple1(cur_array[2]);
				index->InsertNextTuple1(cur_array[3]);

				vtk_points_cloud->SetVerts(vertices);
				vtk_points_cloud->SetPoints(points_data);
				vtk_points_cloud->GetPointData()->SetScalars(index);
				vtk_points_cloud->GetCellData()->SetNormals(vtk_normals);

				std::stringstream ss;
				ss << "F:/data/tmp/4pcs_res/con_" << setfill('0') << setw(3) << b_index << "_"  << c_index <<  ".vtk";
				poly_data_writer->SetInputData(vtk_points_cloud);
				poly_data_writer->SetFileName(ss.str().c_str());
				poly_data_writer->SetFileTypeToBinary();
				poly_data_writer->Write();
				++c_index;
			}
			
		}
#endif
    	return true;
    }

    template <template <typename, typename> typename _Functor,
              typename TransformVisitor,
              typename PairFilteringFunctor,
              template < class, class > typename PFO>
    typename Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::Scalar
    Match4pcsBase<_Functor, TransformVisitor, PairFilteringFunctor, PFO>::distSegmentToSegment(
        const VectorType& p1, const VectorType& p2,
        const VectorType& q1, const VectorType& q2,
        Scalar& invariant1, Scalar& invariant2) {

        static const Scalar kSmallNumber = 0.0001;
        VectorType u = p2 - p1;
        VectorType v = q2 - q1;
        VectorType w = p1 - q1;
        Scalar a = u.dot(u);
        Scalar b = u.dot(v);
        Scalar c = v.dot(v);
        Scalar d = u.dot(w);
        Scalar e = v.dot(w);
        Scalar f = a * c - b * b;
        // s1,s2 and t1,t2 are the parametric representation of the intersection.
        // they will be the invariants at the end of this simple computation.
        Scalar s1 = 0.0;
        Scalar s2 = f;
        Scalar t1 = 0.0;
        Scalar t2 = f;

        if (f < kSmallNumber) {
            s1 = 0.0;
            s2 = 1.0;
            t1 = e;
            t2 = c;
        } else {
            s1 = (b * e - c * d);
            t1 = (a * e - b * d);
            if (s1 < 0.0) {
                s1 = 0.0;
                t1 = e;
                t2 = c;
            } else if (s1 > s2) {
                s1 = s2;
                t1 = e + b;
                t2 = c;
            }
        }

        if (t1 < 0.0) {
            t1 = 0.0;
            if (-d < 0.0)
                s1 = 0.0;
            else if (-d > a)
                s1 = s2;
            else {
                s1 = -d;
                s2 = a;
            }
        } else if (t1 > t2) {
            t1 = t2;
            if ((-d + b) < 0.0)
                s1 = 0;
            else if ((-d + b) > a)
                s1 = s2;
            else {
                s1 = (-d + b);
                s2 = a;
            }
        }
        invariant1 = (std::abs(s1) < kSmallNumber ? 0.0 : s1 / s2);
        invariant2 = (std::abs(t1) < kSmallNumber ? 0.0 : t1 / t2);

        return ( w + (invariant1 * u) - (invariant2 * v)).norm();
    }
}

