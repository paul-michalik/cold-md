#pragma once

#include <cold-md/bullet/min_dist_bullet.h>
#include <cold-md/bullet/min_dist_types.h>
#include <algorithm>

namespace cold {
    namespace bullet {
        class min_dist_traits {
            static btConvexPenetrationDepthSolver& get_depth_solver()
            {
                static btGjkEpaPenetrationDepthSolver _s_epa_solver;
                return _s_epa_solver;
            }

            static void calc_minimum_distance_raw(
                btConvexShape const* shape_a_,
                btConvexShape const* shape_b_,
                btTransform const& transf_a_,
                btTransform const& transf_b_,
                btPointCollector& output)
            {
                // prototype: void btConvexConvexAlgorithm::processCollision
                btVoronoiSimplexSolver gjk_simplex_solver;
                btGjkPairDetector::ClosestPointInput input;

                btGjkPairDetector	gjk_pair_detector(
                    shape_a_, shape_b_,
                    &gjk_simplex_solver,
                    &get_depth_solver());

                input.m_transformA = transf_a_;
                input.m_transformB = transf_b_;

                gjk_pair_detector.getClosestPoints(input, output, nullptr);
            }
        public:
            // ...for boxes in world coordinate system. Precondition: Boxes do not collide
            static min_dist_output calc_lower_upper_bound(
                btAABB const& box_a_,
                btAABB const& box_b_,
                btScalar const& margin_)
            {
                // 1. Convert bounding boxes to convex shapes. Since AABBs have 
                // constant orintation, we only need to set proper position:
                btVector3 center_a, extend_a;
                box_a_.get_center_extend(center_a, extend_a);
                btBoxShape box_shape_a(
                    btVector3(extend_a.getX(), extend_a.getY(), extend_a.getZ()));
                box_shape_a.setMargin(margin_);

                btVector3 center_b, extend_b;
                box_b_.get_center_extend(center_b, extend_b);
                btBoxShape box_shape_b(
                    btVector3(extend_b.getX(), extend_b.getY(), extend_b.getZ()));
                box_shape_b.setMargin(margin_);

                static const btQuaternion c_nulltransf(0., 0., 0.);
                btTransform transf_a(c_nulltransf, center_a);
                btTransform transf_b(c_nulltransf, center_b);

                btPointCollector pc_output;
                calc_minimum_distance_raw(
                    &box_shape_a, &box_shape_b,
                    transf_a, transf_b,
                    pc_output);

                // 2. Calculate lower/upper bound:
                min_dist_output cur_result;

                cur_result.point_on_b =
                    pc_output.m_pointInWorld;
                cur_result.point_on_a =
                    pc_output.m_pointInWorld + pc_output.m_normalOnBInWorld * pc_output.m_distance;
                cur_result.distance = pc_output.m_distance;

                // lower bound:
                cur_result.cur_dist_interval.first = cur_result.distance;

                // upper bound: This policy calculates the upper bound 
                // as a maximum distance to vertices of other box:
                // TODO: Optimization, it'd probably good enough to take the 
                // maxnorm of distances to the closest plance of the other box...
                cur_result.cur_dist_interval.second = cur_result.distance;
                btVector3 v;

                // a->b
                for (auto idx_v = 0, size_v = box_shape_b.getNumVertices(); idx_v < size_v; ++idx_v) {
                    box_shape_b.getVertex(idx_v, v);
                    v = transf_b * v;

                    cur_result.cur_dist_interval.second = std::max(
                        cur_result.cur_dist_interval.second,
                        cur_result.point_on_a.distance(v));
                }
                // b->a
                for (auto idx_v = 0, size_v = box_shape_a.getNumVertices(); idx_v < size_v; ++idx_v) {
                    box_shape_a.getVertex(idx_v, v);
                    v = transf_a * v;

                    cur_result.cur_dist_interval.second = std::max(
                        cur_result.cur_dist_interval.second,
                        cur_result.point_on_b.distance(v));
                }

                return cur_result;
            }

            // ...for triangles in world coordinate system. 
            static min_dist_output calc_lower_upper_bound(
                btTriangleShape const& tri_a_,
                btTriangleShape const& tri_b_,
                btScalar const& margin_)
            {
                static const btTransform null_transf{
                    btQuaternion(0, 0, 0), btVector3(0, 0, 0)
                };

                btPointCollector pc_output;
                calc_minimum_distance_raw(
                    &tri_a_, &tri_b_,
                    null_transf,
                    null_transf, pc_output);

                min_dist_output cur_result;

                cur_result.point_on_b =
                    pc_output.m_pointInWorld;
                cur_result.point_on_a =
                    pc_output.m_pointInWorld + pc_output.m_normalOnBInWorld * pc_output.m_distance;
                cur_result.distance = pc_output.m_distance;

                // logical correctness of the result:
                if (pc_output.m_distance < 0.) {
                    // I want distances to be positive...
                    pc_output.m_distance = 0.;

                    cur_result.kind = output_kind::collision;
                    std::copy_n(&tri_a_.getVertexPtr(0), 3, cur_result.triangle_of_a);
                    std::copy_n(&tri_b_.getVertexPtr(0), 3, cur_result.triangle_of_b);
                } else {
                    cur_result.kind = output_kind::free;
                }

                // lower/upper bound:
                cur_result.cur_dist_interval.first =
                    cur_result.cur_dist_interval.second = cur_result.distance;

                return cur_result;
            }
        };
    }
}