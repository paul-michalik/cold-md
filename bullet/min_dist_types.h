#pragma once

#include "cold-md/bullet/min_dist_bullet.h"
#include <boost/scope_exit.hpp>
#include <limits>
#include <vector>
#include <algorithm>
#include <unordered_map>

namespace cold {
    namespace bullet {
        enum class output_kind {
            undefined,
            collision,
            free
        };

        inline char const* to_string(output_kind const& kind_)
        {
            switch (kind_) {
                case output_kind::collision: return "collision";
                case output_kind::free: return "free";
                default:
                    return "undefined";
            }
        }

        struct min_dist_output {
            btCollisionObject const* obj_a = nullptr;
            btCollisionObject const* obj_b = nullptr;

            btVector3 point_on_a;
            btVector3 point_on_b;

            btVector3 triangle_of_a[3];
            btVector3 triangle_of_b[3];

            // Minimum distance. After initialization will be equal to cur_dist_interval.first.
            btScalar distance = std::numeric_limits<btScalar>::max();

            // Used for internal pruning tests. Represents the lower and upper bound
            // of current distance intervals during the computation for respective objects.
            // Initialized to respective max, min values of current floating point models
            // so that it can be used as a initial value for pruning operations.
            std::pair<btScalar, btScalar> cur_dist_interval = {
                +std::numeric_limits<btScalar>::max(),
                +std::numeric_limits<btScalar>::max()
            };

            output_kind kind = output_kind::undefined;

            inline friend std::ostream& operator << (std::ostream& out_, min_dist_output const& md_res_)
            {
                return out_ << typeid(md_res_).name() << ": " << &md_res_ << std::endl <<
                    "obj_a:             " << md_res_.obj_a << std::endl <<
                    "obj_b:             " << md_res_.obj_b << std::endl <<
                    "point_on_a:        " << md_res_.point_on_a << std::endl <<
                    "point_on_b:        " << md_res_.point_on_b << std::endl <<
                    "triangle_of_a:     " << md_res_.triangle_of_a[0] << ", " << md_res_.triangle_of_a[1] << ", " << md_res_.triangle_of_a[2] << std::endl <<
                    "triangle_of_b:     " << md_res_.triangle_of_b[0] << ", " << md_res_.triangle_of_b[1] << ", " << md_res_.triangle_of_b[2] << std::endl <<
                    "distance:          " << md_res_.distance << std::endl <<
                    "cur_dist_interval: " << md_res_.cur_dist_interval.first << ", " << md_res_.cur_dist_interval.second << std::endl <<
                    "kind:              " << to_string(md_res_.kind);
            }
        };

        using min_dist_output_matrix = std::vector<min_dist_output>;

        // Bullet requires to lock the triangle containers before retrieving triangle data.
        // Therefore, it is very expensive to do when triangles pairs are actually needed and we
        // must collect all triangles in a repository before the actual operation starts...
        class triangle_repository {
            using triangle_set_t = std::unordered_map<int, btTriangleShapeEx>;

            btGImpactBoxSet const* _part_boxes_a;
            btGImpactBoxSet const* _part_boxes_b;
            triangle_set_t _triangles_a;
            triangle_set_t _triangles_b;

            static triangle_set_t retrieve_triangles(btGImpactMeshShapePart const* part_, btTransform const& transf_)
            {
                triangle_set_t triangles;

                part_->lockChildShapes();

                BOOST_SCOPE_EXIT(part_)
                {
                    part_->unlockChildShapes();
                } BOOST_SCOPE_EXIT_END;

                auto part_boxes = part_->getBoxSet();
                btTriangleShapeEx tri;
                for (auto i = 0, n = part_boxes->getNodeCount(); i < n; ++i) {
                    if (part_boxes->isLeafNode(i)) {
                        part_->getBulletTriangle(part_boxes->getNodeData(i), tri);
                        tri.applyTransform(transf_);
                        triangles[i] = tri;
                    }
                }

                return std::move(triangles);
            }
        public:
            triangle_repository(
                btTransform const& transf_part_a_,
                btTransform const& transf_part_b_,
                btGImpactMeshShapePart const* part_a_,
                btGImpactMeshShapePart const* part_b_)
                : _part_boxes_a(part_a_->getBoxSet())
                , _part_boxes_b(part_b_->getBoxSet())
                , _triangles_a(retrieve_triangles(part_a_, transf_part_a_))
                , _triangles_b(retrieve_triangles(part_b_, transf_part_b_))
            {
            }

            // Tries to retrieve the triangle from btGImpactBoxSet with index key without actually accessing the box set.
            bool try_get_triangle(btGImpactBoxSet const* part_boxes_, int key_, btTriangleShapeEx& tri_) const
            {
                if (part_boxes_ == _part_boxes_a || part_boxes_ == _part_boxes_b) {
                    auto const& triangles = (part_boxes_ == _part_boxes_a ? _triangles_a : _triangles_b);
                    auto tri_itr = triangles.find(key_);
                    if (tri_itr != triangles.end()) {
                        tri_ = tri_itr->second;
                        return true;
                    }
                }

                return false;
            }

            // Retrieves the triangle from btGImpactBoxSet with index key without actually accessing the box set. Throws if not possible.
            btTriangleShapeEx const& get_triangle(btGImpactBoxSet const* part_boxes_, int key_) const
            {
                if (part_boxes_ == _part_boxes_a || part_boxes_ == _part_boxes_b) {
                    auto const& triangles = (part_boxes_ == _part_boxes_a ? _triangles_a : _triangles_b);
                    return triangles.at(key_);
                }

                throw std::invalid_argument(__FUNCTION__);
            }
        };
    }
}