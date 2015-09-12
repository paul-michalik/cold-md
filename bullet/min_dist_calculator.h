#pragma once

#include <cold-md/bullet/min_dist_bullet.h>
#include <cold-md/bullet/min_dist_types.h>
#include <cold-md/bullet/min_dist_traits.h>

namespace cold {
    namespace bullet {

        template<class TMinDistTraits = ::cold::bullet::min_dist_traits>
        class min_dist_calculator {
            btCollisionWorld* _world;

            bool does_pass_lower_upper_bound_test(
                const btGImpactQuantizedBvh * boxes_a_, const btGImpactQuantizedBvh * boxes_b_,
                int box_idx_a, int box_idx_b,
                btTransform const& transf_a_, btTransform const& transf_b_,
                btScalar const& margin_,
                min_dist_output& global_res_)
            {
                btAABB box_a;
                boxes_a_->getNodeBound(box_idx_a, box_a);

                btAABB box_b;
                boxes_b_->getNodeBound(box_idx_b, box_b);

                // transform into world coordinates:
                box_a.appy_transform(transf_a_);
                box_b.appy_transform(transf_b_);

                // If boxes collide, no decision about the lower and upper bounds can be made,
                // neither can this pair of boxes contribute to global lower/upper bound:
                if (!box_a.has_collision(box_b)) {
                    // perform_distance_calculation lower/upper bound distances:
                    min_dist_output cur_res =
                        TMinDistTraits::calc_lower_upper_bound(box_a, box_b, margin_);

                    // The test passes (the box can be pruned) if the lower bound of this 
                    // box pair is greater than the current upper bound...
                    if (global_res_.cur_dist_interval.second < cur_res.cur_dist_interval.first) {
                        return true;
                    } else {
                        // ...otherwise update current interval to minimum of each bound.
                        // This lets the current interval grow "to the left" but never to the right...
                        global_res_.cur_dist_interval.first = std::min(
                            global_res_.cur_dist_interval.first,
                            cur_res.cur_dist_interval.first);

                        global_res_.cur_dist_interval.second = std::min(
                            global_res_.cur_dist_interval.second,
                            cur_res.cur_dist_interval.second);
                    }
                    // ...otherwise, if the lower-upper bounds intervals intersect,
                    // nothing meanigfull can be done, just leave the values as they are...
                }

                return false;
            }

            void gimpact_boxset_vs_gimpact_boxset_rec(
                const btGImpactQuantizedBvh * boxes_a_, const btGImpactQuantizedBvh * boxes_b_,
                int box_idx_a_, int box_idx_b_,
                btTransform const& transf_a_, btTransform const& transf_b_,
                triangle_repository const& triangles_,
                btScalar const& margin_,
                min_dist_output& global_res_)
            {
                // prototype: _find_quantized_collision_pairs_recursive...

                // bail out... doesn't make sense to continue...
                if (output_kind::collision == global_res_.kind) {
                    return;
                }

                auto const box_a_is_leaf = boxes_a_->isLeafNode(box_idx_a_);
                auto const box_b_is_leaf = boxes_b_->isLeafNode(box_idx_b_);

                // internal node pruning. If this box pair passes the lower/upper bound test
                // we can safely skip it along all their children from the set of candidates 
                // which contain triangles with minimal distance...
                if (!box_a_is_leaf && 
                    !box_b_is_leaf && 
                    does_pass_lower_upper_bound_test(
                        boxes_a_, boxes_b_,
                        box_idx_a_, box_idx_b_,
                        transf_a_, transf_b_,
                        margin_,
                        global_res_)) 
                {
                    return;
                }

                if (box_a_is_leaf) {
                    if (box_b_is_leaf) {
                        // calculate minima and bounds! Triangles in triangle_repo are already in world coordinates! 
                        // TODO: This isn't a good idea, since I'm doing a lot work to transform 
                        // triangles which won't be tested at all...

                        auto cur_result = TMinDistTraits::calc_lower_upper_bound(
                            triangles_.get_triangle(boxes_a_, box_idx_a_),
                            triangles_.get_triangle(boxes_b_, box_idx_b_),
                            margin_);

                        if (output_kind::collision == cur_result.kind) {
                            global_res_.kind = cur_result.kind;
                            global_res_.distance = cur_result.distance;
                            global_res_.point_on_a = cur_result.point_on_a;
                            global_res_.point_on_b = cur_result.point_on_b;
                            std::copy_n(cur_result.triangle_of_a, 3, global_res_.triangle_of_a);
                            std::copy_n(cur_result.triangle_of_b, 3, global_res_.triangle_of_b);

                            global_res_.cur_dist_interval.first =
                                global_res_.cur_dist_interval.second = 0.;
                        } else if (output_kind::free == cur_result.kind) {
                            // update distance, points and bounds!
                            if (cur_result.distance < global_res_.distance) {
                                global_res_.kind = cur_result.kind;
                                global_res_.distance = cur_result.distance;
                                global_res_.point_on_a = cur_result.point_on_a;
                                global_res_.point_on_b = cur_result.point_on_b;

                                global_res_.cur_dist_interval.first = std::min(
                                    global_res_.cur_dist_interval.first,
                                    cur_result.distance);

                                global_res_.cur_dist_interval.second = std::min(
                                    global_res_.cur_dist_interval.second,
                                    cur_result.distance);
                            }
                        }
                    } else {

                        // descend left recursive:
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            box_idx_a_, boxes_b_->getLeftNode(box_idx_b_),
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);

                        // descend right recursive
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            box_idx_a_, boxes_b_->getRightNode(box_idx_b_),
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);
                    }
                } else {
                    if (box_b_is_leaf) {

                        // descend left recursive:
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            boxes_a_->getLeftNode(box_idx_a_), box_idx_b_,
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);

                        // descend right recursive:

                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            boxes_a_->getRightNode(box_idx_a_), box_idx_b_,
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);
                    } else {
                        // descend left0 left1
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            boxes_a_->getLeftNode(box_idx_a_), boxes_b_->getLeftNode(box_idx_b_),
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);

                        // descend left0 right1
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            boxes_a_->getLeftNode(box_idx_a_), boxes_b_->getRightNode(box_idx_b_),
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);

                        // descend right0 left1
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            boxes_a_->getRightNode(box_idx_a_), boxes_b_->getLeftNode(box_idx_b_),
                            transf_a_, transf_b_,
                            triangles_,
                            margin_,
                            global_res_);

                        // descend right0 right1
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            boxes_a_, boxes_b_,
                            boxes_a_->getRightNode(box_idx_a_), boxes_b_->getRightNode(box_idx_b_),
                            transf_a_, transf_b_,
                            triangles_,
                            margin_, 
                            global_res_);

                    }// else if box_idx_b_ is not a leaf
                }// else if box_idx_a_ is not a leaf
            }

            void gimpact_mesh_part_vs_gimpact_mesh_part(
                const btTransform & trans_a_,
                const btTransform & trans_b_,
                const btGImpactMeshShapePart* shape_part_a_,
                const btGImpactMeshShapePart* shape_part_b_,
                min_dist_output& global_res_)
            {
                // almost there!
                // prototype: gimpact_vs_gimpact_find_pairs...

                if (shape_part_a_->hasBoxSet() && shape_part_b_->hasBoxSet()) {
                    // prototype: btGImpactQuantizedBvh::find_collision

                    auto box_set_a = btGetBoxSet(shape_part_a_);
                    auto box_set_b = btGetBoxSet(shape_part_b_);

                    if (box_set_a->getNodeCount() > 0 && box_set_b->getNodeCount() > 0) {
                        
                        // retrieve all triangles. Since the container must be locked
                        // it would be really expensive to query the data dynamically...
                        triangle_repository triangles {
                            trans_a_,
                            trans_b_,
                            shape_part_a_,
                            shape_part_b_
                        };

                        // from now on, we could fire up parallel tasks. 
                        // Only need to take care of global_res_...
                        gimpact_boxset_vs_gimpact_boxset_rec(
                            box_set_a, box_set_b,
                            0, 0,
                            trans_a_, trans_b_,
                            triangles,
                            std::min(shape_part_a_->getMargin(), shape_part_b_->getMargin()),
                            global_res_);
                    }

                } else { // what...?
                    assert(false);
                }
            }

            // This method is invoked recursively until we arrive at two mesh parts.
            void gimpact_vs_gimpact(
                const btCollisionObjectWrapper* obj_wrap_a_,
                const btCollisionObjectWrapper* obj_wrap_b_,
                const btGImpactShapeInterface* shape_a_,
                const btGImpactShapeInterface* shape_b_,
                min_dist_output& global_res_)
            {
                if (shape_a_->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE) {
                    auto meshshape_a = static_cast<const btGImpactMeshShape *>(shape_a_);
                    auto mpcount_a = meshshape_a->getMeshPartCount();

                    while (mpcount_a--) {
                        gimpact_vs_gimpact(
                            obj_wrap_a_,
                            obj_wrap_b_,
                            meshshape_a->getMeshPart(mpcount_a),
                            shape_b_,
                            global_res_);
                    }

                    return;
                }

                if (shape_b_->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE) {
                    auto meshshape_b = static_cast<const btGImpactMeshShape *>(shape_b_);
                    auto mpcount_b = meshshape_b->getMeshPartCount();

                    while (mpcount_b--) {
                        gimpact_vs_gimpact(
                            obj_wrap_a_,
                            obj_wrap_b_,
                            shape_a_,
                            meshshape_b->getMeshPart(mpcount_b),
                            global_res_);
                    }

                    return;
                }

                if (shape_a_->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART &&
                    shape_b_->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART)
                {
                    const btGImpactMeshShapePart * shapepart_a =
                        static_cast<const btGImpactMeshShapePart *>(shape_a_);
                    const btGImpactMeshShapePart * shapepart_b =
                        static_cast<const btGImpactMeshShapePart *>(shape_b_);

                    gimpact_mesh_part_vs_gimpact_mesh_part(
                        obj_wrap_a_->getWorldTransform(),
                        obj_wrap_b_->getWorldTransform(),
                        shapepart_a,
                        shapepart_b,
                        global_res_);
                }
            }

            /*!
               Calculate minimum distance of given objects. Returns true on success, false otherwise.
            */
            bool perform_distance_calculation(
                btCollisionObjectWrapper* obj_a_,
                btCollisionObjectWrapper* obj_b_,
                min_dist_output& result_)
            {
                if (obj_a_->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) {
                    auto gimpactshape_a =
                        static_cast<const btGImpactShapeInterface *>(obj_a_->getCollisionShape());

                    if (obj_b_->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) {
                        auto gimpactshape_b =
                            static_cast<const btGImpactShapeInterface *>(obj_b_->getCollisionShape());

                        gimpact_vs_gimpact(obj_a_, obj_b_, gimpactshape_a, gimpactshape_b, result_);

                        return true;
                    }
                }

                return false;
            }

            /*!
              Calculate minimum distance from collision data present of current world.
              Precondition: the performDiscreteCollisonDetection or equivalent for obj_a_ and obj_b_ was run.
              If any manifolds with contact information for obj_a_ and obj_b_ are found, the one with
              smallest mutual distance of the contact points is returned in result_.
              Returns true if a manifold was found false otherwise.
            */
            bool try_calculate_from_collision_data(
                btCollisionObject* obj_a_,
                btCollisionObject* obj_b_,
                min_dist_output& result_)
            {
                auto contact_found = false;
#ifdef COLD_BULLET_MIN_DIST_USE_COLLISION_DATA
                // construct result from manifolds, if any...
                // doesn't make sense to calculate euclidian distance,
                // when we know it equals zero. 
                // TODO: Optimization: Could cache the values in a hashbag if necessary...

                result_.distance = std::numeric_limits<btScalar>::max();

                auto minkowski_dist = std::numeric_limits<btScalar>::max();

                if (auto dispatcher = _world->getDispatcher()) {

                    // Not sure if should search further once a contact found is set...
                    for (auto idx_m = 0, size_m = dispatcher->getNumManifolds(); idx_m < size_m; ++idx_m) {

                        auto manifold = dispatcher->getManifoldByIndexInternal(idx_m);
#if BT_BULLET_VERSION >= 283
                        auto obj_a = manifold->getBody0();
                        auto obj_b = manifold->getBody1();
#else
                        auto obj_a = static_cast<btCollisionObject*>(manifold->getBody0());
                        auto obj_b = static_cast<btCollisionObject*>(manifold->getBody1());
#endif
                        if (obj_a == obj_a_ && obj_b == obj_b_) {
                            // find minimum of all contact points...
                            for (auto idx_c = 0, size_c = manifold->getNumContacts(); idx_c < size_c; ++idx_c) {

                                auto const& contact_pt = manifold->getContactPoint(idx_c);

                                std::cout << "manifold[" << idx_m << "], " << "contact[" << idx_c << "]:" << std::endl
                                    << contact_pt << std::endl;

                                // logically the only meaningfull information which is contained
                                // here is that the distance should be zero... The positions
                                // of the contact points are useless...

                                if (contact_pt.m_distance1 <= 0.) {
                                    // I am only interested in euclidian distance...
                                    auto dist = std::min(0., contact_pt.m_distance1);

                                    if (dist < result_.distance) {
                                        result_.distance = dist;
                                        result_.obj_a = const_cast<btCollisionObject*>(obj_a);
                                        result_.obj_b = const_cast<btCollisionObject*>(obj_b);
                                        // these values are mostly totally screwed...
                                        result_.point_on_a = contact_pt.getPositionWorldOnA();
                                        result_.point_on_b = contact_pt.getPositionWorldOnB();

                                        contact_found = true;
                                    }
                                }
                            }
                        }
                    }
                }
#endif
                return contact_found;
            }
        public:
            explicit min_dist_calculator(btCollisionWorld* world_)
                : _world(world_)
            {
            }

            /// Calculate minimumum distance between objects. idx_a and idx_b are the indices 
            /// of bodies a and b in bullet world
            min_dist_output perform_distance_calculation(
                btCollisionObject* obj_a_,
                btCollisionObject* obj_b_)
            {
                min_dist_output result;

                result.obj_a = obj_a_;
                result.obj_b = obj_b_;

                if (!try_calculate_from_collision_data(obj_a_, obj_b_, result)) {
                    btCollisionObjectWrapper obj_a = {
                       nullptr,
                       obj_a_->getCollisionShape(),
                       obj_a_,
                       obj_a_->getWorldTransform(),
                       -1, // partId
                       -1  // index
                    };

                    btCollisionObjectWrapper obj_b = {
                       nullptr,
                       obj_b_->getCollisionShape(),
                       obj_b_,
                       obj_b_->getWorldTransform(),
                       -1,
                       -1
                    };

                    // prototype is a "discrete collision detection query" in 
                    // e.g. btCollisionWorld::contactPairTest
                    perform_distance_calculation(&obj_a, &obj_b, result);
                }

                return result;
            }

            min_dist_output_matrix perform_distance_calculation()
            {
                auto const& obj_arr = _world->getCollisionObjectArray();
                auto const obj_arr_size = obj_arr.size();
                min_dist_output_matrix results;

                for (auto ia = 0; ia < obj_arr_size; ++ia) {
                    for (auto ib = ia + 1; ib < obj_arr_size; ++ib) {
                        results.push_back(
                            perform_distance_calculation(obj_arr[ia], obj_arr[ib]));
                    }
                }

                return results;
            }
        };
    }
}