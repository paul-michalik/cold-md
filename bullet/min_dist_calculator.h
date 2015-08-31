#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <limits>
#include <vector>

namespace cold {
    namespace bullet {

        class min_dist_input {
        };

        class min_dist_output {
        public:
            btScalar dist = std::numeric_limits<btScalar>::max();
            btVector3 point_on_a, point_on_b;
            btCollisionObject *obj_a, *obj_b;
        };

        using min_dist_output_matrix = std::vector<min_dist_output>;

        class min_dist_calculator {
            btCollisionWorld* _world;

            void gimpact_mesh_vs_gimpact_mesh(
                const btTransform & trans_a_,
                const btTransform & trans_b,
                const btGImpactMeshShapePart* shape_a,
                const btGImpactMeshShapePart* shape_b,
                min_dist_output& global_res_)
            {
                // almost there!
                // gimpact_vs_gimpact_find_pairs...
            }

            void gimpact_vs_gimpact(
                const btCollisionObjectWrapper* obj_wrap_a_,
                const btCollisionObjectWrapper* obj_wrap_b_,
                const btGImpactShapeInterface* shape_a_,
                const btGImpactShapeInterface* shape_b_,
                min_dist_output& global_res_)
            {
                // process mesh parts until we get to meshes...
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

                    gimpact_mesh_vs_gimpact_mesh(
                        obj_wrap_a_->getWorldTransform(),
                        obj_wrap_b_->getWorldTransform(),
                        shapepart_a,
                        shapepart_b,
                        global_res_);
                }
            }

            min_dist_output calculate(
                btCollisionObjectWrapper* obj_a_,
                btCollisionObjectWrapper* obj_b_)
            {
                min_dist_output res;
                if (obj_a_->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) {
                    auto gimpactshape_a = 
                        static_cast<const btGImpactShapeInterface *>(obj_a_->getCollisionShape());

                    if (obj_b_->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) {
                        auto gimpactshape_b = 
                            static_cast<const btGImpactShapeInterface *>(obj_b_->getCollisionShape());

                        gimpact_vs_gimpact(obj_a_, obj_b_, gimpactshape_a, gimpactshape_b, res);
                    }
                }
            }
        public:
            min_dist_calculator(btCollisionWorld* world_)
                : _world(world_)
            {
            }

            min_dist_output find_collision(
                btCollisionObject* obj_a_,
                btCollisionObject* obj_b_)
            {

                min_dist_output res;
                // construct result from manifolds, if any...
                // doesn't make sense to calculate euclidian distance,
                // we know it equals zero...

                return res;
            }

            min_dist_output calculate(
                btCollisionObject* obj_a_,
                btCollisionObject* obj_b_)
            {
                min_dist_output res = find_collision(obj_a_, obj_b_);

                // only continue if there is no collision:
                if (res.dist > 0.) {
                    btCollisionObjectWrapper obj_a = {
                        nullptr,
                        obj_a_->getCollisionShape(),
                        obj_a_,
                        obj_a_->getWorldTransform(), 
                        -1, -1
                    };

                    btCollisionObjectWrapper obj_b = {
                        nullptr,
                        obj_b_->getCollisionShape(),
                        obj_b_,
                        obj_b_->getWorldTransform(), 
                        -1, -1
                    };

                     
                    // equivalent to "discrete collision detection query" in e.g. btCollisionWorld::contactPairTest
                    return calculate(&obj_a, &obj_b);
                }
                return res;
            }

            min_dist_output_matrix calculate()
            {
                auto const& obj_arr = _world->getCollisionObjectArray();
                auto const obj_arr_size = obj_arr.size();
                min_dist_output_matrix results;

                for (auto ia = 0; ia < obj_arr_size; ++ia) {
                    for (auto ib = ia + 1; ib < obj_arr_size; ++ib) {
                        results.push_back(
                            calculate(obj_arr[ia], obj_arr[ib]));
                    }
                }
            }
        };
    }
}
