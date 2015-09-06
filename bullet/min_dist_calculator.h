#ifndef MINDISTCALCULATOR_H
#define MINDISTCALCULATOR_H

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include <limits>
#include <vector>
#include <algorithm>
#include <hash_map>
#include <ostream>

#if BT_BULLET_VERSION >= 283
#else
#ifndef BT_COLLISION_OBJECT_WRAPPER_H
#define BT_COLLISION_OBJECT_WRAPPER_H

///btCollisionObjectWrapperis an internal data structure. 
///Most users can ignore this and use btCollisionObject and btCollisionShape instead
#include "LinearMath/btScalar.h" // for SIMD_FORCE_INLINE definition

namespace {

#define BT_DECLARE_STACK_ONLY_OBJECT \
 private: \
  void* operator new(size_t size); \
  void operator delete(void*);

    struct btCollisionObjectWrapper;
    struct btCollisionObjectWrapper
    {
        BT_DECLARE_STACK_ONLY_OBJECT

    private:
        btCollisionObjectWrapper(const btCollisionObjectWrapper&); // not implemented. Not allowed.
        btCollisionObjectWrapper* operator=(const btCollisionObjectWrapper&);

    public:
        const btCollisionObjectWrapper* m_parent;
        const btCollisionShape* m_shape;
        const btCollisionObject* m_collisionObject;
        const btTransform& m_worldTransform;
        int  m_partId;
        int  m_index;

        btCollisionObjectWrapper(const btCollisionObjectWrapper* parent, const btCollisionShape* shape, const btCollisionObject* collisionObject, const btTransform& worldTransform, int partId, int index)
            : m_parent(parent), m_shape(shape), m_collisionObject(collisionObject), m_worldTransform(worldTransform),
            m_partId(partId), m_index(index)
        {
        }

        SIMD_FORCE_INLINE const btTransform& getWorldTransform() const
        {
            return m_worldTransform;
        }
        SIMD_FORCE_INLINE const btCollisionObject* getCollisionObject() const
        {
            return m_collisionObject;
        }
        SIMD_FORCE_INLINE const btCollisionShape* getCollisionShape() const
        {
            return m_shape;
        }
    };
}
#endif //BT_COLLISION_OBJECT_WRAPPER_H

#endif


inline std::ostream& operator<<(std::ostream& out_, btVector3 const& v_)
{
    return out_ << "{"
        << v_.getX() << ", "
        << v_.getY() << ", "
        << v_.getZ() << "}";
}

inline std::ostream& operator<<(std::ostream& out_, btManifoldPoint const& contact_pt_)
{
    return out_ << typeid(contact_pt_).name() << ": " << &contact_pt_ << std::endl <<
        "m_localPointA:      " << contact_pt_.m_localPointA << std::endl <<
        "m_localPointB:      " << contact_pt_.m_localPointB << std::endl <<
        "m_positionWorldOnA: " << contact_pt_.m_positionWorldOnA << std::endl <<
        "m_positionWorldOnB: " << contact_pt_.m_positionWorldOnB << std::endl <<
        "m_normalWorldOnB:   " << contact_pt_.m_normalWorldOnB << std::endl <<
        "m_distance1:        " << contact_pt_.m_distance1 << std::endl <<
        "m_partId0:          " << contact_pt_.m_partId0 << std::endl <<
        "m_partId1:          " << contact_pt_.m_partId1 << std::endl <<
        "m_index0:           " << contact_pt_.m_index0 << std::endl <<
        "m_index1:           " << contact_pt_.m_index1 << std::endl <<
        "|A,B|:              " <<
        contact_pt_.getPositionWorldOnA()
        .distance(contact_pt_.getPositionWorldOnB());
}

namespace NSBulletPhysicsExt {

    struct min_dist_output {
        btCollisionObject const* obj_a = nullptr;
        btCollisionObject const* obj_b = nullptr;

        btVector3 point_on_a;
        btVector3 point_on_b;

        // Minimum distance. After initialization will be equal to 
        // cur_dist_interval.first.
        btScalar distance = std::numeric_limits<btScalar>::max();

        // Used for internal pruning tests. Represents the lower and upper bound
        // of current distance intervals during the computation for respective objects.
        // Initialized to respective max, min values of current floating point models
        // so that it can be used as a initial value for pruning operations.
        std::pair<btScalar, btScalar> cur_dist_interval = {
           -std::numeric_limits<btScalar>::max(),
           +std::numeric_limits<btScalar>::max()
        };

        inline friend std::ostream& operator << (std::ostream& out_, min_dist_output const& md_res_)
        {
            return out_ << typeid(md_res_).name() << ": " << &md_res_ << std::endl <<
                "obj_a:      " << md_res_.obj_a << std::endl <<
                "obj_b:      " << md_res_.obj_b << std::endl <<
                "point_on_a: " << md_res_.point_on_a << std::endl <<
                "point_on_b: " << md_res_.point_on_b << std::endl <<
                "distance:   " << md_res_.distance << std::endl <<
                "cur_dist_interval: " << md_res_.cur_dist_interval.first << ", " << md_res_.cur_dist_interval.second;
        }
    };

    using min_dist_output_matrix = std::vector<min_dist_output>;

    using triangle_set_t = std::hash_map<int, btTriangleShapeEx>; 

    // Bullet requires to lock the triangle containers before retrieving triangle data.
    // Therefore, it is very expensive to do when triangles pairs are actually needed and we
    // must collect all triangles in a repository before the actual operation starts...
    class triangle_repository {
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

    class min_dist_calculator {
        btCollisionWorld* _world;
        btGjkEpaPenetrationDepthSolver _epa_solver;

        // prototype: void btConvexConvexAlgorithm::processCollision
        void calc_minimum_distance_raw(
            btConvexShape const* shape_a_,
            btConvexShape const* shape_b_,
            btTransform const& transf_a_,
            btTransform const& transf_b_,
            btPointCollector& output)
        {
            btVoronoiSimplexSolver gjk_simplex_solver;
            btGjkPairDetector::ClosestPointInput input;

            btGjkPairDetector	gjk_pair_detector(shape_a_, shape_b_, &gjk_simplex_solver, &_epa_solver);

            input.m_transformA = transf_a_;
            input.m_transformB = transf_b_;

            gjk_pair_detector.getClosestPoints(input, output, nullptr);
        }

        // ...for boxes in world coordinate system. Precondition:
        // Boxes do not collide!
        min_dist_output calc_lower_upper_bound(
            btAABB const& box_a_,
            btAABB const& box_b_)
        {
            // 1. Bounding boxes to convex shapes. Since AABBs have 
            // constant orintation, we only need to set proper position:
            btVector3 center_a, extend_a;
            box_a_.get_center_extend(center_a, extend_a);
            btBoxShape box_shape_a(
                btVector3(extend_a.getX() / 2., extend_a.getY() / 2., extend_a.getZ() / 2.));

            btVector3 center_b, extend_b;
            box_b_.get_center_extend(center_b, extend_b);
            btBoxShape box_shape_b(
                btVector3(extend_b.getX() / 2., extend_b.getY() / 2., extend_b.getZ() / 2.));

            btPointCollector pc_output;
            calc_minimum_distance_raw(
                &box_shape_a, &box_shape_b,
                btTransform(btQuaternion(0., 0., 0.), center_a),
                btTransform(btQuaternion(0., 0., 0.), center_b),
                pc_output);

            // 2. lower/upper bound:
            min_dist_output cur_result;

            cur_result.point_on_a =
                pc_output.m_pointInWorld;
            cur_result.point_on_b =
                pc_output.m_pointInWorld + pc_output.m_normalOnBInWorld * pc_output.m_distance;
            cur_result.distance = pc_output.m_distance;
             
            // lower bound:
            cur_result.cur_dist_interval.first = cur_result.distance;

            // upper bound: maximum distance to vertices of other box.
            cur_result.cur_dist_interval.second = cur_result.distance;
            btVector3 v;

            // TODO: Optimization, it'd probably good enough to take the 
            // maxnorm of distances to the closest plance of the other box...

            // a->b
            for (auto idx_v = 0, size_v = box_shape_b.getNumVertices(); idx_v < size_v; ++idx_v) {
                box_shape_b.getVertex(idx_v, v);

                cur_result.cur_dist_interval.second = std::max(
                        cur_result.cur_dist_interval.second,
                        cur_result.point_on_a.distance(v));
            }
            // b->a
            for (auto idx_v = 0, size_v = box_shape_a.getNumVertices(); idx_v < size_v; ++idx_v) {
                box_shape_a.getVertex(idx_v, v);

                cur_result.cur_dist_interval.second = std::max(
                        cur_result.cur_dist_interval.second,
                        cur_result.point_on_b.distance(v));
            }

            return cur_result;
        }

        // ...for triangles in world coordinate system. 
        // TODO: This smells for intersecting or otherwise colliding triangles. We'll need to take care...
        min_dist_output calc_lower_upper_bound(
            btTriangleShape const& tri_a_,
            btTriangleShape const& tri_b_)
        {
            btTransform null_transf{
                btQuaternion(0, 0, 0), btVector3(0, 0, 0)
            };

            btPointCollector pc_output;
            calc_minimum_distance_raw(
                &tri_a_, &tri_b_, 
                null_transf,
                null_transf, pc_output);

            min_dist_output cur_result;

            cur_result.point_on_a =
                pc_output.m_pointInWorld;
            cur_result.point_on_b =
                pc_output.m_pointInWorld + pc_output.m_normalOnBInWorld * pc_output.m_distance;
            cur_result.distance = pc_output.m_distance;

            // lower/upper bound:
            cur_result.cur_dist_interval.first = 
                cur_result.cur_dist_interval.first = cur_result.distance;

            return cur_result;
        }

        bool does_pass_lower_upper_bound_test(
            const btGImpactQuantizedBvh * boxes_a_,
            const btGImpactQuantizedBvh * boxes_b_,
            int box_idx_a,
            int box_idx_b,
            const BT_BOX_BOX_TRANSFORM_CACHE & trans_cache_b_to_a_,
            min_dist_output& global_res_)
        {
            btAABB box_a;
            boxes_a_->getNodeBound(box_idx_a, box_a);
            btAABB box_b;
            boxes_b_->getNodeBound(box_idx_b, box_b);

            // transform b into a's coordinate system:
            box_b.appy_transform_trans_cache(trans_cache_b_to_a_);

            // If boxes collide, no decision about the lower and upper bounds can be made,
            // neither can this pair of boxes contribute to global lower/upper bound:
            if (!box_a.has_collision(box_b)) {
                // calculate lower/upper bound distances:
                min_dist_output cur_res = 
                    calc_lower_upper_bound(box_a, box_b);

                // the actual pruning test. The test passes (the box can be pruned) if
                // the lower bound of this box pair is greater than the current upper bound...
                if (global_res_.cur_dist_interval.second < cur_res.cur_dist_interval.first) {
                    return true;
                } else {
                    // ...otherwise, if upper bound of this box pair is smaller than current lower bound,
                    // replace current lower and upper bounds with the the values 
                    // of current current box:
                    if (cur_res.cur_dist_interval.second < global_res_.cur_dist_interval.first) {
                        global_res_.cur_dist_interval = cur_res.cur_dist_interval;
                    } else {
                        // ...otherwise, if the lower bound from this box pair is smaller than 
                        // current lower bound update current lower bound. This lets the current 
                        // interval grow "to the left" but never to the right...
                        global_res_.cur_dist_interval.first = std::min(
                            global_res_.cur_dist_interval.first,
                            cur_res.cur_dist_interval.first);
                    }
                }
                // ...otherwise, if the lower-upper bounds intervals intersect,
                // nothing meanigfull can be done, just leave the values as they are...
            }

            return false;
        }

        void gimpact_boxset_vs_gimpact_boxset_rec(
            const btGImpactQuantizedBvh * boxes_a_,
            const btGImpactQuantizedBvh * boxes_b_,
            int box_idx_a_,
            int box_idx_b_,
            triangle_repository const& triangles_,
            const BT_BOX_BOX_TRANSFORM_CACHE & trans_cache_b_to_a_,
            min_dist_output& global_res_)
        {
            // prototype: _find_quantized_collision_pairs_recursive...

            // internal node pruning. If this box pair passes the lower/upper bound test
            // we can safely skip it along all their children from the set of candidates 
            // which contain triangles with minimal distance...
            if (does_pass_lower_upper_bound_test(
                boxes_a_, boxes_b_,
                box_idx_a_, box_idx_b_,
                trans_cache_b_to_a_, global_res_)) {
                return;
            }

            if (boxes_a_->isLeafNode(box_idx_a_)) {
                if (boxes_b_->isLeafNode(box_idx_b_)) {
                    // calculate minima and bounds! Triangles in triangle_repo
                    // are already in world coordinates!
                    auto cur_result = calc_lower_upper_bound(
                        triangles_.get_triangle(boxes_a_, box_idx_a_),
                        triangles_.get_triangle(boxes_b_, box_idx_b_));

                    // update distance and bounds!
                    if (global_res_.distance < cur_result.distance) {
                        global_res_.distance = cur_result.distance;
                    }
                    
                    global_res_.cur_dist_interval.first = std::min(
                        global_res_.cur_dist_interval.first,
                        cur_result.distance);

                    global_res_.cur_dist_interval.second = std::min(
                        global_res_.cur_dist_interval.second,
                        cur_result.distance);
                } else {

                    // descend left recursive:
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        box_idx_a_, boxes_b_->getLeftNode(box_idx_b_),
                        triangles_,
                        trans_cache_b_to_a_,
                        global_res_);

                    // descend right recursive
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        box_idx_a_, boxes_b_->getRightNode(box_idx_b_),
                        triangles_,
                        trans_cache_b_to_a_,
                        global_res_);
                }
            } else {
                if (boxes_b_->isLeafNode(box_idx_b_)) {

                    // descend left recursive:
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        boxes_a_->getLeftNode(box_idx_a_), box_idx_b_,
                        triangles_,
                        trans_cache_b_to_a_, global_res_);

                    // descend right recursive:

                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        boxes_a_->getRightNode(box_idx_a_), box_idx_b_,
                        triangles_,
                        trans_cache_b_to_a_, global_res_);
                } else {
                    // descend left0 left1
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        boxes_a_->getLeftNode(box_idx_a_), boxes_b_->getLeftNode(box_idx_b_), 
                        triangles_,
                        trans_cache_b_to_a_, global_res_);

                    // descend left0 right1
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        boxes_a_->getLeftNode(box_idx_a_), boxes_b_->getRightNode(box_idx_b_),
                        triangles_,
                        trans_cache_b_to_a_, global_res_);

                    // descend right0 left1
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        boxes_a_->getRightNode(box_idx_a_), boxes_b_->getLeftNode(box_idx_b_),
                        triangles_,
                        trans_cache_b_to_a_, global_res_);

                    // descend right0 right1
                    gimpact_boxset_vs_gimpact_boxset_rec(
                        boxes_a_, boxes_b_,
                        boxes_a_->getRightNode(box_idx_a_), boxes_b_->getRightNode(box_idx_b_),
                        triangles_,
                        trans_cache_b_to_a_, global_res_);

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

                // getBoxSet is not defined const in our version!
#if BT_BULLET_VERSION >= 283
                auto box_set_a = shape_part_a_->getBoxSet();
                auto box_set_b = shape_part_b_->getBoxSet();
#else
                auto box_set_a = const_cast<btGImpactMeshShapePart*>(shape_part_a_)->getBoxSet();
                auto box_set_b = const_cast<btGImpactMeshShapePart*>(shape_part_b_)->getBoxSet();
#endif

                if (box_set_a->getNodeCount() > 0 && box_set_b->getNodeCount() > 0) {
                    BT_BOX_BOX_TRANSFORM_CACHE trans_cache_b_to_a;
                    trans_cache_b_to_a.calc_from_homogenic(trans_a_, trans_b_);

                    // retrieve all triangles. Since the container must be locked
                    // it would be really expensive to query the data dynamically...
                    triangle_repository triangles {
                        trans_a_,
                        trans_b_,
                        shape_part_a_,
                        shape_part_b_
                    };

                    gimpact_boxset_vs_gimpact_boxset_rec(
                        box_set_a, box_set_b,
                        0, 0,
                        triangles,
                        trans_cache_b_to_a,
                        global_res_);
                }

            } else { // what...?
                assert(false);
            }
        }

        /*!
           This method is invoked recursively until we arrive at two mesh parts.
        */
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
        bool calculate(
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
            // construct result from manifolds, if any...
            // doesn't make sense to calculate euclidian distance,
            // when we know it equals zero. 
            // TODO: Optimization: Could cache the values in a hashbag if necessary...

            result_.distance = std::numeric_limits<btScalar>::max();

            auto minkowski_dist = std::numeric_limits<btScalar>::max();
            auto contact_found = false;

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


                            // I am only interested in euclidian distance...
                            auto dist = contact_pt.getPositionWorldOnA()
                                .distance(contact_pt.getPositionWorldOnB());

                            if (dist < result_.distance) {
                                minkowski_dist = std::fabs(contact_pt.m_distance1);
                                result_.distance = dist;
                                result_.obj_a = const_cast<btCollisionObject*>(obj_a);
                                result_.obj_b = const_cast<btCollisionObject*>(obj_b);
                                result_.point_on_a = contact_pt.getPositionWorldOnA();
                                result_.point_on_b = contact_pt.getPositionWorldOnB();

                                contact_found = true;
                            }
                        }
                    }
                }
            }

            return contact_found;
        }
    public:
        explicit min_dist_calculator(btCollisionWorld* world_)
            : _world(world_)
        {
        }

        /// Calculate minimumum distance between objects. idx_a and idx_b are the indices 
        /// of bodies a and b in bullet world
        min_dist_output calculate(
            btCollisionObject* obj_a_,
            btCollisionObject* obj_b_)
        {
            min_dist_output result;
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
                calculate(&obj_a, &obj_b, result);
            }

            return result;
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

            return results;
        }
    };
}
#endif