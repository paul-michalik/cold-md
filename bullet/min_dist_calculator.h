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

namespace NSBulletPhysicsExt {

struct min_dist_output {
   btCollisionObject const* obj_a = nullptr;
   btCollisionObject const* obj_b = nullptr;

   btVector3 point_on_a;
   btVector3 point_on_b;

   btScalar distance = std::numeric_limits<btScalar>::max();

   // Used for internal pruning tests. Represents the lower and upper bound
   // of current distance intervals during the computation for respective objects.
   std::pair<btScalar, btScalar> cur_dist_interval = {
      std::numeric_limits<btScalar>::max(),
      0,
   };

   friend std::ostream& operator << (std::ostream& out_, min_dist_output const& md_res_)
   {
      return out_ << "result: " << std::endl <<
         "obj_a:      " << md_res_.obj_a << std::endl <<
         "obj_b:      " << md_res_.obj_b << std::endl <<
         "point_on_a: " << md_res_.point_on_a.getX() << ", " << md_res_.point_on_a.getY() << ", " << md_res_.point_on_a.getZ() << std::endl <<
         "point_on_b: " << md_res_.point_on_b.getX() << ", " << md_res_.point_on_b.getY() << ", " << md_res_.point_on_b.getZ() << std::endl <<
         "distance:   " << md_res_.distance << std::endl <<
         "cur_dist_interval: " << md_res_.cur_dist_interval.first << ", " << md_res_.cur_dist_interval.second;
   }
};

using min_dist_output_matrix = std::vector<min_dist_output>;

class min_dist_calculator {
   btCollisionWorld* _world;

   bool pass_lower_upper_bound_test(
      const btGImpactQuantizedBvh * boxes_a_,
      const btGImpactQuantizedBvh * boxes_b_,
      int box_idx_a,
      int box_idx_b,
      const BT_BOX_BOX_TRANSFORM_CACHE & trans_cache_b_to_a_,
      min_dist_output& global_res_)
   {
      // TODO: Implement optimization, gjk box/box?
      return true;
   }

   void gimpact_boxset_vs_gimpact_boxset_rec(
      const btGImpactQuantizedBvh * boxes_a_,
      const btGImpactQuantizedBvh * boxes_b_,
      int box_idx_a_,
      int box_idx_b_,
      const BT_BOX_BOX_TRANSFORM_CACHE & trans_cache_b_to_a_,
      min_dist_output& global_res_)
   {
      // prototype: _find_quantized_collision_pairs_recursive...

      // internal node pruning:
      if (!pass_lower_upper_bound_test(
         boxes_a_, boxes_b_,
         box_idx_a_, box_idx_b_,
         trans_cache_b_to_a_, global_res_)) {
         return;
      }

      if (boxes_a_->isLeafNode(box_idx_a_)) {
         if (boxes_b_->isLeafNode(box_idx_b_)) {
            // TODO: calculate min dist triangle/triangle gjk and store in global_res_!

            return;
         } else {

            // descend left recursive:
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               box_idx_a_, boxes_b_->getLeftNode(box_idx_b_),
               trans_cache_b_to_a_,
               global_res_);

            // descend right recursive
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               box_idx_a_, boxes_b_->getRightNode(box_idx_b_),
               trans_cache_b_to_a_,
               global_res_);
         }
      } else {
         if (boxes_b_->isLeafNode(box_idx_b_)) {

            // descend left recursive:
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               boxes_a_->getLeftNode(box_idx_a_), box_idx_b_,
               trans_cache_b_to_a_, global_res_);

            // descend right recursive:

            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               boxes_a_->getRightNode(box_idx_a_), box_idx_b_,
               trans_cache_b_to_a_, global_res_);
         } else {
            // descend left0 left1
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               boxes_a_->getLeftNode(box_idx_a_), boxes_b_->getLeftNode(box_idx_b_),
               trans_cache_b_to_a_, global_res_);

            // descend left0 right1
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               boxes_a_->getLeftNode(box_idx_a_), boxes_b_->getRightNode(box_idx_b_),
               trans_cache_b_to_a_, global_res_);

            // descend right0 left1
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               boxes_a_->getRightNode(box_idx_a_), boxes_b_->getLeftNode(box_idx_b_),
               trans_cache_b_to_a_, global_res_);

            // descend right0 right1
            gimpact_boxset_vs_gimpact_boxset_rec(
               boxes_a_, boxes_b_,
               boxes_a_->getRightNode(box_idx_a_), boxes_b_->getRightNode(box_idx_b_),
               trans_cache_b_to_a_, global_res_);

         }// else if box_idx_b_ is not a leaf
      }// else if box_idx_a_ is not a leaf
   }

   void gimpact_mesh_part_vs_gimpact_mesh_part(
      const btTransform & trans_a_,
      const btTransform & trans_b,
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
            trans_cache_b_to_a.calc_from_homogenic(trans_a_, trans_b);

            gimpact_boxset_vs_gimpact_boxset_rec(
               box_set_a, box_set_b,
               0, 0,
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
      auto contact_found = false;
   
      if (auto dispatcher = _world->getDispatcher()) {

         // Not sure if should search further once a contact found is set...
         for (auto idx_m = 0, size_m = dispatcher->getNumManifolds(); idx_m < size_m /*&& !contact_found*/; ++idx_m) {

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

                  // I am only interested in euclidian distance...
                  auto dist = contact_pt.getPositionWorldOnA()
                     .distance(contact_pt.getPositionWorldOnB());

                  if (dist < result_.distance) {
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