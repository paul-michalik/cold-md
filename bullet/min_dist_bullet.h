#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include <ostream>

#if BT_BULLET_VERSION < 283
#ifndef BT_COLLISION_OBJECT_WRAPPER_H
#define BT_COLLISION_OBJECT_WRAPPER_H

///btCollisionObjectWrapperis an internal data structure. 
///Most users can ignore this and use btCollisionObject and btCollisionShape instead
#include "LinearMath/btScalar.h" // for SIMD_FORCE_INLINE definition

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
#endif //BT_COLLISION_OBJECT_WRAPPER_H
namespace {
    btGImpactBoxSet const* btGetBoxSet(btGImpactMeshShapePart const* ms_part_)
    {
        return const_cast<btGImpactMeshShapePart*>(ms_part_)->getBoxSet();
    }

    btGImpactBoxSet const* btGetBoxSet(btGImpactMeshShapePart* ms_part_)
    {
        return ms_part_->getBoxSet();
    }
}
#else
namespace {
    btGImpactBoxSet const* btGetBoxSet(btGImpactMeshShapePart const* ms_part_)
    {
        return ms_part_->getBoxSet();
    }

    btGImpactBoxSet const* btGetBoxSet(btGImpactMeshShapePart* ms_part_)
    {
        return ms_part_->getBoxSet();
    }
}
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
