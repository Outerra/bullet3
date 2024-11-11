#include "navigation_probe.h"

#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>

#include <ot/glm/glm_bt.h>

#include <comm/commassert.h>
#include <comm/log.h>


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

class btKinematicClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
public:
    btKinematicClosestNotMeConvexResultCallback(btCollisionObject* me, const btVector3& up, btScalar minSlopeDot)
        : btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
        , m_me(me)
        , m_up(up)
        , m_minSlopeDot(minSlopeDot)
    {
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
    {
        if (convexResult.m_hitCollisionObject == m_me)
            return btScalar(1.0);

        if (!convexResult.m_hitCollisionObject->hasContactResponse())
            return btScalar(1.0);

        btVector3 hitNormalWorld;
        if (normalInWorldSpace)
        {
            hitNormalWorld = convexResult.m_hitNormalLocal;
        }
        else
        {
            ///need to transform normal into worldspace
            hitNormalWorld = convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
        }

        btScalar dotUp = m_up.dot(hitNormalWorld);
        if (dotUp < m_minSlopeDot) {
            return btScalar(1.0);
        }

        return ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
    }
protected:
    btCollisionObject* m_me;
    const btVector3 m_up;
    btScalar m_minSlopeDot;
};


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bt::ot_navigation_probe::ot_navigation_probe(btPairCachingGhostObject* ghost_object_ptr,
    const float3& shape_offset)
    : _ghost_object(ghost_object_ptr)
    , _shape_offset(shape_offset)
{
    btCollisionShape* shape_ptr = ghost_object_ptr->getCollisionShape();
    DASSERTX(shape_ptr->isConvex(), "must be covnex shape so the sweep test can be used");
    _collision_shape = static_cast<btConvexShape*>(shape_ptr);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void bt::ot_navigation_probe::set_transform(const double3& pos, const quat& rot)
{
    _ghost_object->setWorldTransform(calculate_transform_internal(pos, rot));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void bt::ot_navigation_probe::sim_step(btCollisionWorld* world_ptr, const double3& target_position, const quat& target_rotation, float dt)
{
    btTransform current_transform = _ghost_object->getWorldTransform();
    btTransform target_transform(calculate_transform_internal(target_position, target_rotation));


    // printf("m_normalizedDirection=%f,%f,%f\n",
    // 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
    // phase 2: forward and strafe
    btTransform start, end;

    start.setIdentity();
    end.setIdentity();

    btVector3 current_position_bt = current_transform.getOrigin();
    btVector3 target_position_bt = target_transform.getOrigin();


    start.setOrigin(current_transform.getOrigin());
    end.setOrigin(target_transform.getOrigin());
    btVector3 sweepDirNegative(current_position_bt - target_position_bt);

    start.setRotation(current_transform.getRotation());
    end.setRotation(target_transform.getRotation());

    btKinematicClosestNotMeConvexResultCallback callback(_ghost_object, sweepDirNegative, btScalar(0.0));
    callback.m_collisionFilterGroup = _ghost_object->getBroadphaseHandle()->m_collisionFilterGroup;
    callback.m_collisionFilterMask = _ghost_object->getBroadphaseHandle()->m_collisionFilterMask;

    if (sweepDirNegative.length2() > 0.0001)
    {
        world_ptr->convexSweepTest(_collision_shape, start, end, callback, 0.0);
    }

    if (callback.hasHit() && needs_collision_with_internal(callback.m_hitCollisionObject))
    {
        current_position_bt -= sweepDirNegative * callback.m_closestHitFraction;
    }
    else 
    {
        current_position_bt = target_position_bt;
    }

    _ghost_object->setWorldTransform(btTransform(target_transform.getRotation(), current_position_bt));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double3 bt::ot_navigation_probe::get_pos() const
{    
    double3 result = bt::todouble3(_ghost_object->getWorldTransform().getOrigin());
    double3 offset(bt::toquat(_ghost_object->getWorldTransform().getRotation()) * _shape_offset);
    return  result - offset;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

quat bt::ot_navigation_probe::get_rot() const
{
    return bt::toquat(_ghost_object->getWorldTransform().getRotation());
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

btTransform bt::ot_navigation_probe::calculate_transform_internal(const double3& pos, const quat& rot) const
{
    return btTransform(bt::toQuat(rot), bt::toVector3(pos + double3(rot * _shape_offset)));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool bt::ot_navigation_probe::needs_collision_with_internal(const btCollisionObject* other_ptr)
{
    const bool v0 = (_ghost_object->getBroadphaseHandle()->m_collisionFilterGroup & other_ptr->getBroadphaseHandle()->m_collisionFilterMask) != 0;
    const bool v1 = (other_ptr->getBroadphaseHandle()->m_collisionFilterGroup & _ghost_object->getBroadphaseHandle()->m_collisionFilterMask) != 0;
    return v0 && v1;
}
