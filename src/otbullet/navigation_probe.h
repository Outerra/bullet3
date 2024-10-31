#pragma once
#include <ot/glm/glm_types.h>

class btPairCachingGhostObject;
class btConvexShape;
class btCollisionWorld;
class btTransform;
class btCollisionObject;

namespace bt
{

class ot_navigation_probe
{
public: // methods only
    ot_navigation_probe(
        btPairCachingGhostObject* ghost_object_ptr,
        const float3& shape_offset
    );

    void set_transform(const double3& pos, const quat& rot);

    void sim_step(btCollisionWorld* world_ptr, const double3& target_position, const quat& target_rotation, float dt);

    double3 get_pos() const;
    quat get_rot() const;

protected: // methods only
    ot_navigation_probe(const ot_navigation_probe&) = delete;
    ot_navigation_probe& operator=(const ot_navigation_probe&) = delete;

    btTransform calculate_transform_internal(const double3& pos, const quat& rot) const;

    bool needs_collision_with_internal(const btCollisionObject* other_ptr);
protected: // members only
    btPairCachingGhostObject* _ghost_object = nullptr;
    btConvexShape* _collision_shape = nullptr;
    float3 _shape_offset = float3(0);
};

}; // end of namespace bt