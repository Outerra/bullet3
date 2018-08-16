#pragma once
#ifndef __OT_TERRAIN_MESH_BROADPHASE_H__
#define __OT_TERRAIN_MESH_BROADPHASE_H__

#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <ot/glm/glm_types.h>

#include <comm/dynarray.h>

class simple_collider;

namespace bt{

class terrain_mesh_broadphase {
    public:
        bt32BitAxisSweep3 _broadphase;
        terrain_mesh_broadphase(const double3& min, const double3& max);

    private:
        coid::dynarray<simple_collider> _colliders;
};

} // end of namespace ot

#endif //__OT_TERRAIN_MESH_BROADPHASE_H__