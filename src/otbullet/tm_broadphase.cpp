#include "tm_broadphase.h"
#include <LinearMath/btVector3.h>

namespace bt {
    terrain_mesh_broadphase::terrain_mesh_broadphase(const double3 & min, const double3 & max)
        :_broadphase(btVector3(min.x,min.y,min.z),btVector3(max.x,max.y,max.z), 5000)
    {
    }
}