#pragma once
#include <comm/dynarray.h>
#include <comm/commtypes.h>
#include "../btBulletCollisionCommon.h"

#include <ot/glm/glm_types.h>

//#include "terrain/terrain_mesh.h"
class terrain_mesh;

struct tree_batch
{
	terrain_mesh * _parent_mesh;
	uint32 _last_frame_used;
	uint8 _tree_count;
	double3 _pos[16];
	btCapsuleShape * _shapes_cache[16];
	btCollisionObject _objects_cache[16];

	tree_batch();
	~tree_batch();

private:
	uint8 _shapes_mem[sizeof(btCapsuleShape) * 16];
};

