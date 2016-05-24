#include "tree_batch.h"



tree_batch::tree_batch()
{
	for (int i = 0; i < 16; i++) {
		_shapes_cache[i] = reinterpret_cast<btCapsuleShape *>(&(_shapes_mem[sizeof(btCapsuleShape)*i]));
	}
}


tree_batch::~tree_batch()
{
}
