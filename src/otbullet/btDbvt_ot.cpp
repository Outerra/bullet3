#include "../BulletCollision/BroadphaseCollision/btDbvt.h"

#include <comm/atomic/pool.h>
#include <comm/singleton.h>

LOCAL_FILE_PROCWIDE_SINGLETON_DEF(coid::pool<btAlignedObjectArray<const btDbvtNode*>>) _ray_test_stack_pool;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

btAlignedObjectArray<const btDbvtNode*>& btDbvt::getPooledRayTestStack() 
{
    return *_ray_test_stack_pool->create_item();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void btDbvt::releasePooledRayTestStack(btAlignedObjectArray<const btDbvtNode*>& stack)
{
    btAlignedObjectArray<const btDbvtNode*>* stack_ptr = &stack;
    _ray_test_stack_pool->release_item(stack_ptr);
}
