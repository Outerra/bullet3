#include "../BulletCollision/BroadphaseCollision/btDbvt.h"

#include <comm/atomic/pool.h>
#include <comm/singleton.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

btAlignedObjectArray<const btDbvtNode*>& btDbvt::getPooledRayTestStack() 
{
    return *PROCWIDE_SINGLETON(coid::pool<btAlignedObjectArray<const btDbvtNode*>>).create_item();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void btDbvt::releasePooledRayTestStack(btAlignedObjectArray<const btDbvtNode*>& stack)
{
    btAlignedObjectArray<const btDbvtNode*>* stack_ptr = &stack;
    PROCWIDE_SINGLETON(coid::pool<btAlignedObjectArray<const btDbvtNode*>>).release_item(stack_ptr);
}
