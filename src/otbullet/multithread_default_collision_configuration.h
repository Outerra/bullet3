#pragma once
#ifndef __multithread_default_collision_configuration_h__
#define __multithread_default_collision_configuration_h__

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>

class multithread_default_collision_configuration: public btDefaultCollisionConfiguration
{
public:
    multithread_default_collision_configuration(const btDefaultCollisionConstructionInfo& constructionInfo = btDefaultCollisionConstructionInfo())
        : btDefaultCollisionConfiguration(constructionInfo)
    {
    }

    virtual	btVoronoiSimplexSolver*	getSimplexSolver() override;
    virtual btConvexPenetrationDepthSolver* getPdSolver() override;

};

#endif // __multithread_default_collision_configuration_h__