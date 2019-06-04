#include "multithread_default_collision_configuration.h"
#include <comm/singleton.h>

#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>

btVoronoiSimplexSolver*	multithread_default_collision_configuration::getSimplexSolver()
{
    THREAD_LOCAL_SINGLETON_DEF(btVoronoiSimplexSolver) solver;
    return solver.get();
}

btConvexPenetrationDepthSolver* multithread_default_collision_configuration::getPdSolver() {
    if (info.m_useEpaPenetrationAlgorithm)
    {
        THREAD_LOCAL_SINGLETON_DEF(btGjkEpaPenetrationDepthSolver) solver;
        return solver.get();
    }
    else
    {
        THREAD_LOCAL_SINGLETON_DEF(btMinkowskiPenetrationDepthSolver)solver;
        return solver.get();
    }
}