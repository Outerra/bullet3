#include "discrete_dynamics_world.h"

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>

#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>

#include <LinearMath/btIDebugDraw.h>

#include "otbullet.hpp"
#include "physics_cfg.h"

#include <comm/ref_i.h>
#include <comm/commexception.h>

static btBroadphaseInterface* _overlappingPairCache = 0;
static btCollisionDispatcher* _dispatcher = 0;
static btConstraintSolver* _constraintSolver = 0;
static btDefaultCollisionConfiguration* _collisionConfiguration = 0;

static physics * _physics = nullptr;

////////////////////////////////////////////////////////////////////////////////
#ifdef _LIB

extern bool _ext_collider(const void* context,
	const double3& center,
	float radius,
    float lod_dimension,
	coid::dynarray<bt::triangle>& data,
	coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame );

extern bool _ext_collider_obb(
    const void * context,
    const double3& center,
    const float3x3& basis,
    float lod_dimension,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame,
    bool& is_above_tm);

extern float _ext_terrain_ray_intersect(
    const void* planet,
    const double3& from,
    const float3& dir,
    const float2& minmaxlen,
    float3* norm,
    double3* pos);

extern static float _ext_elevation_above_terrain(
    const double3& pos,
    float maxlen,
    float3* norm,
    double3* hitpoint);



extern float3 _ext_tree_col(btRigidBody * obj, 
        bt::tree_collision_contex & ctx,
    float time_step,
    coid::slotalloc<bt::tree_batch>& tree_baFBtches);
#else

static bool _ext_collider(
    const void* planet,
    const double3& center,
    float radius,
    float lod_dimension,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame )
{
    return _physics->terrain_collisions(planet, center, radius, lod_dimension, data, trees, tree_batches, frame);
}

static bool _ext_collider_obb(
    const void * planet,
    const double3& center,
    const float3x3& basis,
    float lod_dimension,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame,
    bool& is_above_tm)
{
    return _physics->terrain_collisions_aabb(planet, center, basis, lod_dimension, data, trees, tree_batches, frame,is_above_tm);
}

static float _ext_terrain_ray_intersect(
    const void* planet,
    const double3& from,
    const float3& dir,
    const float2& minmaxlen,
    float3* norm,
    double3* pos) {
    return _physics->terrain_ray_intersect(
        planet,
        from,
        dir,
        minmaxlen,
        norm,
        pos);
}

static float _ext_elevation_above_terrain(
    const double3& pos,
    float maxlen,
    float3* norm,
    double3* hitpoint) 
{
    return _physics->elevation_above_terrain(
        pos,
        maxlen,
        norm,
        hitpoint);
}


static float3 _ext_tree_col(btRigidBody * obj,
    bt::tree_collision_contex & ctx,
    float time_step,
    coid::slotalloc<bt::tree_batch>& tree_batches) {

    return _physics->tree_collisions(obj, ctx, time_step,tree_batches);

}
#endif


void debug_draw_world() {
    if (_physics) {
        _physics->debug_draw_world();
    }
}

void set_debug_drawer_enabled(btIDebugDraw * debug_draw) {
    if (_physics) {
        _physics->set_debug_draw_enabled(debug_draw);
    }
}

////////////////////////////////////////////////////////////////////////////////
iref<physics> physics::create(double r, void* context)
{
    _physics = new physics;

    _collisionConfiguration = new btDefaultCollisionConfiguration();
    _dispatcher = new btCollisionDispatcher(_collisionConfiguration);
    btVector3 worldMin(-r,-r,-r);
    btVector3 worldMax(r,r,r);

    _overlappingPairCache = new bt32BitAxisSweep3(worldMin, worldMax);
    _constraintSolver = new btSequentialImpulseConstraintSolver();

    ot::discrete_dynamics_world * wrld = new ot::discrete_dynamics_world(
        _dispatcher,
        _overlappingPairCache,
        _constraintSolver,
        _collisionConfiguration,
        &_ext_collider,
        &_ext_tree_col,
        &_ext_terrain_ray_intersect,
        &_ext_elevation_above_terrain,
		context
        );

    wrld->_aabb_intersect = &_ext_collider_obb;

    _physics->_world = wrld;

    _physics->_world->setForceUpdateAllAabbs(false);

    _physics->_dbg_drawer = nullptr;
   
    // default mode
    _physics->_dbg_draw_mode = btIDebugDraw::DBG_DrawContactPoints | btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits;

    return _physics;
}

iref<physics> physics::get()
{
	if (!_physics) {
		throw coid::exception("Bullet not initialized yet!");
	}

	return _physics;
}

void physics::debug_draw_world() {
    if (_dbg_drawer) {
        _world->debugDrawWorld();
    }
}

void physics::query_volume_sphere(const double3 & pos, float rad, coid::dynarray<btCollisionObject*>& result)
{
    _world->query_volume_sphere(pos,rad,result);
}

void physics::query_volume_frustum(const double3 & pos,const float4 * f_planes_norms, uint8 nplanes, bool include_partial, coid::dynarray<btCollisionObject*>& result)
{
    _world->query_volume_frustum(pos, f_planes_norms, nplanes, include_partial,result);
}


////////////////////////////////////////////////////////////////////////////////
btCollisionShape* physics::create_shape( bt::EShape sh, const float hvec[3] )
{
    switch(sh) {
    case bt::SHAPE_CONVEX:  return new btConvexHullShape();
    case bt::SHAPE_SPHERE:  return new btSphereShape(hvec[0]);
    case bt::SHAPE_BOX:     return new btBoxShape(btVector3(hvec[0], hvec[1], hvec[2]));
    case bt::SHAPE_CYLINDER:return new btCylinderShapeZ(btVector3(hvec[0], hvec[1], hvec[2]));
    case bt::SHAPE_CAPSULE: {
        if (glm::abs(hvec[1] - hvec[2]) < 0.000001f) {
           //btCapsuleX
            return new btCapsuleShapeX(hvec[1], 2.f*(hvec[0] - hvec[1]));
        }
        else if(glm::abs(hvec[0] - hvec[2]) < 0.000001f){
            //btCapsuleY
            return new btCapsuleShape(hvec[0], 2.f*(hvec[1] - hvec[0]));
        }
        else{
            //btCapsuleZ
            return new btCapsuleShapeZ(hvec[1], 2.f*(hvec[2] - hvec[1]));
        }
    }
    case bt::SHAPE_CONE:    return new btConeShapeZ(hvec[0], hvec[2]);
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_shape( btCollisionShape*& shape )
{
    delete shape;
    shape = 0;
}

////////////////////////////////////////////////////////////////////////////////
void physics::add_convex_point( btCollisionShape* shape, const float pt[3] )
{
    static_cast<btConvexHullShape*>(shape)->addPoint(btVector3(pt[0], pt[1], pt[2]), false);
}

////////////////////////////////////////////////////////////////////////////////
void physics::close_convex_shape( btCollisionShape* shape )
{
    shape->setMargin(0.005);
    static_cast<btConvexHullShape*>(shape)->recalcLocalAabb();
}

////////////////////////////////////////////////////////////////////////////////
btCompoundShape* physics::create_compound_shape()
{
    return new btCompoundShape;
}

////////////////////////////////////////////////////////////////////////////////
void physics::add_child_shape( btCompoundShape* group, btCollisionShape* child, const btTransform& tr )
{
    group->addChildShape(tr, child);
}

////////////////////////////////////////////////////////////////////////////////
void physics::update_child( btCompoundShape* group, int index, const btTransform& tr )
{
    group->updateChildTransform(index, tr, false);
}

////////////////////////////////////////////////////////////////////////////////
void physics::recalc_compound_shape( btCompoundShape* shape )
{
    shape->recalculateLocalAabb();
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_compound_shape( btCompoundShape*& shape )
{
    delete shape;
    shape = 0;
}



////////////////////////////////////////////////////////////////////////////////
btCollisionObject* physics::create_collision_object( btCollisionShape* shape, void* usr1, void* usr2 )
{
    btCollisionObject* obj = new btCollisionObject;
    obj->setCollisionShape(shape);

    obj->setUserPointer(usr1);
    obj->m_userDataExt = usr2;

    return obj;
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_collision_info(btCollisionObject* obj, unsigned int group, unsigned int mask)
{
	btBroadphaseProxy* bp = obj->getBroadphaseHandle();
	if (bp) {
		bp->m_collisionFilterGroup = group;
		bp->m_collisionFilterMask = mask;
	}
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_collision_object( btCollisionObject*& obj )
{
    if(obj) delete obj;
    obj = 0;
}

////////////////////////////////////////////////////////////////////////////////
void physics::add_collision_object( btCollisionObject* obj, unsigned int group, unsigned int mask, bool inactive )
{
    if(inactive)
        obj->setActivationState(DISABLE_SIMULATION);

    _world->addCollisionObject(obj, group, mask);
}

////////////////////////////////////////////////////////////////////////////////
void physics::remove_collision_object( btCollisionObject* obj )
{
    _world->removeCollisionObject(obj);
}

////////////////////////////////////////////////////////////////////////////////
void physics::update_collision_object( btCollisionObject* obj, const btTransform& tr, bool update_aabb )
{
    obj->setWorldTransform(tr);

    if(update_aabb && obj->getBroadphaseHandle())
        _world->updateSingleAabb(obj);
}



////////////////////////////////////////////////////////////////////////////////
void physics::step_simulation( double step )
{
    _world->stepSimulation(step, 0, step);
}

////////////////////////////////////////////////////////////////////////////////
void physics::ray_test( const double from[3], const double to[3], void* cb)
{
    btVector3 afrom = btVector3(from[0], from[1], from[2]);
    btVector3 ato = btVector3(to[0], to[1], to[2]);

    _world->rayTest(afrom, ato, *(btCollisionWorld::RayResultCallback*)cb);
}

////////////////////////////////////////////////////////////////////////////////

bt::ot_world_physics_stats physics::get_stats() {
    return ((ot::discrete_dynamics_world*)(_world))->get_stats();
}

////////////////////////////////////////////////////////////////////////////////

bt::ot_world_physics_stats* physics::get_stats_ptr() {
    return const_cast<bt::ot_world_physics_stats*>(&((ot::discrete_dynamics_world*)(_world))->get_stats());
}

////////////////////////////////////////////////////////////////////////////////

void physics::set_debug_draw_enabled(btIDebugDraw * debug_drawer) {
    _dbg_drawer = debug_drawer;
    ((ot::discrete_dynamics_world*)_world)->setDebugDrawer(debug_drawer);

    if (debug_drawer) {
        debug_drawer->setDebugMode(_dbg_draw_mode);
    }
}

////////////////////////////////////////////////////////////////////////////////

void physics::set_debug_drawer_mode(int debug_mode) {
    _dbg_draw_mode = debug_mode;

    if (_dbg_drawer) {
        _dbg_drawer->setDebugMode(debug_mode);
    }
}
