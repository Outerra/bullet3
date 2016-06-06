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
	coid::dynarray<bt::triangle>& data,
	coid::dynarray<bt::tree_batch*>& trees);

#else

static bool _ext_collider(
    const void* planet,
    const double3& center,
    float radius,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<bt::tree_batch*>& trees)
{
    return _physics->terrain_collisions(planet, center, radius, data, trees);
}

#endif



void set_debug_drawer(btIDebugDraw * debug_draw) {
    if (_physics) {
        _physics->set_debug_draw(debug_draw);
    }
}

void debug_draw_world() {
    if (_physics) {
        _physics->debug_draw_world();
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
		context
        );

    _physics->_world = wrld;

    _physics->_world->setForceUpdateAllAabbs(false);

    return _physics;
}

iref<physics> physics::get()
{
	if (!_physics) {
		throw coid::exception("Bullet not initialized yet!");
	}

	return _physics;
}

void physics::set_debug_draw(btIDebugDraw * debug_draw) {
	if (_physics->_world) {
		_physics->_world->setDebugDrawer(debug_draw);
	}
}

void physics::debug_draw_world() {
    _physics->_world->debugDrawWorld();
}


////////////////////////////////////////////////////////////////////////////////
btCollisionShape* physics::create_shape( bt::EShape sh, const float hvec[3] )
{
    switch(sh) {
    case bt::SHAPE_CONVEX:  return new btConvexHullShape();
    case bt::SHAPE_SPHERE:  return new btSphereShape(hvec[0]);
    case bt::SHAPE_BOX:     return new btBoxShape(btVector3(hvec[0], hvec[1], hvec[2]));
    case bt::SHAPE_CYLINDER:return new btCylinderShapeZ(btVector3(hvec[0], hvec[1], hvec[2]));
    case bt::SHAPE_CAPSULE: return new btCapsuleShape(hvec[0], hvec[2]);
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
    _world->rayTest(*(const btVector3*)from, *(const btVector3*)to,
        *(btCollisionWorld::RayResultCallback*)cb);
}
