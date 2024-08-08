#include "discrete_dynamics_world.h"

#include "multithread_default_collision_configuration.h"

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>

#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>

#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

#include <LinearMath/btIDebugDraw.h>

#include "otbullet.hpp"
#include "physics_cfg.h"

#include <comm/ref_i.h>
#include <comm/commexception.h>
#include <comm/taskmaster.h>
#include <comm/singleton.h>

static btBroadphaseInterface* _overlappingPairCache = 0;
static btCollisionDispatcher* _dispatcher = 0;
static btConstraintSolver* _constraintSolver = 0;
static btDefaultCollisionConfiguration* _collisionConfiguration = 0;

static physics* _physics = nullptr;

extern uint gOuterraSimulationFrame;

class custom_ghost_pair_callback : public btGhostPairCallback
{
public:
    virtual btBroadphasePair* addOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
    {
        RASSERT(_physics != nullptr);
        btCollisionObject* collision_object0_ptr = static_cast<btCollisionObject*>(proxy0->m_clientObject);
        btCollisionObject* collision_object1_ptr = static_cast<btCollisionObject*>(proxy1->m_clientObject);
        btGhostObject* ghost_object0_ptr = btGhostObject::upcast(collision_object0_ptr);
        btGhostObject* ghost_object1_ptr = btGhostObject::upcast(collision_object1_ptr);

        // ghost object without flags means its not terrain occluder but the trigger
        if (ghost_object0_ptr && (ghost_object0_ptr->m_otFlags & bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT) == bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT)
        {
            _physics->_world->add_sensor_internal(ghost_object0_ptr, collision_object1_ptr);
        }

        if (ghost_object1_ptr && (ghost_object0_ptr->m_otFlags & bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT) == bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT)
        {
            _physics->_world->add_sensor_internal(ghost_object1_ptr, collision_object0_ptr);
        }

        return btGhostPairCallback::addOverlappingPair(proxy0, proxy1);
    }

    virtual void* removeOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, btDispatcher* dispatcher)
    {
        RASSERT(_physics != nullptr);
        btCollisionObject* collision_object0_ptr = static_cast<btCollisionObject*>(proxy0->m_clientObject);
        btCollisionObject* collision_object1_ptr = static_cast<btCollisionObject*>(proxy1->m_clientObject);
        btGhostObject* ghost_object0_ptr = btGhostObject::upcast(collision_object0_ptr);
        btGhostObject* ghost_object1_ptr = btGhostObject::upcast(collision_object1_ptr);

        // ghost object without flags means its not terrain occluder but the trigger
        if (ghost_object0_ptr && (ghost_object0_ptr->m_otFlags & bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT) == bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT)
        {
            _physics->_world->remove_sensor_internal(ghost_object0_ptr, collision_object1_ptr);
        }

        if (ghost_object1_ptr && (ghost_object0_ptr->m_otFlags & bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT) == bt::EOtFlags::OTF_SENSOR_GHOST_OBJECT)
        {
            _physics->_world->remove_sensor_internal(ghost_object1_ptr, collision_object0_ptr);
        }

        return btGhostPairCallback::removeOverlappingPair(proxy0, proxy1, dispatcher);
    }

};

////////////////////////////////////////////////////////////////////////////////
#ifdef _LIB

extern bool _ext_collider(const void* context,
    const double3& center,
    float radius,
    float lod_dimension,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame);

extern int _ext_collider_obb(
    const void* context,
    const double3& center,
    const float3x3& basis,
    float lod_dimension,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame,
    bool& is_above_tm,
    double3& under_contact,
    float3& under_normal);

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



extern float3 _ext_tree_col(btRigidBody* obj,
    bt::tree_collision_contex& ctx,
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
    uint frame)
{
    return _physics->terrain_collisions(planet, center, radius, lod_dimension, data, trees, tree_batches, frame);
}

static int _ext_collider_obb(
    const void* planet,
    const double3& center,
    const float3x3& basis,
    float lod_dimension,
    coid::dynarray<bt::triangle>& data,
    coid::dynarray<uint>& trees,
    coid::slotalloc<bt::tree_batch>& tree_batches,
    uint frame,
    bool& is_above_tm,
    double3& under_contact,
    float3& under_normal,
    coid::dynarray<bt::external_broadphase*>& broadphases)
{
    return _physics->terrain_collisions_aabb(planet, center, basis, lod_dimension, data, trees,
        tree_batches, frame, is_above_tm, under_contact, under_normal, broadphases);
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

static void _ext_terrain_ray_intersect_broadphase(
    const void* planet,
    const double3& from,
    const float3& dir,
    const float2& minmaxlen,
    coid::dynarray32<bt::external_broadphase*>& bps) {

    _physics->terrain_ray_intersect_broadphase(
        planet,
        from,
        dir,
        minmaxlen,
        bps);
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


static float3 _ext_tree_col(btRigidBody* obj,
    bt::tree_collision_contex& ctx,
    float time_step,
    coid::slotalloc<bt::tree_batch>& tree_batches) {

    return _physics->tree_collisions(obj, ctx, time_step, tree_batches);
}

static void _ext_add_static_collider(const void* context, btCollisionObject* obj, const double3& cen, const float3x3& basis) {
    _physics->add_static_collider(context, obj, cen, basis);
}
#endif


void debug_draw_world(btScalar extrapolation_step) {
    if (_physics) {
        _physics->debug_draw_world(extrapolation_step);
    }
}

void set_debug_drawer_enabled(btIDebugDraw* debug_draw) {
    if (_physics) {
        _physics->set_debug_draw_enabled(debug_draw);
    }
}

////////////////////////////////////////////////////////////////////////////////
iref<physics> physics::create(double r, void* context, coid::taskmaster* tm)
{
    _physics = new physics;
    btDefaultCollisionConstructionInfo dccinfo;
    dccinfo.m_owns_simplex_and_pd_solver = false;

    _collisionConfiguration = new multithread_default_collision_configuration(dccinfo);
    _dispatcher = new btCollisionDispatcher(_collisionConfiguration);
    btVector3 worldMin(-r, -r, -r);
    btVector3 worldMax(r, r, r);

    _overlappingPairCache = new bt32BitAxisSweep3(worldMin, worldMax, 10000);
    _overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new custom_ghost_pair_callback());
    _constraintSolver = new btSequentialImpulseConstraintSolver();

    ot::discrete_dynamics_world* wrld = new ot::discrete_dynamics_world(
        _dispatcher,
        _overlappingPairCache,
        _constraintSolver,
        _collisionConfiguration,
        &_ext_collider,
        &_ext_tree_col,
        &_ext_terrain_ray_intersect,
        &_ext_elevation_above_terrain,
        context,
        tm);

    wrld->setGravity(btVector3(0, 0, 0));

    wrld->_aabb_intersect = &_ext_collider_obb;
    wrld->_terrain_ray_intersect_broadphase = &_ext_terrain_ray_intersect_broadphase;

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

////////////////////////////////////////////////////////////////////////////////
void physics::set_simulation_frame(uint frame)
{
    gOuterraSimulationFrame = frame;
}

////////////////////////////////////////////////////////////////////////////////
void physics::debug_draw_world(btScalar extrapolation_step) {
    if (_dbg_drawer) {
        _world->debugDrawWorld(extrapolation_step);
    }
}

////////////////////////////////////////////////////////////////////////////////
void physics::get_broadphase_handles_aabbs(const bt::external_broadphase* broadphase, coid::dynarray<double3>& minmaxes)
{
    bt32BitAxisSweep3* bp = nullptr;
    uint revision = 0xffffffff;
    if (broadphase)
    {
        bp = broadphase->_broadphase;
        revision = broadphase->_revision;
    }
    else
    {
        bp = static_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
    }

    _world->for_each_object_in_broadphase(bp, revision, [&minmaxes](btCollisionObject* co) {
        btBroadphaseProxy* bp = co->getBroadphaseHandle();
        if (bp && (co->getCollisionFlags() & btCollisionObject::CollisionFlags::CF_DISABLE_VISUALIZE_OBJECT) == 0)
        {
            minmaxes.push(double3(bp->m_aabbMin[0], bp->m_aabbMin[1], bp->m_aabbMin[2]));
            minmaxes.push(double3(bp->m_aabbMax[0], bp->m_aabbMax[1], bp->m_aabbMax[2]));
        }
    });
}

////////////////////////////////////////////////////////////////////////////////
bt::external_broadphase* physics::create_external_broadphase(const double3& min, const double3& max)
{
    return _world->create_external_broadphase(min, max);
}

////////////////////////////////////////////////////////////////////////////////
void physics::delete_external_broadphase(bt::external_broadphase* bp)
{
    return _world->delete_external_broadphase(bp);
}

////////////////////////////////////////////////////////////////////////////////
void physics::update_external_broadphase(bt::external_broadphase* bp)
{
    return _world->update_terrain_mesh_broadphase(bp);
}

////////////////////////////////////////////////////////////////////////////////
bool physics::add_collision_object_to_external_broadphase(bt::external_broadphase* bp, btCollisionObject* co, unsigned int group, unsigned int mask)
{
    if (bp->_broadphase->is_full()) {
        return false;
    }

    btTransform trans = co->getWorldTransform();

    btVector3	minAabb;
    btVector3	maxAabb;
    co->getCollisionShape()->getAabb(trans, minAabb, maxAabb);

    int type = co->getCollisionShape()->getShapeType();
    co->setBroadphaseHandle(bp->_broadphase->createProxy(
        minAabb,
        maxAabb,
        type,
        co,
        group,
        mask,
        0, 0
    ));

    //bp->_colliders.push(sc);

    return true;
}
/*
////////////////////////////////////////////////////////////////////////////////
void physics::remove_collision_object_from_external_broadphase(bt::external_broadphase * bp, simple_collider * sc, btCollisionObject * co)
{
    for (uints i = 0; i < bp->_colliders.size(); i++) {
        if (bp->_colliders[i] == sc) {
            bp->_colliders.del(i);
            break;
        }
    }

    bp->_broadphase.destroyProxy(co->getBroadphaseHandle(),nullptr);
}*/

////////////////////////////////////////////////////////////////////////////////
btCollisionObject* physics::query_volume_sphere(const double3& pos, float rad, const void* exclude_object)
{
#ifdef _DEBUG
    bt32BitAxisSweep3* broad = dynamic_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
    DASSERT(broad != nullptr);
#else
    bt32BitAxisSweep3* broad = static_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
#endif

    btCollisionObject* result = 0;

    _world->query_volume_sphere(broad, pos, rad, [&](btCollisionObject* obj) {
        if (obj->getUserPointer() && obj->getUserPointer() != exclude_object) {
            result = obj;
            return true;
        }
        return false;
    });

    THREAD_LOCAL_SINGLETON_DEF(coid::dynarray<bt::external_broadphase*>) ebps;
    ebps->reset();
    _physics->external_broadphases_in_radius(_world->getContext(), pos, rad, gCurrentFrame, *ebps);

    ebps->for_each([&](bt::external_broadphase* ebp) {
        /*if (ebp->_dirty) {
            _world->update_terrain_mesh_broadphase(ebp);
        }

        DASSERT(!ebp->_dirty);*/

        _world->query_volume_sphere(ebp->_broadphase, pos, rad, [&](btCollisionObject* obj) {
            if (obj->getUserPointer() && obj->getUserPointer() != exclude_object) {
                result = obj;
                return true;
            }
            return false;
        });
    });

    return result;
}

////////////////////////////////////////////////////////////////////////////////
void physics::query_volume_sphere(const double3& pos, float rad, coid::dynarray<btCollisionObject*>& result)
{
#ifdef _DEBUG
    bt32BitAxisSweep3* broad = dynamic_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
    DASSERT(broad != nullptr);
#else
    bt32BitAxisSweep3* broad = static_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
#endif

    _world->query_volume_sphere(broad, pos, rad, [&](btCollisionObject* obj) {
        if (obj->getUserPointer())
            result.push(obj);
        return false;
    });

    THREAD_LOCAL_SINGLETON_DEF(coid::dynarray<bt::external_broadphase*>) ebps;
    ebps->reset();
    _physics->external_broadphases_in_radius(_world->getContext(), pos, rad, gCurrentFrame, *ebps);

    ebps->for_each([&](bt::external_broadphase* ebp) {
        /*if (ebp->_dirty) {
            _world->update_terrain_mesh_broadphase(ebp);
        }

        DASSERT(!ebp->_dirty);*/

        _world->query_volume_sphere(ebp->_broadphase, pos, rad, [&](btCollisionObject* obj) {
            if (obj->getUserPointer())
                result.push(obj);
            return false;
        });
    });
}

////////////////////////////////////////////////////////////////////////////////
void physics::query_volume_frustum(const double3& pos, const float4* f_planes_norms, uint8 nplanes, bool include_partial, coid::dynarray<btCollisionObject*>& result)
{
#ifdef _DEBUG
    bt32BitAxisSweep3* broad = dynamic_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
    DASSERT(broad != nullptr);
#else
    bt32BitAxisSweep3* broad = static_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
#endif


    _world->query_volume_frustum(broad, pos, f_planes_norms, nplanes, include_partial, [&](btCollisionObject* obj) {
        result.push(obj);
    });

    THREAD_LOCAL_SINGLETON_DEF(coid::dynarray<bt::external_broadphase*>) ebps;
    ebps->reset();
    _physics->external_broadphases_in_frustum(_world->getContext(), pos, f_planes_norms, nplanes, gCurrentFrame, *ebps);

    ebps->for_each([&](bt::external_broadphase* ebp) {
        /*if (ebp->_dirty) {
            _world->update_terrain_mesh_broadphase(ebp);
        }

        DASSERT(!ebp->_dirty);*/

        _world->query_volume_frustum(ebp->_broadphase, pos, f_planes_norms, nplanes, include_partial, [&](btCollisionObject* obj) {
            if (obj->getUserPointer())
                result.push(obj);
        });
    });
}

////////////////////////////////////////////////////////////////////////////////
void physics::wake_up_objects_in_radius(const double3& pos, float rad) {
#ifdef _DEBUG
    bt32BitAxisSweep3* broad = dynamic_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
    DASSERT(broad != nullptr);
#else
    bt32BitAxisSweep3* broad = static_cast<bt32BitAxisSweep3*>(_world->getBroadphase());
#endif

    _world->query_volume_sphere(broad, pos, rad, [&](btCollisionObject* obj) {
        obj->setActivationState(ACTIVE_TAG);
        obj->setDeactivationTime(0);
        return false;
    });
}

////////////////////////////////////////////////////////////////////////////////
void physics::wake_up_object(btCollisionObject* obj) {
    obj->setActivationState(ACTIVE_TAG);
    obj->setDeactivationTime(0);
}

////////////////////////////////////////////////////////////////////////////////
bool physics::is_point_inside_terrain_ocluder(const double3& pt)
{
    return _world->is_point_inside_terrain_occluder(btVector3(pt.x, pt.y, pt.z));
}

////////////////////////////////////////////////////////////////////////////////
bool physics::contact_pair_test(btCollisionObject* a, btCollisionObject* b)
{
    ot::is_inside_callback cbk;
    _world->contactPairTest(a, b, cbk);

    return cbk.is_inside;
}

void physics::pause_simulation(bool pause)
{
    _world->pause_simulation(pause);
}
////////////////////////////////////////////////////////////////////////////////
btTriangleMesh* physics::create_triangle_mesh()
{
    btTriangleMesh* result = new btTriangleMesh();
    return result;
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_triangle_mesh(btTriangleMesh* triangle_mesh)
{
    DASSERT(triangle_mesh);
    delete triangle_mesh;
}

////////////////////////////////////////////////////////////////////////////////
void physics::add_triangle(btTriangleMesh* mesh, const float3& v0, const float3& v1, const float3& v2)
{
    btVector3 btv0(v0.x, v0.y, v0.z);
    btVector3 btv1(v1.x, v1.y, v1.z);
    btVector3 btv2(v2.x, v2.y, v2.z);

    mesh->addTriangle(btv0, btv1, btv2);
}

////////////////////////////////////////////////////////////////////////////////
btCollisionShape* physics::create_shape(bt::EShape sh, const float3& hvec, void* data)
{
    switch (sh) {
    case bt::SHAPE_CONVEX:          return new btConvexHullShape();
    case bt::SHAPE_MESH_STATIC:     return new btBvhTriangleMeshShape(reinterpret_cast<btTriangleMesh*>(data), false);
    case bt::SHAPE_SPHERE:          return new btSphereShape(hvec[0]);
    case bt::SHAPE_BOX:             return new btBoxShape(btVector3(hvec[0], hvec[1], hvec[2]));
    case bt::SHAPE_CYLINDER:        return new btCylinderShapeZ(btVector3(hvec[0], hvec[1], hvec[2]));
    case bt::SHAPE_CAPSULE: {
        if (glm::abs(hvec[1] - hvec[2]) < 0.000001f) {
            //btCapsuleX
            return new btCapsuleShapeX(hvec[1], 2.f * (hvec[0] - hvec[1]));
        }
        else if (glm::abs(hvec[0] - hvec[2]) < 0.000001f) {
            //btCapsuleY
            return new btCapsuleShape(hvec[0], 2.f * (hvec[1] - hvec[0]));
        }
        else {
            //btCapsuleZ
            return new btCapsuleShapeZ(hvec[1], 2.f * (hvec[2] - hvec[1]));
        }
    }
    case bt::SHAPE_CONE:    return new btConeShapeZ(hvec[0], hvec[2]);
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
ifc_fn btCollisionShape* physics::clone_shape(const btCollisionShape* shape)
{
    return shape->getClone();
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_shape(btCollisionShape*& shape)
{
    delete shape;
    shape = 0;
}

////////////////////////////////////////////////////////////////////////////////
void physics::add_convex_point(btCollisionShape* shape, const float3& pt)
{
    static_cast<btConvexHullShape*>(shape)->addPoint(btVector3(pt[0], pt[1], pt[2]), false);
}

////////////////////////////////////////////////////////////////////////////////
void physics::close_convex_shape(btCollisionShape* shape)
{
    shape->setMargin(0.005);
    static_cast<btConvexHullShape*>(shape)->recalcLocalAabb();
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_collision_shape_local_scaling(btCollisionShape* shape, const float3& scale)
{
    shape->setLocalScaling(btVector3(scale[0], scale[1], scale[2]));
}

////////////////////////////////////////////////////////////////////////////////
btCompoundShape* physics::create_compound_shape()
{
    btCompoundShape* res = new btCompoundShape();
    return res;
}

////////////////////////////////////////////////////////////////////////////////
void physics::add_child_shape(btCompoundShape* group, btCollisionShape* child, const btTransform& tr)
{
    group->addChildShape(tr, child);
}

////////////////////////////////////////////////////////////////////////////////
void physics::remove_child_shape(btCompoundShape* group, btCollisionShape* child)
{
    group->removeChildShape(child);
}

////////////////////////////////////////////////////////////////////////////////
void physics::update_child(btCompoundShape* group, btCollisionShape* child, const btTransform& tr)
{
    int index = -1;
    const int num_children = group->getNumChildShapes();
    for (int i = 0; i < num_children; i++) {
        if (group->getChildShape(i) == child) {
            index = i;
            break;
        }
    }

    group->updateChildTransform(index, tr, false);
}

////////////////////////////////////////////////////////////////////////////////
void physics::get_child_transform(btCompoundShape* group, btCollisionShape* child, btTransform& tr)
{
    int index = -1;
    const int num_children = group->getNumChildShapes();
    for (int i = 0; i < num_children; i++) {
        if (group->getChildShape(i) == child) {
            index = i;
            break;
        }
    }

    tr = group->getChildTransform(index);
}

////////////////////////////////////////////////////////////////////////////////
void physics::recalc_compound_shape(btCompoundShape* shape)
{
    shape->recalculateLocalAabb();
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_compound_shape(btCompoundShape*& shape)
{
    delete shape;
    shape = 0;
}



////////////////////////////////////////////////////////////////////////////////
btCollisionObject* physics::create_collision_object(btCollisionShape* shape, void* usr1, void* usr2)
{
    btCollisionObject* obj = new btCollisionObject;
    obj->setCollisionShape(shape);

    obj->setUserPointer(usr1);
    obj->m_userDataExt = usr2;

    return obj;
}

////////////////////////////////////////////////////////////////////////////////
btGhostObject* physics::create_ghost_object(btCollisionShape* shape, void* usr1, void* usr2, bt::EOtFlags ot_flags)
{
    btGhostObject* obj = new btGhostObject;
    obj->setCollisionShape(shape);

    obj->setUserPointer(usr1);
    obj->m_userDataExt = usr2;

    obj->m_otFlags = ot_flags;

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
void physics::destroy_collision_object(btCollisionObject*& obj)
{
    if (obj) delete obj;
    obj = 0;
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_ghost_object(btGhostObject*& obj)
{
    if (obj) delete obj;
    obj = 0;
}

////////////////////////////////////////////////////////////////////////////////
bool physics::add_collision_object(btCollisionObject* obj, unsigned int group, unsigned int mask, bool inactive)
{
    if (inactive)
        obj->setActivationState(DISABLE_SIMULATION);

    /* if (obj->isStaticObject()) {
         float3x3 basis;
         double3 cen;

         _world->get_obb(obj->getCollisionShape(),obj->getWorldTransform(),cen,basis);
         add_static_collider(_world->get_context(),obj,cen,basis);
     }
     else {
         _world->addCollisionObject(obj, group, mask);
     }*/

    if (!_world->addCollisionObject(obj, group, mask)) {
        return false;
    }


    btGhostObject* ghost = btGhostObject::upcast(obj);
    if (ghost) {
        obj->setCollisionFlags(obj->getCollisionFlags() | btCollisionObject::CollisionFlags::CF_NO_CONTACT_RESPONSE | btCollisionObject::CollisionFlags::CF_DISABLE_VISUALIZE_OBJECT);
        _world->add_terrain_occluder(ghost);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool physics::add_sensor_object(btGhostObject* obj, unsigned int group, unsigned int mask)
{
    if (!_world->addCollisionObject(obj, group, mask)) 
    {
        return false;
    }
    
    obj->setCollisionFlags(obj->getCollisionFlags() | btCollisionObject::CollisionFlags::CF_NO_CONTACT_RESPONSE);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
void physics::remove_collision_object(btCollisionObject* obj)
{
    _world->removeCollisionObject(obj);

    btGhostObject* ghost = btGhostObject::upcast(obj);
    if (ghost) {
        _world->remove_terrain_occluder(ghost);
    }
}

////////////////////////////////////////////////////////////////////////////////
void physics::remove_collision_object_external(btCollisionObject* obj)
{
    _world->removeCollisionObject_external(obj);

    btGhostObject* ghost = btGhostObject::upcast(obj);
    if (ghost) {
        _world->remove_terrain_occluder(ghost);
    }
}

void physics::remove_sensor_object(btGhostObject* obj)
{
    _world->removeCollisionObject(obj);
}

////////////////////////////////////////////////////////////////////////////////
int physics::get_collision_flags(const btCollisionObject* co)
{
    return co->getCollisionFlags();
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_collision_flags(btCollisionObject* co, int flags)
{
    return co->setCollisionFlags(flags);
}

////////////////////////////////////////////////////////////////////////////////
void physics::force_update_aabbs()
{
    _world->updateAabbs();
}

////////////////////////////////////////////////////////////////////////////////
void physics::update_collision_object(btCollisionObject* obj, const btTransform& tr, bool update_aabb)
{
    obj->setWorldTransform(tr);

    if (update_aabb || obj->getBroadphaseHandle()) {
        //_world->updateSingleAabb(obj);
        obj->m_otFlags |= bt::OTF_TRANSFORMATION_CHANGED;
    }
}

////////////////////////////////////////////////////////////////////////////////
void physics::step_simulation(double step, bt::bullet_stats* stats)
{
    _world->set_ot_stats(stats);
    _world->stepSimulation(step, 0, step);
}

////////////////////////////////////////////////////////////////////////////////
void physics::ray_test(const double3& from, const double3& to, void* cb, bt::external_broadphase* bp)
{
    btVector3 afrom = btVector3(from[0], from[1], from[2]);
    btVector3 ato = btVector3(to[0], to[1], to[2]);

    if (bp && bp->_dirty) {
        DASSERT(0 && "should be already updated");
        //_world->update_terrain_mesh_broadphase(bp);
    }

    _world->rayTest(afrom, ato, *(btCollisionWorld::RayResultCallback*)cb, bp);
}

void physics::set_current_frame(uint frame)
{
    gCurrentFrame = frame;
}

////////////////////////////////////////////////////////////////////////////////

bt::ot_world_physics_stats physics::get_stats()
{
    return ((ot::discrete_dynamics_world*)(_world))->get_stats();
}

////////////////////////////////////////////////////////////////////////////////

bt::ot_world_physics_stats* physics::get_stats_ptr()
{
    return const_cast<bt::ot_world_physics_stats*>(&((ot::discrete_dynamics_world*)(_world))->get_stats());
}

////////////////////////////////////////////////////////////////////////////////

void physics::set_debug_draw_enabled(btIDebugDraw* debug_drawer)
{
    _dbg_drawer = debug_drawer;
    ((ot::discrete_dynamics_world*)_world)->setDebugDrawer(debug_drawer);

    if (debug_drawer) {
        debug_drawer->setDebugMode(_dbg_draw_mode);
    }
}

////////////////////////////////////////////////////////////////////////////////

void physics::set_debug_drawer_mode(int debug_mode)
{
    _dbg_draw_mode = debug_mode;

    if (_dbg_drawer) {
        _dbg_drawer->setDebugMode(debug_mode);
    }
}

////////////////////////////////////////////////////////////////////////////////

btTypedConstraint* physics::add_constraint_ball_socket(btRigidBody* rb_a, const btVector3& pivot_a, btRigidBody* rb_b, const btVector3& pivot_b, bool disable_collision)
{
    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*rb_a, *rb_b, pivot_a, pivot_b);
    _physics->_world->addConstraint(p2p, disable_collision);
    return p2p;
}

////////////////////////////////////////////////////////////////////////////////

void physics::remove_constraint(btTypedConstraint* constraint) {
    _physics->_world->removeConstraint(constraint);
}

////////////////////////////////////////////////////////////////////////////////

void physics::get_triggered_sensors(coid::dynarray32<std::pair<btGhostObject*, btCollisionObject*>>& result_out)
{
    _world->get_trigerred_sensors(result_out);
}

