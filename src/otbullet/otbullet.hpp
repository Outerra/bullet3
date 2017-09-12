#pragma once

#include <comm/intergen/ifc.h>

//ifc{
#include "physics_cfg.h"
#include <comm/alloc/slotalloc.h>

class btDynamicsWorld;
class btCollisionShape;
class btCompoundShape;
class btCollisionObject;
class btRigidBody;
class btActionInterface;
class btTransform;
class btIDebugDraw;
class btManifoldPoint;

namespace bt {
    class constraint_info;
    class physics;
    struct ot_world_physics_stats;
}
extern bt::physics* BT;
//}ifc

namespace ot {
    class discrete_dynamics_world;
}


///
class physics : public policy_intrusive_base
{
public:

    ///Interface for the physics module
    ifc_class_var(bt::physics, "", _ifc_host);

    ifc_fn static iref<physics> create( double r, void* context );
	ifc_fn static iref<physics> get();

    ifc_fn void step_simulation( double step );
    ifc_fn void ray_test( const double from[3], const double to[3], void* cb);

    ifc_fn btRigidBody* fixed_object();
    ifc_fn btRigidBody* create_rigid_body( float mass, btCollisionShape* shape, void* usr1, void* usr2 );
    ifc_fn void destroy_rigid_body( btRigidBody*& obj );
    ifc_fn void add_rigid_body( btRigidBody* obj, unsigned int group, unsigned int mask,
        btActionInterface* action, bt::constraint_info* constraint );
    ifc_fn void remove_rigid_body( btRigidBody* obj, btActionInterface* action, bt::constraint_info* constraint );
    ifc_fn void pause_rigid_body( btRigidBody* obj, bool pause );
    ifc_fn void set_rigid_body_mass( btRigidBody* obj, float mass, const float inertia[3] );
    ifc_fn void set_rigid_body_gravity( btRigidBody* obj, const double gravity[3] );
    ifc_fn void set_rigid_body_transform( btRigidBody* obj, const btTransform& tr, const double gravity[3] );
    ifc_fn void predict_rigid_body_transform( btRigidBody* obj, double dt, ifc_out btTransform& tr );


    ifc_fn btCollisionObject* create_collision_object( btCollisionShape* shape, void* usr1, void* usr2 );
    ifc_fn void destroy_collision_object( btCollisionObject*& obj );
    ifc_fn void update_collision_object( btCollisionObject* obj, const btTransform& tr, bool update_aabb );
    ifc_fn void set_collision_info( btCollisionObject* obj, unsigned int group, unsigned int mask );
    ifc_fn void add_collision_object( btCollisionObject* obj, unsigned int group, unsigned int mask, bool inactive );
    ifc_fn void remove_collision_object( btCollisionObject* obj );

    ifc_fn btCompoundShape* create_compound_shape();
    ifc_fn void add_child_shape( btCompoundShape* group, btCollisionShape* child, const btTransform& tr );
    ifc_fn void update_child( btCompoundShape* group, int index, const btTransform& tr );
    ifc_fn void recalc_compound_shape( btCompoundShape* shape );
    ifc_fn void destroy_compound_shape( ifc_inout btCompoundShape*& shape );

    ifc_fn btCollisionShape* create_shape( bt::EShape sh, const float hvec[3] );
    ifc_fn void add_convex_point( btCollisionShape* shape, const float point[3] );
    ifc_fn void close_convex_shape( btCollisionShape* shape );
    ifc_fn void destroy_shape( ifc_inout btCollisionShape*& shape );

    ifc_fn bt::ot_world_physics_stats get_stats();
    ifc_fn bt::ot_world_physics_stats* get_stats_ptr();
    ifc_fn void set_debug_draw_enabled(btIDebugDraw * debug_drawer);
    ifc_fn void set_debug_drawer_mode(int debug_mode);
    ifc_fn void debug_draw_world();

    ifc_fn void query_volume_sphere(const double3 & pos, float rad, ifc_inout coid::dynarray<btCollisionObject*>& result);
    ifc_fn void query_volume_frustum(const double3 & pos, const float4 * f_planes_norms, uint8 nplanes, bool include_partial, ifc_inout coid::dynarray<btCollisionObject *>& result);


    ifc_event bool terrain_collisions(
        const void* context,
        const double3& center,
        float radius,
        float lod_dimension,
        coid::dynarray<bt::triangle>& data,
        coid::dynarray<uint>& trees,
        coid::slotalloc<bt::tree_batch>& tree_batches,
        uint frame );

    ifc_event bool terrain_collisions_aabb(
        const void* context,
        const double3& center,
        float3x3 basis,
        float lod_dimension,
        coid::dynarray<bt::triangle>& data,
        coid::dynarray<uint>& trees,
        coid::slotalloc<bt::tree_batch>& tree_batches,
        uint frame, 
        bool& is_above_tm);

    ifc_event float3 tree_collisions(btRigidBody * obj,
        bt::tree_collision_contex & ctx,
        float time_step, 
        coid::slotalloc<bt::tree_batch>& tree_batches);

    ifc_event float terrain_ray_intersect(
        const void* context,
        const double3& from,
        const float3& dir,
        const float2& minmaxlen,
        float3* norm,
        double3* pos);

    ifc_event float elevation_above_terrain(const double3& pos, 
        float maxlen, 
        float3* norm, 
        double3* hitpoint);

    ifc_event void add_static_collider(const void * context, btCollisionObject * obj, const double3& cen, const float3x3& basis);

    ifc_event void log(const coid::token& text);

private:

    ot::discrete_dynamics_world* _world;
    btIDebugDraw * _dbg_drawer;
    int _dbg_draw_mode;
    
};
