#ifndef OT_DISCRETE_WORLD_DYNAMICS_H
#define OT_DISCRETE_WORLD_DYNAMICS_H

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>

#include "physics_cfg.h"

#include <ot/glm/glm_types.h>

#include <comm/dynarray.h>
#include <comm/alloc/slotalloc.h>

//#include <ot/logger.h>
//#include <ot/sketch.h>

class btDispatcher;
class btBroadphaseInterface;
class btManifoldResult;
struct skewbox;
class ot_terrain_contact_common;
class planet_qtree;

namespace ot {
///
struct tree_collision_pair
{
    btCollisionObject* obj;
    bt::tree_collision_info* tree;
    const terrain_mesh * tm;
    bool active;

    const bool operator== (const tree_collision_pair & tcp) const {
        return obj == tcp.obj && tree == tcp.tree;
    }

    tree_collision_pair()
        : obj(0)
        , tree(0)
        , tm(0)
        , active(false)
    {}

    tree_collision_pair(btCollisionObject* obj, bt::tree_collision_info* tree, const terrain_mesh * tm)
        :obj(obj)
        , tree(tree) 
        , reused(false)
        , manifold(0)
        , tree_identifier(0){}
};

///
struct raw_collision_pair {
	btCollisionObject * _obj1;
	btCollisionObject * _obj2;
	raw_collision_pair()
		:_obj1(0), _obj2(0) {}
	raw_collision_pair(btCollisionObject * obj1, btCollisionObject * obj2)
		:_obj1(obj1), _obj2(obj2) {}
};

///
struct compound_processing_entry {
	btCollisionShape * _shape;
	btTransform _world_trans;
	compound_processing_entry(btCollisionShape * shape, const btTransform & world_trans)
		:_shape(shape)
		,_world_trans(world_trans) 
	{};
	compound_processing_entry() {
	}
};

////////////////////////////////////////////////////////////////////////////////
class discrete_dynamics_world : public btDiscreteDynamicsWorld
{
protected:
	struct btCollisionObjectWrapperCtorArgs {
		const btCollisionObjectWrapper* _parent;
		const btCollisionShape* _shape;
		const btCollisionObject* _collisionObject;
		const btTransform _worldTransform;
		int _partId;
		int _index;
		btCollisionObjectWrapperCtorArgs(const btCollisionObjectWrapper* parent, const btCollisionShape* shape, const btCollisionObject* collisionObject, const btTransform& worldTransform, int partId, int index)
			:_parent(parent)
			, _shape(shape)
            , _collisionObject(collisionObject)
			, _worldTransform(worldTransform)
			, _partId(partId)
			, _index(index)
		{};
	private:
		btCollisionObjectWrapperCtorArgs();
	};
	const void* _context;

	btRigidBody * _planet_body;
	btCollisionObjectWrapper * _pb_wrap;
	coid::slotalloc<btPersistentManifold *> _manifolds;
	//coid::slotalloc<tree_batch> _tree_cache;
	coid::slotalloc<tree_collision_pair> _tree_collision_pairs;
	coid::dynarray<btCollisionObjectWrapperCtorArgs> _cow_internal;
	coid::dynarray<compound_processing_entry> _compound_processing_stack;
    //iref<ot::logger> _logger;
	//iref<ot::sketch> _sketch;

    coid::dynarray<bt::triangle> _triangles;
    coid::dynarray<bt::tree_batch*> _trees;

    double3 _from;
    float3 _ray;
    float _rad;
    float3x3 _basis;
    float _lod_dim;
    //bt::ECollisionShape _col_shape;

    bt::ot_world_physics_stats _stats;

public:

    typedef bool (*fn_ext_collision)(
        const void* context,
        const double3& center,
        float radius,
        float lod_dimension,
        coid::dynarray<bt::triangle>& data,
        coid::dynarray<bt::tree_batch*>& trees );

    typedef bool(*fn_ext_collision_2)(
        const void* context,
        const double3& center,
        const float3x3& basis,
        float lod_dimension,
        coid::dynarray<bt::triangle>& data,
        coid::dynarray<bt::tree_batch*>& trees);

    typedef void(*fn_process_tree_collision)(btRigidBody * obj, const btManifoldPoint * cp, uint32 tree_ident);

    fn_ext_collision_2 _aabb_intersect;

	discrete_dynamics_world(btDispatcher* dispatcher,
		btBroadphaseInterface* pairCache,
		btConstraintSolver* constraintSolver,
		btCollisionConfiguration* collisionConfiguration,
        fn_ext_collision ext_collider, 
        fn_process_tree_collision ext_tree_col,
		const void* context = 0);

    const bt::ot_world_physics_stats & get_stats() const {
        return _stats;
    }

protected:

	virtual void internalSingleStepSimulation(btScalar timeStep) override;
    virtual void removeRigidBody(btRigidBody* body) override;

	void ot_terrain_collision_step();

    void process_trees_cache(btCollisionObject * cur_obj, const coid::dynarray<bt::tree_batch*>& trees_cache, uint32 frame);
    void build_tb_collision_info(bt::tree_batch * tb);

    void add_tree_collision_pair(btCollisionObject * obj, bt::tree_collision_info* tree, const terrain_mesh * tm);

    fn_ext_collision _sphere_intersect;
    fn_process_tree_collision _tree_collision;

    void prepare_tree_collisions();
    void process_tree_collisions();
    void get_obb(const btCollisionShape * cs, const btTransform& t, double3& cen, float3x3& basis);
    void oob_to_aabb(const btVector3& src_cen,
        const btMatrix3x3& src_basis,
        const btVector3& dst_cen,
        const btMatrix3x3& dst_basis,
        btVector3& aabb_cen,
        btVector3& aabb_half);

    void reset_stats() {
        _stats.broad_phase_time_ms = 0.f;
        _stats.total_time_ms = 0.f;
        _stats.tree_processing_time_ms = 0.f;
        _stats.triangle_processing_time_ms = 0.f;
        _stats.triangles_processed_count = 0;
        _stats.trees_processed_count = 0;
    };
};

} // namespace ot

#endif