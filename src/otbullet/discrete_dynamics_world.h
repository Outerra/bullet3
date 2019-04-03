#ifndef OT_DISCRETE_WORLD_DYNAMICS_H
#define OT_DISCRETE_WORLD_DYNAMICS_H

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>

#include "physics_cfg.h"

#include <ot/glm/glm_types.h>
#include <ot/glm/coal.h>

#include <comm/dynarray.h>
#include <comm/hash/slothash.h>
#include <comm/alloc/slotalloc.h>
#include <comm/local.h>
#include <comm/taskmaster.h>

#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>

//#include <ot/logger.h>
//#include <ot/sketch.h>

#define TREE_COLLISION_TIME   0.15 // 150ms

class btDispatcher;
class btBroadphaseInterface;
class btManifoldResult;
struct skewbox;
class ot_terrain_contact_common;
class btGhostObject;

namespace bt {
    class terrain_mesh_broadphase;
}

namespace ot {

struct tree_flex_inf {
    float3 _flex;
    uint16 _tree_iden;
    tree_flex_inf(float3 flex, uint16 iden)
        :_flex(flex)
        , _tree_iden(iden)
    {}
};

///
struct tree_collision_pair
{
    btCollisionObject* obj;
    uint tree_col_info;

    bool reused;
    btPersistentManifold * manifold;

    bt::tree_collision_contex tc_ctx;

    bool operator==(const tree_collision_pair & tcp) const {
        return obj == tcp.obj && tree_col_info == tcp.tree_col_info;
    }

    tree_collision_pair()
        : obj(0)
        , tree_col_info(-1)

        , reused(false)
        , manifold(0)
    {}

    tree_collision_pair(btCollisionObject* col_obj, uint bid, uint8 tid)
        :tree_collision_pair(){
        obj = col_obj;
        tree_col_info = bid << 4 | (tid &0xf);
    }

    void init_with (btCollisionObject* col_obj, uint bid, uint8 tid,const bt::tree& tree_props) {
        obj = col_obj;
        tree_col_info = bid << 4 | (tid & 0xf);
        tc_ctx.tree_identifier = tree_props.identifier;
    }
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

struct p_treebatch_key_extractor {
    typedef uints ret_type;
    uints operator()(const bt::tree_batch* tb) const{
        return (uints)tb;
    }
};

struct tree_key_extractor {
    typedef uint16 ret_type;

    ret_type operator()(const bt::tree* t) const {
        return t->identifier;
    }

    ret_type operator()(const bt::tree& t) const {
        return t.identifier;
    }

    ret_type operator()(const tree_flex_inf* t) const {
        return t->_tree_iden;
    }

    ret_type operator()(const tree_flex_inf& t) const {
        return t._tree_iden;
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

    coid::taskmaster * _task_master;

    btRigidBody * _planet_body;
    btCollisionObjectWrapper * _pb_wrap;
    coid::slotalloc<btPersistentManifold *> _manifolds;
	//coid::slotalloc<tree_batch> _tree_cache;
    coid::slotalloc<tree_collision_pair> _tree_collision_pairs;
    coid::dynarray<btCollisionObjectWrapperCtorArgs> _cow_internal;
    coid::dynarray<compound_processing_entry> _compound_processing_stack;

    coid::dynarray<bt::triangle> _triangles;
    coid::slotalloc<bt::tree_batch> _tb_cache;
    //void * _relocation_offset;

    coid::dynarray<uint> _tree_batches;

    coid::slotalloc<btBroadphasePair> _terrain_mesh_broadphase_pairs;

    coid::dynarray<bt::external_broadphase*> _debug_external_broadphases;

    coid::slotalloc_pool<bt::external_broadphase> _external_broadphase_pool;

    double3 _from;
    float3 _ray;
    float _rad;
    float3x3 _basis;
    float _lod_dim;

    bt::ot_world_physics_stats _stats;

    coid::dynarray<bt::triangle> _debug_terrain_triangles;
    coid::dynarray<btVector3> _debug_lines;
    /*coid::slothash<bt::tree*, uint16, tree_key_extractor> _debug_terrain_trees;
    coid::slothash<tree_flex_inf, uint16, tree_key_extractor> _debug_terrain_trees_active;
    */
    coid::dynarray<uint> _debug_trees;

    coid::local<ot_terrain_contact_common> _common_data;

public:
    bt::external_broadphase * create_external_broadphase(const double3& min, const double3& max);

#ifdef _DEBUG
    void dump_triangle_list_to_obj(const char * fname,float off_x, float off_y, float off_z, float rx, float ry, float rz, float rw);
#endif
    void process_terrain_broadphases(const coid::dynarray<bt::external_broadphase*>& broadphase, btCollisionObject * col_obj);
    void update_terrain_mesh_broadphase(bt::external_broadphase* bp);
    void add_terrain_broadphase_collision_pair(btCollisionObject * obj1, btCollisionObject * obj2);
    void remove_terrain_broadphase_collision_pair(btBroadphasePair& pair);
    void process_terrain_broadphase_collision_pairs();

    void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback, bt::external_broadphase* bp) const;

    virtual void removeRigidBody(btRigidBody* body) override;
    virtual void removeCollisionObject(btCollisionObject* collisionObject) override;

    virtual void debugDrawWorld() override;

    typedef bool (*fn_ext_collision)(
        const void* context,
        const double3& center,
        float radius,
        float lod_dimension,
        coid::dynarray<bt::triangle>& data,
        coid::dynarray<uint>& trees,
        coid::slotalloc<bt::tree_batch>& tree_batches,
        uint frame );

    typedef int(*fn_ext_collision_2)(
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
        float3& under_normal,
        coid::dynarray<bt::external_broadphase*>& broadphases);

    typedef float3(*fn_process_tree_collision)(btRigidBody * obj, bt::tree_collision_contex & ctx, float time_step, coid::slotalloc<bt::tree_batch>& tree_batches );

    typedef float(*fn_terrain_ray_intersect)(
        const void* context,
        const double3& from,
        const float3& dir,
        const float2& minmaxlen,
        float3* norm,
        double3* pos);

    typedef float(*fn_elevation_above_terrain)(const double3& pos,
        float maxlen,
        float3* norm,
        double3* hitpoint);

    fn_ext_collision_2 _aabb_intersect;
    fn_terrain_ray_intersect _terrain_ray_intersect;
    fn_elevation_above_terrain _elevation_above_terrain;

    discrete_dynamics_world(btDispatcher* dispatcher,
        btBroadphaseInterface* pairCache,
        btConstraintSolver* constraintSolver,
        btCollisionConfiguration* collisionConfiguration,
        fn_ext_collision ext_collider,
        fn_process_tree_collision ext_tree_col,
        fn_terrain_ray_intersect ext_terrain_ray_intersect,
        fn_elevation_above_terrain ext_elevation_above_terrain,
        const void* context = 0,
        coid::taskmaster * tm = 0);

    const bt::ot_world_physics_stats & get_stats() const {
        return _stats;
    }

    void get_obb(const btCollisionShape * cs, const btTransform& t, double3& cen, float3x3& basis);
    
    template<class fn> // void (*fn)(btCollisionObject * obj)
    void query_volume_sphere(bt32BitAxisSweep3 * broadphase, const double3& pos, float rad, fn process_fn)
    {
        static coid::dynarray<const btDbvtNode *> _processing_stack(1024);
        _processing_stack.reset();

        const btDbvtBroadphase* raycast_acc = broadphase->getRaycastAccelerator();
        DASSERT(raycast_acc);

        const btDbvt * dyn_set = &raycast_acc->m_sets[0];
        const btDbvt * stat_set = &raycast_acc->m_sets[1];

        const btDbvtNode * cur_node = nullptr;

        if (dyn_set && dyn_set->m_root) {
            _processing_stack.push(dyn_set->m_root);
        }

        if (stat_set && stat_set->m_root) {
            _processing_stack.push(stat_set->m_root);
        }

        while (_processing_stack.pop(cur_node)) {
            const btVector3& bt_aabb_cen = cur_node->volume.Center();
            const btVector3& bt_aabb_half = cur_node->volume.Extents();
            double3 aabb_cen(bt_aabb_cen[0], bt_aabb_cen[1], bt_aabb_cen[2]);
            double3 aabb_half(bt_aabb_half[0], bt_aabb_half[1], bt_aabb_half[2]);
            if (coal::intersects_sphere_aabb(pos, (double)rad, aabb_cen, aabb_half, (double*)nullptr)) {
                if (cur_node->isleaf()) {
                    if (cur_node->data) {
                        btDbvtProxy* dat = reinterpret_cast<btDbvtProxy*>(cur_node->data);
                        process_fn(reinterpret_cast<btCollisionObject*>(dat->m_clientObject));
                    }
                }
                else {
                    _processing_stack.push(cur_node->childs[0]);
                    _processing_stack.push(cur_node->childs[1]);
                }
            }
        }
    }


    template<typename fn> //void(*fn)(btBroadphaseProxy * proxy);
    void query_volume_aabb(bt32BitAxisSweep3 * broadphase, const double3& aabb_cen, const double3& aabb_half, fn process_fn)
    {
        static coid::dynarray<const btDbvtNode *> _processing_stack(1024);
        _processing_stack.reset();

        const btDbvtBroadphase* raycast_acc = broadphase->getRaycastAccelerator();
        DASSERT(raycast_acc);

        const btDbvt * dyn_set = &raycast_acc->m_sets[0];
        const btDbvt * stat_set = &raycast_acc->m_sets[1];

        const btDbvtNode * cur_node = nullptr;

        if (dyn_set && dyn_set->m_root) {
            _processing_stack.push(dyn_set->m_root);
        }

        if (stat_set && stat_set->m_root) {
            _processing_stack.push(stat_set->m_root);
        }

        btCollisionObject p_obj;

        while (_processing_stack.pop(cur_node)) {
            const btVector3& bt_node_aabb_cen = cur_node->volume.Center();
            const btVector3& bt_node_aabb_half = cur_node->volume.Extents();
            double3 node_aabb_cen(bt_node_aabb_cen[0], bt_node_aabb_cen[1], bt_node_aabb_cen[2]);
            double3 node_aabb_half(bt_node_aabb_half[0], bt_node_aabb_half[1], bt_node_aabb_half[2]);

            if (coal::intersects_aabb_aabb(node_aabb_cen, node_aabb_half, aabb_cen, aabb_half)) {

                if (cur_node->isleaf()) {
                    //add_debug_aabb(bt_node_aabb_cen - bt_node_aabb_half, bt_node_aabb_cen + bt_node_aabb_half, btVector3(1, 0, 0));
                    if (cur_node->data) {
                        btDbvtProxy* dat = reinterpret_cast<btDbvtProxy*>(cur_node->data);
                        process_fn(reinterpret_cast<btCollisionObject*>(dat->m_clientObject)->getBroadphaseHandle());
                    }
                }
                else {
                    //add_debug_aabb(bt_node_aabb_cen - bt_node_aabb_half, bt_node_aabb_cen + bt_node_aabb_half, btVector3(1, 1, 1));
                    _processing_stack.push(cur_node->childs[0]);
                    _processing_stack.push(cur_node->childs[1]);
                }
            }
        }
    }


    template<class fn> // void (*fn)(btCollisionObject * obj)
    void query_volume_frustum(const double3&pos, const float4 * f_planes_norms, uint8 nplanes, bool include_partial, fn process_fn)
    {
#ifdef _DEBUG
        bt32BitAxisSweep3 * broad = dynamic_cast<bt32BitAxisSweep3 *>(m_broadphasePairCache);
        DASSERT(broad != nullptr);
#else
        bt32BitAxisSweep3 * broad = static_cast<bt32BitAxisSweep3 *>(m_broadphasePairCache);
#endif
        static coid::dynarray<const btDbvtNode *> _processing_stack(1024);
        _processing_stack.reset();

        const btDbvtBroadphase* raycast_acc = broad->getRaycastAccelerator();
        DASSERT(raycast_acc);

        const btDbvt * dyn_set = &raycast_acc->m_sets[0];
        const btDbvt * stat_set = &raycast_acc->m_sets[1];

        const btDbvtNode * cur_node = nullptr;

        if (dyn_set && dyn_set->m_root) {
            _processing_stack.push(dyn_set->m_root);
        }

        if (stat_set && stat_set->m_root) {
            _processing_stack.push(stat_set->m_root);
        }

        btCollisionObject p_obj;

        while (_processing_stack.pop(cur_node)) {
            const btVector3& bt_aabb_cen = cur_node->volume.Center();
            const btVector3& bt_aabb_half = cur_node->volume.Extents();
            double3 aabb_cen(bt_aabb_cen[0], bt_aabb_cen[1], bt_aabb_cen[2]);
            float3 aabb_half(bt_aabb_half[0], bt_aabb_half[1], bt_aabb_half[2]);

            if (coal::intersects_frustum_aabb(aabb_cen, aabb_half, pos, f_planes_norms, nplanes, true)) {
                if (cur_node->isleaf()) {
                    if (cur_node->data) {
                        btDbvtProxy* dat = reinterpret_cast<btDbvtProxy*>(cur_node->data);
                        btCollisionObject* leaf_obj = reinterpret_cast<btCollisionObject*>(dat->m_clientObject);

                        const btVector3& cen = leaf_obj->getWorldTransform().getOrigin();
                        const float3 aabb_pos(float(cen[0] - pos.x), float(cen[1] - pos.y), float(cen[2] - pos.z));
                        bool passes = true;
                        for (uint8 p = 0; p < nplanes; p++) {
                            float3 n(f_planes_norms[p]);
                            btVector3 min, max;
                            btTransform t(btMatrix3x3(n.x, n.y, n.z, 0., 0., 0., 0., 0., 0.));
                            leaf_obj->getCollisionShape()->getAabb(t, min, max);
                            const float np = float(max[0] - min[0]) * 0.5f;
                            const float mp = glm::dot(n, aabb_pos) + f_planes_norms[p].w;
                            if ((include_partial ? mp + np : mp - np) < 0.0f) {
                                passes = false;
                                break;
                            }
                        }

                        if (passes) {
                            process_fn(leaf_obj);
                        }
                    }
                }
                else {
                    _processing_stack.push(cur_node->childs[0]);
                    _processing_stack.push(cur_node->childs[1]);
                }
            }
        }
    }

    void add_terrain_occluder(btGhostObject * go) { _terrain_occluders.push(go); }
    void remove_terrain_occluder(btGhostObject * go);
    bool is_point_inside_terrain_occluder(const btVector3& pt);
    void set_potential_collision_flag(btRigidBody * rb);
    template<class fn> // void (*fn)(btCollisionObject * obj)
    void for_each_object_in_broadphase(bt32BitAxisSweep3 * broadphase, uint revision, fn process_fn) {
        static coid::dynarray<const btDbvtNode *> _processing_stack(1024);
        _processing_stack.reset();

        const btDbvtBroadphase* raycast_acc = broadphase->getRaycastAccelerator();
        DASSERT(raycast_acc);

        const btDbvt * dyn_set = &raycast_acc->m_sets[0];
        const btDbvt * stat_set = &raycast_acc->m_sets[1];

        const btDbvtNode * cur_node = nullptr;

        if (dyn_set && dyn_set->m_root) {
            _processing_stack.push(dyn_set->m_root);
        }

        if (stat_set && stat_set->m_root) {
            _processing_stack.push(stat_set->m_root);
        }

        btCollisionObject p_obj;

        while (_processing_stack.pop(cur_node)) {
            const btVector3& bt_node_aabb_cen = cur_node->volume.Center();
            const btVector3& bt_node_aabb_half = cur_node->volume.Extents();
            double3 node_aabb_cen(bt_node_aabb_cen[0], bt_node_aabb_cen[1], bt_node_aabb_cen[2]);
            double3 node_aabb_half(bt_node_aabb_half[0], bt_node_aabb_half[1], bt_node_aabb_half[2]);

            if (cur_node->isleaf()) {
                if (cur_node->data) {
                    btDbvtProxy* dat = reinterpret_cast<btDbvtProxy*>(cur_node->data);
                    btCollisionObject * co = reinterpret_cast<btCollisionObject*>(dat->m_clientObject);
                    if(co->getBroadphaseHandle() && broadphase->ownsProxy(co->getBroadphaseHandle()) && co->getBroadphaseHandle()->m_ot_revision == revision)
                        process_fn(co);
                }
            }
            else {
                _processing_stack.push(cur_node->childs[0]);
                _processing_stack.push(cur_node->childs[1]);
            }
        }
    }

protected:

    virtual void internalSingleStepSimulation(btScalar timeStep) override;

    void ot_terrain_collision_step();

    void prepare_tree_collision_pairs(btCollisionObject * cur_obj, const coid::dynarray<uint>& trees_cache, uint32 frame);
    void build_tb_collision_info(bt::tree_batch * tb);

    fn_ext_collision _sphere_intersect;
    fn_process_tree_collision _tree_collision;

    void process_tree_collisions(btScalar time_step);
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
        _stats.tri_list_construction_time_ms = 0.f;
        _stats.triangles_processed_count = 0;
        _stats.trees_processed_count = 0;
        _stats.after_ot_phase_time_ms = 0;
        _stats.before_ot_phase_time_ms = 0;
    };

    void repair_tree_collision_pairs();
    void repair_tree_batches();
    bt::tree_collision_info* get_tree_collision_info(const tree_collision_pair& tcp);
    bt::tree* get_tree(const tree_collision_pair& tcp);
    bt::tree* get_tree(uint tree_id);

    void add_debug_aabb(const btVector3& min, const btVector3& max, const btVector3& color);

};

} // namespace ot

#endif