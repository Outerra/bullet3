#pragma once

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <ot/glm/glm_types.h>

#include <comm/dynarray.h>

class rigid_body_constraint;
class terrain_mesh;

class btRigidBody;

class simple_collider;

namespace bt {

enum EOtCollisionFlags {
	CF_POTENTIAL_TUNNEL_COLLISION = 1,
	CF_POTENTIAL_OBJECT_COLLISION = 2,

	CF_COUNT
};

enum EShape {
    SHAPE_NONE = 0,

    SHAPE_CONVEX,
    SHAPE_SPHERE,
    SHAPE_BOX,
    SHAPE_CYLINDER,
    SHAPE_CAPSULE,
    SHAPE_CONE,
};

enum ECollisionShape {
    csLine = 0,
    csSphere,

    csCount
};

struct triangle
{
    float3 a;
    float3 b;
    float3 c;
    uint8 t_flags;
    uint32 idx_a;
    uint32 idx_b;
    uint32 idx_c;
	const double3 * parent_offset_p;
    uint32 tri_idx;
    float fric;
    float roll_fric;
    float rest;

    triangle(){}
    triangle(const float3 & va,
        const float3 & vb,
        const float3 & vc,
        uint32 ia,
        uint32 ib,
        uint32 ic,
        uint8 flags,
		const double3 * offsetp,
        uint32 tri_idx)
        : a(va)
        , b(vb)
        , c(vc)
        , idx_a(ia)
        , idx_b(ib)
        , idx_c(ic)
        , t_flags(flags)
		, parent_offset_p(offsetp)
        , tri_idx(tri_idx)
        , fric(1.0f)
        , roll_fric(1.0f)
        , rest(1.0f)
    {}
};

//
struct tree
{
    double3 pos;
    //quat rot;
    float I;
    float E;
    float sig_max;
    float radius;
    float height;
    float max_flex;
    int8 * spring_force_uv;
    //uint8 objbuf[sizeof(btCollisionObject)];
    //uint8 shapebuf[sizeof(btCapsuleShape)];
};

//
struct tree_collision_info
{
    btCollisionObject obj;
    btCapsuleShape shape;
};

//
struct tree_batch
{
    const terrain_mesh* tm;
    uint tm_version;
    uint16 idx_in_tm;
    uint last_frame_used;
    uint tree_count;
    tree trees[16];

    __declspec(align(16)) uint8 buf[16 * sizeof(tree_collision_info)];

    tree_collision_info* info(int i) { return (tree_collision_info*)buf + i; }
    ~tree_batch() {
        tree_count = 0;
    }
};

//
struct tree_collision_contex {
    uint32 tree_identifier;
    float l;
    float braking_force;
    float collision_duration;
    bool collision_started;
    bool custom_handling;
    btVector3 force_apply_pt;
    btVector3 force_dir;
    btVector3 orig_tree_dir;

    const float max_collision_duration = 0.15f;
    const float max_collision_duration_inv = 1.0f / 0.15f;

    float just_temp_r;

    tree_collision_contex() 
        : tree_identifier(0xffffffff)
    , braking_force(0)
    , collision_duration(0)
    , collision_started(false)
    , custom_handling(false)
    , force_apply_pt(0,0,0)
    , force_dir(0,0,0)
    , orig_tree_dir(0,0,0)
    {}
};

//
struct ot_world_physics_stats {
    uint32 triangles_processed_count;
    uint32 trees_processed_count;
    float total_time_ms;
    float broad_phase_time_ms;
    float triangle_processing_time_ms;
    float tree_processing_time_ms;
    float after_ot_phase_time_ms;
    float before_ot_phase_time_ms;
    float tri_list_construction_time_ms;
    float broad_aabb_intersections_time_ms;
    uint32 broad_aabb_intersections_count;
};

//
class constraint_info
{
public:

    constraint_info()
        : _constraint(0)
    {}

    virtual void getInfo1( btTypedConstraint::btConstraintInfo1* info ) = 0;
    virtual void getInfo2( btTypedConstraint::btConstraintInfo2* info ) = 0;

    virtual ~constraint_info()
    {}

    rigid_body_constraint* _constraint;
};

struct external_broadphase {
    struct broadphase_entry {
        btCollisionObject * _collision_object = nullptr;
        uint _collision_mask = 0;
        uint _collision_group = 0;
        bool _procedural = false;

        broadphase_entry(btCollisionObject * col_obj, uint col_mask, uint col_group, bool procedural = false)
            : _collision_object(col_obj)
            , _collision_mask(col_mask)
            , _collision_group(col_group)
            , _procedural(procedural)
        {}
    };

    bt32BitAxisSweep3 * _broadphase;
    coid::dynarray<broadphase_entry> _entries;
    coid::dynarray<btCollisionObject*> _procedural_objects;
    bool _dirty = false;
    uint _revision = 0;

    external_broadphase(const double3& min, const double3& max) 
    {
        _broadphase = new bt32BitAxisSweep3(btVector3(min.x, min.y, min.z), btVector3(max.x, max.y, max.z), 5000);
    }

};

}; //namespace bt
