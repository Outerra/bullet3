#pragma once

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <ot/glm/glm_types.h>

class rigid_body_constraint;
class terrain_mesh;

namespace bt {

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

    triangle(){}
    triangle(const float3 & va,
        const float3 & vb,
        const float3 & vc,
        uint32 ia,
        uint32 ib,
        uint32 ic,
        uint8 flags,
		const double3 * offsetp)
        : a(va)
        , b(vb)
        , c(vc)
        , idx_a(ia)
        , idx_b(ib)
        , idx_c(ic)
        , t_flags(flags)
		, parent_offset_p(offsetp)
    {}
};

//
struct tree
{
    double3 pos;
    //quat rot;
    float height;

    //uint8 objbuf[sizeof(btCollisionObject)];
    //uint8 shapebuf[sizeof(btCapsuleShape)];
};

struct tree_collision_info
{
    btCollisionObject obj;
    btCapsuleShape shape;
};

struct tree_batch
{
    const terrain_mesh* tm;
    uint last_frame_used;
    uint tree_count;
    tree trees[16];

    uint8 buf[16 * sizeof(tree_collision_info)];

    tree_collision_info* info(int i) { return (tree_collision_info*)buf + i; }
};

//
struct tree_collision_pair
{
    void* obj;
    tree_collision_info* tree;
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


} //namespace bt
