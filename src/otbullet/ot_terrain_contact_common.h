#pragma once
#include <comm/commtypes.h>
#include <comm/dynarray.h>
#include <comm/hash/hashkeyset.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "physics_cfg.h"

class btConvexPolyhedron;
class btBoxShape;
struct btFace;
class btCollisionWorld;
class btRigidBody;
class btCollisionAlgorithm;

namespace ot {
    class discrete_dynamics_world;
}
////////////////////////////////////////////////////////////////////////

struct cached_edge {
	uint32 _vId1;
	uint32 _vId2;
	cached_edge()
		:_vId1(-1),
		_vId2(-1)
	{}
	cached_edge(uint32 i1, uint32 i2) {
		_vId1 = glm::max(i1, i2);
		_vId2 = glm::min(i1, i2);
	}
};

template <class VAL,class KEY>
struct edge_keyext {
	typedef KEY ret_type;

	uint32 operator()(const cached_edge & ce) const {
		return (ce._vId1 << 16) | ce._vId2;
	}
};

enum ColliderType {
	ctSphere = 0,
	ctCapsule,
	ctHull,
	ctBox,
	ctCount
};

//
struct contact_point
{
    double3 point;
    float3 normal;
    float depth;
    uint32 t_idx;

    contact_point() {}
    contact_point(const double3 & p,
        const float3 & n,
        float d,
        uint32 idx)
        : point(p)
        , normal(n)
        , depth(d)
        , t_idx(idx)
    {}
};

bool GJK_contact_added(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);
bool friction_combiner_cbk(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);

bool plane_contact_added(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);


/// Copied from the Bullet btManifoldResult

float calculate_combined_friction(float b1_fric, float b2_fric);
float calculate_combined_restitution(float b1_rest, float b2_rest);
float calculate_combined_rolling_friction(float b1_roll_fric, float b2_roll_fric);

/// ///////////////

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

class ot_terrain_contact_common
{
private:
	void prepare(btManifoldResult * result);
    ////////////////////////////////////////////////////////////////////////////
public:

    COIDNEWDELETE(ot_terrain_contact_common);

	ot_terrain_contact_common(float triangle_collision_margin, ot::discrete_dynamics_world * world, btCollisionObjectWrapper * planet_body_wrap);
    ~ot_terrain_contact_common();

	void set_terrain_mesh_offset(const glm::dvec3 & offset);
	void prepare_sphere_collision(btManifoldResult * result, const glm::dvec3 & center, float radius, float collision_margin);
	void prepare_capsule_collision(btManifoldResult * result, const glm::dvec3 & p0, const glm::dvec3 & p1,float radius,float collision_margin);
	void prepare_bt_convex_collision(btManifoldResult * result, btCollisionObjectWrapper * convex_object);
    void clear_caches();

	void process_triangle_cache();
	void process_triangle_cache(const coid::dynarray<bt::triangle>& triangle_cache);
	void process_collision_points();
	void process_additional_col_objs();

	void collide_sphere_triangle(const bt::triangle & triangle);
    void collide_capsule_triangle(const bt::triangle & triangle);
	void collide_convex_triangle(const bt::triangle & triangle);
    void collide_object_plane(fn_elevation_above_terrain elevation_above_terrain);

	void add_triangle(const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, uint32 ia, uint32 ib, uint32 ic, uint8 flags, const double3 * mesh_offset, uint32 tri_idx);
	void add_additional_col_obj(btCollisionObject * col_obj);

    void set_internal_obj_wrapper(btCollisionObjectWrapper * internal_wrap) { _internal_object = internal_wrap;};
    
    void clear_common_data();

    void set_bounding_sphere_rad(float rad) { _sphere_radius = rad; }
private:
	typedef void(ot_terrain_contact_common::*CollisionAlgorithm)(const bt::triangle &);
	btCollisionObjectWrapper * _planet_body_wrap;
	btManifoldResult * _manifold;
    ot::discrete_dynamics_world * _collision_world;
	coid::dynarray<bt::triangle> _triangle_cache;
	coid::dynarray<contact_point> _contact_point_cache;

	coid::dynarray<btCollisionObject *> _additional_col_objs;

	const float _triangle_collision_margin;
	glm::dvec3 _mesh_offset;

	ColliderType  _curr_collider;
	CollisionAlgorithm _curr_algo;

	float _collider_collision_margin;
	
	glm::dvec3 _sphere_origin_g;
	glm::dvec3 _capsule_p0_g;
	glm::dvec3 _capsule_p1_g;
	glm::dvec3 _hull_center_g;

	glm::vec3 _sphere_origin;
	glm::vec3 _capsule_p0;
	glm::vec3 _capsule_p1;
	
	float _sphere_radius;
	float _capsule_radius;

	const btConvexPolyhedron * _hull_polydata;
	coid::hash_keyset<cached_edge,edge_keyext<cached_edge,uint32> > _cached_edges;

	btCollisionObjectWrapper * _convex_object;
    btCollisionObjectWrapper * _internal_object;
	btTransform _box_local_transform;
	btCollisionAlgorithm * _bt_ca;
};

