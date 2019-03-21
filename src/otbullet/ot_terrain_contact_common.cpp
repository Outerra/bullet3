#include "ot_terrain_contact_common.h"
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <ot/glm/coal.h>

#include "discrete_dynamics_world.h"

#include <ot/world.h>

const bt::triangle* current_processed_triangle = nullptr;

ot_terrain_contact_common::ot_terrain_contact_common(float triangle_collision_margin, ot::discrete_dynamics_world * world, btCollisionObjectWrapper * planet_body_wrap)
	:_curr_collider(ctCount)
	,_triangle_collision_margin(triangle_collision_margin)
	,_curr_algo(0)
	,_mesh_offset(0)
	,_hull_center_g(0)
	,_hull_polydata(0)
	,_collision_world(world)
	,_planet_body_wrap(planet_body_wrap)
    ,_convex_object(0)
    ,_internal_object(0)
	,_bt_ca(0)
{
	_triangle_cache.reserve(256,true);
	_contact_point_cache.reserve(256, true);
}

ot_terrain_contact_common::~ot_terrain_contact_common()
{
}

void ot_terrain_contact_common::set_terrain_mesh_offset(const glm::dvec3 & offset)
{
	_mesh_offset = offset; 
	_sphere_origin = _sphere_origin_g - offset;
	_capsule_p0 = _capsule_p0_g - offset;
	_capsule_p1 = _capsule_p1_g - offset;
	if (_convex_object) {
		_box_local_transform.setOrigin(_convex_object->getWorldTransform().getOrigin() - btVector3(_mesh_offset.x, _mesh_offset.y, _mesh_offset.z));
	}
	
}

void ot_terrain_contact_common::prepare_sphere_collision(btManifoldResult * result, const glm::dvec3 & center, float radius, float collision_margin)
{
	prepare(result);
	_curr_collider = ctSphere;
	_sphere_radius = radius;
	_collider_collision_margin = collision_margin;
	_sphere_origin_g = center;
	_curr_algo = &ot_terrain_contact_common::collide_sphere_triangle;
}

void ot_terrain_contact_common::prepare_capsule_collision(btManifoldResult * result, const glm::dvec3 & p0, const glm::dvec3 & p1, float radius, float collision_margin)
{
	prepare(result);
	_curr_collider = ctCapsule;
	_capsule_radius = radius;
	_capsule_p0_g = p0;
	_capsule_p1_g = p1;
	_collider_collision_margin = collision_margin;
	_curr_algo = &ot_terrain_contact_common::collide_capsule_triangle;
}

void ot_terrain_contact_common::prepare_bt_convex_collision(btManifoldResult * result, btCollisionObjectWrapper * convex_object)
{
	prepare(result);
	_curr_collider = ctBox;
	_curr_algo = &ot_terrain_contact_common::collide_convex_triangle;
	_box_local_transform = convex_object->getWorldTransform();
	_convex_object = convex_object;
	_bt_ca = 0;
}

void ot_terrain_contact_common::process_triangle_cache()
{
	_triangle_cache.for_each([&](const bt::triangle & t) {
        
        (this->*_curr_algo)(t);
	});

	_triangle_cache.clear();
}

void ot_terrain_contact_common::process_triangle_cache(const coid::dynarray<bt::triangle>& triangle_cache)
{
    btPersistentManifold * p_man = _manifold->getPersistentManifold();

    /// just commented out in case of cashing 

    /* static const uint MAX_CP_COUNT = 4;
    static bool cp_to_del[MAX_CP_COUNT];
    memset(cp_to_del, 0, MAX_CP_COUNT);
    
    for (int8 i = 0; i < p_man->getNumContacts(); i++) {
        if (triangle_cache.find_if([&](const bt::triangle & t) {return t.tri_idx == p_man->getContactPoint(i).m_index1; }) == nullptr) {
            DASSERT(i < MAX_CP_COUNT);
            cp_to_del[i] = true;
        }
    }

    for (int8 i = MAX_CP_COUNT - 1; i >= 0; i--) {
        if (cp_to_del[i]) {
            p_man->removeContactPoint(i);
        }
    }*/
    ///

    triangle_cache.for_each([&](const bt::triangle & t) {
		set_terrain_mesh_offset(*t.parent_offset_p);
		current_processed_triangle = &t;
        
        /*
        this was here for debugging purposes
        bool should_break = false;

        if ((t.a.x == 14.5196447f) && (t.b.x == 14.7275457f) && (t.c.x == 14.4769783f)) {
            float3 e01 = t.b - t.a;
            float3 e12 = t.c - t.b;
            float3 e20 = t.a - t.c;
            e20 = e01;
            should_break = true;
        }

		if (this->_curr_collider == ctCapsule){
			glm::vec3 p0_loc(_capsule_p0_g - _mesh_offset);
			glm::vec3 p1_loc(_capsule_p1_g - _mesh_offset);
			float3 ax_vec = glm::normalize(p1_loc - p0_loc);
			float3 up(glm::normalize(_capsule_p0_g));
            float3 cap_cen = (p0_loc + p1_loc) * 0.5f;
            cap_cen += up*0.1f;


			float3 from = p1_loc + (ax_vec * (_capsule_radius + _collider_collision_margin));
			float3 to = p0_loc - (ax_vec * (_capsule_radius + _collider_collision_margin));
            float u, v, w;
            float d1 = coal::distance_point_trinagle_sqr(from, t.a, t.b, t.c,u,v,w,to);
			float du, dv, dw, dt;
            if (coal::is_valid_barycentric_coord(u, v)) {
                w = 1.f - u - v;
            }
			if (coal::intersects_ray_triangle(to, up, t.a, t.b, t.c, du, dv, dw, dt)){
				float3 cp = t.a * du + t.b * dv + t.c * dw;
				float l = glm::length(cp - from);
				float3 dir = glm::normalize(cp - from);
				du = du;
			}
		}else if (this->_curr_collider == ctSphere) {
            glm::vec3 sph_orig(_sphere_origin_g - _mesh_offset);
            float3 up(glm::normalize(_sphere_origin_g));


            float3 from = sph_orig;
            float du, dv, dw, dt;;
            if (coal::intersects_ray_triangle(from, -up, t.a, t.b, t.c, du, dv, dw, dt)) {
                float3 cp = t.a * du + t.b * dv + t.c * dw;
                float l = glm::length(cp - from);
                float3 dir = glm::normalize(cp - from);
                du = du;
            }
        }
        */
		(this->*_curr_algo)(t);
	});
}

void ot_terrain_contact_common::process_collision_points()
{
    const bool detect_patches = false;

    if (!detect_patches) {
     /*   if (_curr_collider == ctSphere) {
            double3 norm = glm::normalize(this->_sphere_origin_g);
            float elev = ot::world::get()->elevation(this->_sphere_origin_g);
            if (elev <= this->_sphere_radius) {
                btVector3 p(this->_sphere_origin_g.x - norm.x * elev, 
                    this->_sphere_origin_g.y - norm.y * elev, 
                    this->_sphere_origin_g.z - norm.z * elev);
                btVector3 n(norm.x, norm.y, norm.z);
                _manifold->addContactPoint(n, p, elev - _sphere_radius);
            }

            return;
        }*/

        _contact_point_cache.for_each([&](const contact_point & cp) {
            btVector3 p(cp.point.x, cp.point.y, cp.point.z);
            btVector3 n(cp.normal.x, cp.normal.y, cp.normal.z);
            if (_curr_collider == ctSphere) {
                _manifold->addContactPoint(n, p, cp.depth - _sphere_radius);
                btManifoldPoint& mp = _manifold->getPersistentManifold()->getContactPoint(_manifold->getPersistentManifold()->getNumContacts() - 1);
                //mp.m_partId1 = cp.t_idx;
            }
            else if (_curr_collider == ctCapsule) {
                    _manifold->addContactPoint(n, p, cp.depth - _capsule_radius);
            }
        });
    }
    else {
        const float thres_cos = 0.996f;
        const uints cp_count = _contact_point_cache.size();
        
        if (!cp_count) {
            return;
        }

        contact_point * best_cp = &_contact_point_cache[0];

        for (uints i = 1; i < cp_count; i++) {
            contact_point* cur_cp = &_contact_point_cache[i];
            if (glm::dot(best_cp->normal, cur_cp->normal) >= thres_cos ) {
                if (cur_cp->depth < best_cp->depth) {
                    best_cp = cur_cp;
                }
            }
            else {
                btVector3 p(best_cp->point.x, best_cp->point.y, best_cp->point.z);
                btVector3 n(best_cp->normal.x, best_cp->normal.y, best_cp->normal.z);

                if (_curr_collider == ctSphere) {
                    _manifold->addContactPoint(n, p, best_cp->depth - _sphere_radius);
                }
                else if (_curr_collider == ctCapsule) {
                    _manifold->addContactPoint(n, p, best_cp->depth - _capsule_radius);
                }

                best_cp = cur_cp;
            }
        }

        btVector3 p(best_cp->point.x, best_cp->point.y, best_cp->point.z);
        btVector3 n(best_cp->normal.x, best_cp->normal.y, best_cp->normal.z);

        if (_curr_collider == ctSphere) {
            _manifold->addContactPoint(n, p, best_cp->depth - _sphere_radius);
        }
        else if (_curr_collider == ctCapsule) {
            _manifold->addContactPoint(n, p, best_cp->depth - _capsule_radius);
        }
    }
    
}

void ot_terrain_contact_common::process_additional_col_objs()
{
	
}

void ot_terrain_contact_common::collide_sphere_triangle(const bt::triangle & triangle)
{
	static const float small_angle_cos = 0.996f; 
	const float inf_rad = _sphere_radius + _collider_collision_margin + _triangle_collision_margin; // inflated radius
	const float inf_rad_sq = inf_rad * inf_rad + coal::FLOAT_EPS;

	glm::vec3 cp;
    float u, v, w;
	glm::vec3 tn = glm::normalize(glm::cross(triangle.b - triangle.a, triangle.c - triangle.a));
	const float dist_sq = coal::distance_point_trinagle_sqr(_sphere_origin,
		triangle.a, 
		triangle.b, 
		triangle.c,
        u,v,w,
        cp);
    glm::vec3 normal = glm::normalize(_sphere_origin - cp);
	const float cos_theta = glm::dot(normal, tn);

    bool should_generate_contact = cos_theta >= small_angle_cos;
    if (!should_generate_contact) {
        coal::EVoronoiFeature vf = coal::uvw_to_voronoi_feature(u, v, w);
       
    }

	if (dist_sq < inf_rad_sq && should_generate_contact) {
		float penetration_depth = glm::sqrt(dist_sq);
		bool patch_found = false;
		for (uints i = 0; i < _contact_point_cache.size(); i++) {
			if (glm::dot(normal, _contact_point_cache[i].normal) >= small_angle_cos) {
				if (_contact_point_cache[i].depth > penetration_depth) {
					_contact_point_cache[i].point = glm::dvec3(cp) + _mesh_offset;
					_contact_point_cache[i].depth = penetration_depth;
				}

				patch_found = true;
				break;
			}
		}

		if (!patch_found) {
			*_contact_point_cache.add(1) = contact_point(glm::dvec3(cp) + _mesh_offset, normal, penetration_depth,triangle.tri_idx);
		}
	}
}

void ot_terrain_contact_common::collide_capsule_triangle(const bt::triangle & triangle)
{
    const float inf_rad = (_capsule_radius + _collider_collision_margin + _triangle_collision_margin);
    const float inf_rad_sq = inf_rad * inf_rad;
    const float3 ab = triangle.b - triangle.a;
    const float3 ac = triangle.c - triangle.a;
    const float3 tn = glm::normalize(glm::cross(ab,ac));
    float t, u, v, w;
    const float dist_sq = coal::distance_segment_triangle_sqr(_capsule_p0, _capsule_p1, triangle.a, triangle.b, triangle.c, t, u, v, w);

    if (dist_sq >= inf_rad_sq) {
        return;
    }

    coal::EVoronoiFeature vf = coal::uvw_to_voronoi_feature(u,v,w);
    if (coal::is_contact_on_convex_edge(vf, triangle.t_flags)) {
        const float3 cp = triangle.a * u + triangle.b * v + triangle.c * w;
        const float3 sp = _capsule_p0*(1.f - t) + _capsule_p1*t;
        const float3 n = glm::normalize(sp - cp);
        *_contact_point_cache.add() = contact_point(double3(cp) + _mesh_offset,glm::normalize(sp - cp),glm::sqrt(dist_sq),0);
    }
    else {
        const float3 cp = triangle.a * u + triangle.b * v + triangle.c * w;
        *_contact_point_cache.add() = contact_point(double3(cp) + _mesh_offset, tn, glm::sqrt(dist_sq), 0);
    }
}

void ot_terrain_contact_common::collide_convex_triangle(const bt::triangle & triangle)
{
	btTriangleShape tm(btVector3(triangle.a.x + _mesh_offset.x, triangle.a.y + _mesh_offset.y, triangle.a.z + _mesh_offset.z),
        btVector3(triangle.b.x + _mesh_offset.x, triangle.b.y + _mesh_offset.y, triangle.b.z + _mesh_offset.z),
        btVector3(triangle.c.x + _mesh_offset.x, triangle.c.y + _mesh_offset.y, triangle.c.z + _mesh_offset.z));
    tm.setMargin(0.);


    btCollisionObjectWrapper triObWrap(_manifold->getBody1Wrap(), &tm, _manifold->getBody1Wrap()->getCollisionObject(), _manifold->getBody1Wrap()->getWorldTransform(), 0, triangle.tri_idx);//correct transform?

    const btCollisionObjectWrapper * curr_col_obj_wrapper = _manifold->getBody0Wrap();
    _manifold->setBody0Wrap(_convex_object);

    btCollisionAlgorithm* colAlgo = _collision_world->getDispatcher()->findAlgorithm(_manifold->getBody0Wrap(), &triObWrap, _manifold->getPersistentManifold());

    const btCollisionObjectWrapper* tmpWrap = 0;

    tmpWrap = _manifold->getBody1Wrap();
    _manifold->setBody1Wrap(&triObWrap);
    _manifold->setShapeIdentifiersB(0, triangle.tri_idx);


    colAlgo->processCollision(_manifold->getBody0Wrap(), &triObWrap, _collision_world->getDispatchInfo(), _manifold);

    _manifold->setBody1Wrap(curr_col_obj_wrapper);
    _manifold->setBody1Wrap(tmpWrap);
    
    colAlgo->~btCollisionAlgorithm();
    _collision_world->getDispatcher()->freeCollisionAlgorithm(colAlgo);
}

void ot_terrain_contact_common::collide_object_plane(fn_elevation_above_terrain elevation_above_terrain)
{
    DASSERT(_internal_object);
    btVector3 pos_bt = _internal_object->getWorldTransform().getOrigin();
    double3 pos(pos_bt.x(), pos_bt.y(), pos_bt.z());
    double3 norm = glm::normalize(pos);
    double3 hit;
    float3 hit_norm;
    ot::terrain::hitpoint hp;
 
    const double elev(elevation_above_terrain(pos, _sphere_radius, &hit_norm, &hit));

    if (elev == _sphere_radius) {
        return;
    }

    btStaticPlaneShape plane(btVector3(hit_norm.x, hit_norm.y, hit_norm.z).normalized(), glm::dot(glm::normalize(double3(hit_norm)), hit));
    //btStaticPlaneShape plane(btVector3(norm.x,norm.y,norm.z),glm::dot(norm,pos - (norm*elev)));

    btCollisionObjectWrapper planeObWrap(_manifold->getBody1Wrap(), &plane, _manifold->getBody1Wrap()->getCollisionObject(), _manifold->getBody1Wrap()->getWorldTransform(), 0, 0);//correct transform?

	const btCollisionObjectWrapper * curr_col_obj_wrapper = _manifold->getBody0Wrap();
	_manifold->setBody0Wrap(_internal_object);

    btCollisionAlgorithm* colAlgo = _collision_world->getDispatcher()->findAlgorithm(_manifold->getBody0Wrap(), &planeObWrap, _manifold->getPersistentManifold());

    const btCollisionObjectWrapper* tmpWrap = 0;

    tmpWrap = _manifold->getBody1Wrap();
    _manifold->setBody1Wrap(&planeObWrap);
    _manifold->setShapeIdentifiersB(0, 0);


    colAlgo->processCollision(_manifold->getBody0Wrap(), &planeObWrap, _collision_world->getDispatchInfo(), _manifold);

    _manifold->setBody1Wrap(tmpWrap);

//    DASSERT(_manifold->getPersistentManifold()->getNumContacts() <= 1);

    colAlgo->~btCollisionAlgorithm();
    _collision_world->getDispatcher()->freeCollisionAlgorithm(colAlgo);
}

void ot_terrain_contact_common::add_triangle(const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, uint32 ia, uint32 ib, uint32 ic, uint8 flags, const double3 * mesh_offset, uint32 tri_idx)
{
	*_triangle_cache.add(1) = bt::triangle(a, b, c, ia, ib, ic, flags, mesh_offset,tri_idx);
}

void ot_terrain_contact_common::add_additional_col_obj(btCollisionObject * col_obj)
{
	*_additional_col_objs.add(1) = col_obj;
}

void ot_terrain_contact_common::clear_common_data()
{
    _curr_collider = ctCount;
    _curr_algo = 0;
    _mesh_offset = double3(0);
    _hull_center_g = double3(0);
    _hull_polydata = 0;
    _convex_object = 0;
    _internal_object = 0;
    _bt_ca = 0;
    _triangle_cache.clear();
    _contact_point_cache.clear();
    _additional_col_objs.clear();
}

void ot_terrain_contact_common::prepare(btManifoldResult * result)
{
	_manifold = result;
    clear_caches();
	_convex_object = 0;
}

void ot_terrain_contact_common::clear_caches() {
    _triangle_cache.clear();
    _contact_point_cache.clear();
    _additional_col_objs.clear();
}

bool GJK_contact_added(btManifoldPoint & cp, const btCollisionObjectWrapper * colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper * colObj1Wrap, int partId1, int index1)
{    
    DASSERT(current_processed_triangle != nullptr);
    
    const double3 * offset = current_processed_triangle->parent_offset_p;
    const float3 ab = current_processed_triangle->b - current_processed_triangle->a;
    const float3 ac = current_processed_triangle->c - current_processed_triangle->a;
    const float3 ap(cp.m_localPointB[0] - offset->x - current_processed_triangle->a.x, 
        cp.m_localPointB[1] - offset->y - current_processed_triangle->a.y,
        cp.m_localPointB[2] - offset->z - current_processed_triangle->a.z);
    float u, v, w;
    coal::calculate_barycentric_coords(ab,ac,ap,u,v,w);
    coal::EVoronoiFeature vf = coal::uvw_to_voronoi_feature(u,v,w);

    if (!coal::is_contact_on_convex_edge(vf,current_processed_triangle->t_flags)) {
        const float3 n = glm::normalize(glm::cross(ab,ac));
        cp.m_normalWorldOnB = btVector3(n.x,n.y,n.z);
    }
    
    friction_combiner_cbk(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1);

    return true;
}


bool plane_contact_added(btManifoldPoint & cp, const btCollisionObjectWrapper * colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper * colObj1Wrap, int partId1, int index1)
{
    cp.m_combinedFriction = calculate_combined_friction(1.0f, float(colObj0Wrap->getCollisionObject()->getFriction()));
    cp.m_combinedRollingFriction = calculate_combined_rolling_friction(1.0f, float(colObj0Wrap->getCollisionObject()->getRollingFriction()));
    cp.m_combinedRestitution = calculate_combined_restitution(1.0f, float(colObj0Wrap->getCollisionObject()->getRestitution()));
    return true;
}


bool friction_combiner_cbk(btManifoldPoint & cp, const btCollisionObjectWrapper * colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper * colObj1Wrap, int partId1, int index1)
{
    DASSERT(current_processed_triangle != nullptr);
    cp.m_combinedFriction = calculate_combined_friction(current_processed_triangle->fric, float(colObj0Wrap->getCollisionObject()->getFriction()));
    cp.m_combinedRollingFriction = calculate_combined_rolling_friction(current_processed_triangle->roll_fric, float(colObj0Wrap->getCollisionObject()->getRollingFriction()));
    cp.m_combinedRestitution = calculate_combined_restitution(current_processed_triangle->rest, float(colObj0Wrap->getCollisionObject()->getRestitution()));
    return true;
}

float calculate_combined_friction(float b1_fric, float b2_fric)
{
    float friction = b1_fric * b2_fric;

    const float MAX_FRICTION = float(10.);
    if (friction < -MAX_FRICTION)
        friction = -MAX_FRICTION;
    if (friction > MAX_FRICTION)
        friction = MAX_FRICTION;
    return friction;
}

float calculate_combined_restitution(float b1_rest, float b2_rest)
{
    return b1_rest * b2_rest;
}

float calculate_combined_rolling_friction(float b1_roll_fric, float b2_roll_fric)
{
    return calculate_combined_friction(b1_roll_fric,b2_roll_fric);
}
