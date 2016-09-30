#include "ot_terrain_contact_common.h"
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <ot/glm/coal.h>

#ifdef PhysX
	#include <pcm/GuPersistentContactManifold.h>
	#include <common/GuBarycentricCoordinates.h>
#endif

const bt::triangle* current_processed_triangle = nullptr;

bool selectNormal(float u, float v, coid::uint8 data)
{
    if (glm::abs(u) < 0.000001f)
    {
        if (glm::abs(v) < 0.000001f)
        {
            // Vertex 0
            if (!(data & (coal::ceEdge01Convex | coal::ceEdge20Convex)))
                return true;
        }
        else if (glm::abs(v) + 0.000001f > 1.f)
        {
            // Vertex 2
            if (!(data & (coal::ceEdge12Convex | coal::ceEdge20Convex)))
                return true;
        }
        else
        {
            // Edge 0-2
            if (!(data & coal::ceEdge20Convex))
                return true;
        }
    }
    else if (glm::abs(u) + 0.000001f > 1.f)
    {
        if (glm::abs(v) < 0.000001f)
        {
            // Vertex 1
            if (!(data & (coal::ceEdge01Convex | coal::ceEdge12Convex)))
                return true;

        }
    }
    else
    {
        if (glm::abs(v) < 0.000001f)
        {
            // Edge 0-1
            if (!(data & coal::ceEdge01Convex))
                return true;
        }
        else
        {
            const float threshold = 0.9999f;
            const float temp = u + v;
            if (temp >= threshold)
            {
                // Edge 1-2
                if (!(data & coal::ceEdge12Convex))
                    return true;
            }
            else
            {
                // Face
                return true;
            }
        }
    }
    return false;
}

ot_terrain_contact_common::ot_terrain_contact_common(float triangle_collision_margin, btCollisionWorld * world, btCollisionObjectWrapper * planet_body_wrap)
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
#ifdef PhysX
	,_hull_center()
	,mPolyMap(0)
	,_mesh_to_covex()
#endif
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
#ifdef PhysX
	using namespace Ps::aos;
	_hull_center =  V3LoadU(&glm::vec3(_hull_center_g - offset)[0]);
	_convex_transform.p = _hull_center;
	_mesh_to_covex.p = V3LoadU(&glm::vec3(-offset)[0]);
#endif
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
#ifdef PhysX
void ot_terrain_contact_common::prepare_hull_collision(btManifoldResult * result, const glm::quat & hull_rot, const glm::dvec3 & hull_pos, const btConvexPolyhedron & hull_polydata)
{
	prepare(result);
	_curr_collider = ctHull;
	_hull_center_g = hull_pos;
	_hull_polydata = &hull_polydata;
	_curr_algo = &ot_terrain_contact_common::collide_hull_triangle;

	_convex_transform.q = Ps::aos::QuatVLoadXYZW(hull_rot.x, hull_rot.y, hull_rot.z, hull_rot.w);

	_mesh_to_covex = _convex_transform.transformInv(Ps::aos::PsTransformV::createIdentity());

	if (mPolyMap) {
		delete mPolyMap;
	}

	mPolyMap = new physx::Gu::SupportLocalImpl<physx::Gu::BoxV>(_ps_box, _convex_transform, Ps::aos::M33Identity(), Ps::aos::M33Identity());
}
#endif
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
		(this->*_curr_algo)(t);
	});
}

void ot_terrain_contact_common::process_collision_points()
{
	_contact_point_cache.for_each([&](const contact_point & cp) {
		btVector3 p(cp.point.x, cp.point.y, cp.point.z);
		btVector3 n(cp.normal.x, cp.normal.y, cp.normal.z);
		if(_curr_collider == ctSphere){
			_manifold->addContactPoint(n,p, cp.depth - _sphere_radius - _collider_collision_margin - _triangle_collision_margin);
            btManifoldPoint& mp = _manifold->getPersistentManifold()->getContactPoint(_manifold->getPersistentManifold()->getNumContacts() - 1);
            mp.m_partId1 = cp.t_idx;
        }
		else if (_curr_collider == ctCapsule) {
			_manifold->addContactPoint(n, p, cp.depth - _capsule_radius - _collider_collision_margin - _triangle_collision_margin);
		}
	});
}

void ot_terrain_contact_common::process_additional_col_objs()
{
	
}

void ot_terrain_contact_common::collide_sphere_triangle(const bt::triangle & triangle)
{
	const float t = 0.996f;
	float infRad = _sphere_radius + _collider_collision_margin + _triangle_collision_margin;
	float infRadSq = infRad * infRad;
	bool generate_contact = false;
	glm::vec3 contact;
	glm::vec3 tn = glm::normalize(glm::cross(triangle.b - triangle.a, triangle.c - triangle.a));
	float d = coal::distance_point_trinagle_sqr(_sphere_origin,
		triangle.a, 
		triangle.b, 
		triangle.c, 
		triangle.t_flags, 
		contact, 
		generate_contact);
	glm::vec3 normal = glm::normalize(_sphere_origin - contact);
	const float cos_theta = glm::dot(normal, tn);
	if (d <= infRadSq && (generate_contact || cos_theta > t)) {
		float penetration_depth = glm::sqrt(d);
		bool patch_found = false;
		for (uints i = 0; i < _contact_point_cache.size(); i++) {
			if (glm::dot(normal, _contact_point_cache[i].normal) > t) {
				if (_contact_point_cache[i].depth > penetration_depth) {
					_contact_point_cache[i].point = glm::dvec3(contact) + _mesh_offset;
					_contact_point_cache[i].depth = penetration_depth;
				}

				patch_found = true;
				break;
			}
		}

		if (!patch_found) {
			*_contact_point_cache.add(1) = contact_point(glm::dvec3(contact) + _mesh_offset, normal, penetration_depth,triangle.tri_idx);
		}
	}
}

void ot_terrain_contact_common::collide_capsule_triangle(const bt::triangle & triangle)
{
	const float inf_rad = (_capsule_radius + _collider_collision_margin + _triangle_collision_margin);
	const float inf_rad_sq = inf_rad * inf_rad;
	const glm::vec3 ab = triangle.b - triangle.a;
	const glm::vec3 ac = triangle.c - triangle.a;
	const glm::vec3 cen = (_capsule_p0 + _capsule_p1) * .5f;


	const glm::vec3 n = glm::normalize(glm::cross(ab, ac));
	const float d = glm::dot(triangle.a, n);

	const float dist = glm::dot(cen, n) - d;

	if (dist < 0)
		return;

	float t, u, v;
	const float sqDist = coal::distance_segment_triangle_sqr(_capsule_p0, _capsule_p1, triangle.a, triangle.b, triangle.c, t, u, v);

	if (sqDist < inf_rad_sq) {
		glm::vec3 patchNormalInTriangle;
		if (selectNormal(u, v, triangle.t_flags))
		{
			patchNormalInTriangle = n;
		}
		else {
			if (glm::abs(sqDist) < 0.000001f)
			{
				patchNormalInTriangle = n;
			}
			else
			{
				const glm::vec3 pq = _capsule_p1 - _capsule_p0;
				const glm::vec3 pointOnSegment = pq*t + _capsule_p0;
				const float w = 1 - u - v;
				const glm::vec3 pointOnTriangle = triangle.a * w + triangle.b * u + triangle.c * v;
				patchNormalInTriangle = glm::normalize(pointOnSegment - pointOnTriangle);
			}
		}

		const unsigned int previousNumContacts = _contact_point_cache.size();

		generateContacts(triangle.a, triangle.b, triangle.c, n, patchNormalInTriangle, _capsule_p0, _capsule_p1, inf_rad);
		//ML: this need to use the sqInflatedRadius to avoid some bad contacts
		generateEEContacts(triangle.a, triangle.b, triangle.c, patchNormalInTriangle, _capsule_p0, _capsule_p1, inf_rad_sq);

		unsigned int numContacts = _contact_point_cache.size() - previousNumContacts;

		if (numContacts > 0)
		{

			float maxPen = 999999999999999.9f;
			for (unsigned int i = previousNumContacts; i<_contact_point_cache.size(); ++i)
			{
				const float pen = _contact_point_cache[i].depth;
				maxPen = glm::min(maxPen, pen);
			}

			for (unsigned int i = previousNumContacts; i<_contact_point_cache.size(); ++i)
			{
				glm::dvec3 contact0 = _contact_point_cache[i].point;
				for (unsigned int j = i + 1; j<_contact_point_cache.size(); ++j)
				{
					glm::dvec3 contact1 = _contact_point_cache[j].point;
					glm::dvec3 dif = contact1 - contact0;
					double d1 = glm::dot(dif, dif);
					if (0.0000001 > d1)
					{
						_contact_point_cache[j] = _contact_point_cache[_contact_point_cache.size() - 1];
						_contact_point_cache.del(--j);
					}
				}
			}

			if (previousNumContacts > 0)
			{
				for (uints i = 0; i < previousNumContacts; i++) {
					if (glm::dot(_contact_point_cache[i].normal, patchNormalInTriangle)   >  0.996f)
					{
						if (_contact_point_cache[i].depth < _contact_point_cache[previousNumContacts].depth) {
							_contact_point_cache[i].point = _contact_point_cache[previousNumContacts].point;
							_contact_point_cache[i].depth = _contact_point_cache[previousNumContacts].depth;
						}

						_contact_point_cache.del(_contact_point_cache.size() - 1);
						break;
					}
				}
			}
		}
	}
}
#ifdef PhysX

void ot_terrain_contact_common::collide_hull_triangle(const bt::triangle & triangle)
{
	using namespace Ps::aos;


	const Mat33V identity = M33Identity();
	const FloatV zero = FZero();

	const Vec3V v0 = V3LoadU(&triangle.a[0]);
	const Vec3V v1 = V3LoadU(&triangle.b[0]);
	const Vec3V v2 = V3LoadU(&triangle.c[0]);

	const Vec3V v10 = V3Sub(v1, v0);
	const Vec3V v20 = V3Sub(v2, v0);

	const Vec3V n = V3Normalize(V3Cross(v10, v20));
	const FloatV d = V3Dot(v0, n);

	const FloatV dist = FSub(V3Dot(_hull_center, n), d);

	if (FAllGrtr(zero, dist))
		return;


	//tranform verts into the box local space
	const Vec3V locV0 = _mesh_to_covex.transform(v0);
	const Vec3V locV1 = _mesh_to_covex.transform(v1);
	const Vec3V locV2 = _mesh_to_covex.transform(v2);

	physx::Gu::TriangleV localTriangle(locV0, locV1, locV2);

	{

		physx::Gu::SupportLocalImpl<physx::Gu::TriangleV> localTriMap(localTriangle, _convex_transform, identity, identity, true);

		const uint32 previousNumContacts = _contact_point_cache.size();
		uint32 dummy_num_contacts = previousNumContacts;
		Vec3V patchNormal;
		FloatV contact_dist;

		generateTriangleFullContactManifold(localTriangle, 0, &triangle.idx_a, triangle.t_flags, *_hull_polydata, &localTriMap, mPolyMap, _contact_point_cache, dummy_num_contacts, contact_dist, patchNormal);


		uint numContacts = dummy_num_contacts - previousNumContacts;

		if (numContacts > 0)
		{
			bool inActiveEdge0 = (triangle.t_flags & coal::ceEdge01Convex) == 0;
			bool inActiveEdge1 = (triangle.t_flags & coal::ceEdge12Convex) == 0;
			bool inActiveEdge2 = (triangle.t_flags & coal::ceEdge20Convex) == 0;

			if (inActiveEdge0)
				_cached_edges.insert(cached_edge(triangle.idx_a, triangle.idx_b));
			if (inActiveEdge1)
				_cached_edges.insert(cached_edge(triangle.idx_b, triangle.idx_c));
			if (inActiveEdge2)
				_cached_edges.insert(cached_edge(triangle.idx_c, triangle.idx_a));

			addContactsToPatch(patchNormal, previousNumContacts, numContacts);
		}
	}

}
#endif
void ot_terrain_contact_common::collide_convex_triangle(const bt::triangle & triangle)
{
	btTriangleShape tm(btVector3(triangle.a.x + _mesh_offset.x, triangle.a.y + _mesh_offset.y, triangle.a.z + _mesh_offset.z),
        btVector3(triangle.b.x + _mesh_offset.x, triangle.b.y + _mesh_offset.y, triangle.b.z + _mesh_offset.z),
        btVector3(triangle.c.x + _mesh_offset.x, triangle.c.y + _mesh_offset.y, triangle.c.z + _mesh_offset.z));
    tm.setMargin(0.);


    btCollisionObjectWrapper triObWrap(_manifold->getBody1Wrap(), &tm, _manifold->getBody1Wrap()->getCollisionObject(), _manifold->getBody1Wrap()->getWorldTransform(), 0, triangle.tri_idx);//correct transform?
    btCollisionAlgorithm* colAlgo = _collision_world->getDispatcher()->findAlgorithm(_manifold->getBody0Wrap(), &triObWrap, _manifold->getPersistentManifold());

    const btCollisionObjectWrapper* tmpWrap = 0;

    tmpWrap = _manifold->getBody1Wrap();
    _manifold->setBody1Wrap(&triObWrap);
    _manifold->setShapeIdentifiersB(0, triangle.tri_idx);


    colAlgo->processCollision(_manifold->getBody0Wrap(), &triObWrap, _collision_world->getDispatchInfo(), _manifold);

    _manifold->setBody1Wrap(tmpWrap);
    
    colAlgo->~btCollisionAlgorithm();
    _collision_world->getDispatcher()->freeCollisionAlgorithm(colAlgo);
}

void ot_terrain_contact_common::generateContacts(const glm::vec3 & a, const glm::vec3 & b,
	const glm::vec3 & c, const glm::vec3 & planeNormal,
	const glm::vec3 & normal,
	const glm::vec3 & p,
	const glm::vec3 & q,
	float inflatedRadius)
{


	const glm::vec3 ab = b - a;
	const glm::vec3 ac = c - a;
	const glm::vec3 ap = p - a;
	const glm::vec3 aq = q - a;

	//This is used to calculate the barycentric coordinate
	const float d00 = glm::dot(ab, ab);
	const float d01 = glm::dot(ab, ac);
	const float d11 = glm::dot(ac, ac);
	const float bdenom = 1.f / (d00*d11 - d01*d01);

	//compute the intersect point of p and triangle plane abc
	const float inomp = glm::dot(planeNormal, -ap);
	const float ideom = glm::dot(planeNormal, normal);

	const float ipt = inomp / ideom;
	//compute the distance from triangle plane abc
	const float dist3 = glm::dot(ap, planeNormal);

	const glm::vec3 closestP31 = normal*ipt + p;
	const glm::vec3 closestP30 = p;


	//Compute the barycentric coordinate for project point of q
	const glm::vec3 pV20 = closestP31 - a;
	const float pD20 = glm::dot(pV20, ab);
	const float pD21 = glm::dot(pV20, ac);
	const float v0 = (d11*pD20 - d01*pD21) * bdenom;
	const float w0 = (d00*pD21 - d01*pD20) * bdenom;

	if (coal::is_valid_triangle_barycentric_coord(v0, w0) && inflatedRadius > dist3)
	{
		contact_point cp(glm::dvec3(closestP31) + _mesh_offset, normal, -ipt,0);
		*_contact_point_cache.add(1) = cp;
	}

	const float inomq = glm::dot(planeNormal, -aq);

	//compute the distance of q and triangle plane abc
	const float dist4 = glm::dot(aq, planeNormal);

	const float iqt = inomq / ideom;

	const glm::vec3 closestP41 = normal * iqt + q;
	const glm::vec3 closestP40 = q;

	//Compute the barycentric coordinate for project point of q
	const glm::vec3 qV20 = closestP41 - a;
	const float qD20 = glm::dot(qV20, ab);
	const float qD21 = glm::dot(qV20, ac);
	const float v1 = (d11*qD20 - d01*qD21) * bdenom;
	const float w1 = (d00 * qD21 - d01 * qD20) * bdenom;

	if (coal::is_valid_triangle_barycentric_coord(v1, w1) && inflatedRadius > dist4)
	{
		contact_point cp(glm::dvec3(closestP41) + _mesh_offset, normal, -iqt,0);
		*_contact_point_cache.add(1) = cp;
	}
}

void ot_terrain_contact_common::generateEEContacts(const glm::vec3 & a, const glm::vec3 & b,
	const glm::vec3 & c, const glm::vec3 & normal,
	const glm::vec3 & p,
	const glm::vec3 & q,
	float sqInflatedRadius)
{
	generateEE(p, q, sqInflatedRadius, normal, a, b);
	generateEE(p, q, sqInflatedRadius, normal, b, c);
	generateEE(p, q, sqInflatedRadius, normal, a, c);
}

void ot_terrain_contact_common::generateEE(const glm::vec3 & p,
	const glm::vec3 & q,
	float sqInflatedRadius,
	const glm::vec3 & normal,
	const glm::vec3 & a,
	const glm::vec3 & b)
{
	const glm::vec3 ab = b - a;
	const glm::vec3 n = glm::cross(ab, normal);
	const float d = glm::dot(n, a);
	const float np = glm::dot(n, p);
	const float nq = glm::dot(n, q);
	const float signP = np - d;
	const float signQ = nq - d;
	const float  temp = signP *signQ;
	if (temp > 0) return;//If both points in the same side as the plane, no intersect points

						 // if colliding edge (p3,p4) and plane are parallel return no collision
	const glm::vec3 pq = q - p;
	const float npq = glm::dot(n, pq);
	if (glm::abs(npq) < 0.000001f)	return;

	//calculate the intersect point in the segment pq
	const float segTValue = (d - np) / npq;
	const glm::vec3 localPointA = pq * segTValue + p;

	//calculate a normal perpendicular to ray localPointA + normal, 2D segment segment intersection
	const glm::vec3 perNormal = glm::cross(normal, pq);
	const glm::vec3 ap = localPointA - a;
	const float nom = glm::dot(perNormal, ap);
	const float denom = glm::dot(perNormal, ab);

	//const FloatV tValue = FClamp(FDiv(nom, denom), zero, FOne());
	const float tValue = nom / denom;
	if (tValue > 1.f || tValue < .0f)
		return;

	//const Vec3V localPointB = V3ScaleAdd(ab, tValue, a); v = V3Sub(localPointA, localPointB); v =  V3NegScaleSub(ab, tValue, tap)
	const glm::vec3 v = ap - ab*tValue;
	const float sqDist = glm::dot(v, v);

	if (sqInflatedRadius > sqDist)
	{

		const glm::vec3 localPointB = localPointA - v;
		const float signedDist = glm::dot(v, normal);
		contact_point cp(glm::dvec3(localPointB) + _mesh_offset, normal, signedDist,0);
		 *_contact_point_cache.add(1) = cp;
	}
}

#ifdef PhysX

bool ot_terrain_contact_common::generateTriangleFullContactManifold(physx::Gu::TriangleV & localTriangle, const uint32 triangleIndex, const uint32 * triIndices, const uint8 triFlags, const btConvexPolyhedron & polyData, physx::Gu::SupportLocalImpl<physx::Gu::TriangleV>* localTriMap, physx::Gu::SupportLocal * polyMap, coid::dynarray<contact_point>& manifoldContacts, uint32 & numContacts, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V & patchNormal)
{
	using namespace Ps::aos;

	const FloatV threshold = FLoad(0.7071f);//about 45 degree
	PX_UNUSED(threshold);
	{

		physx::Gu::FeatureStatus status = physx::Gu::POLYDATA0;
		FloatV minOverlap = FMax();
		//minNormal will be in the local space of polyData
		Vec3V minNormal = V3Zero();


		uint32 feature0;
		if (!testTriangleFaceNormal(localTriangle, polyData, localTriMap, polyMap, contactDist, minOverlap, feature0, minNormal, physx::Gu::POLYDATA0, status))
			return false;

		uint32 feature1;
		if (!testPolyFaceNormal(localTriangle, polyData, localTriMap, polyMap, contactDist, minOverlap, feature1, minNormal, physx::Gu::POLYDATA1, status))
			return false;


		//const btFace& polygon1 = polyData.m_faces[feature1];
		if (!testPolyEdgeNormal(localTriangle, triFlags, polyData, polyData, localTriMap, polyMap, contactDist, minOverlap, minNormal, physx::Gu::EDGE0, status))
			return false;

		const Vec3V triNormal = localTriangle.normal();

		if (status == physx::Gu::POLYDATA0)
		{
			//minNormal is the triangle normal and it is in the local space of polydata0
			const btFace& referencePolygon = polyData.m_faces[getPolygonIndex(polyData, polyMap, minNormal)];

			patchNormal = triNormal;
			generatedTriangleContacts(localTriangle, triangleIndex, triFlags, polyData, referencePolygon, polyMap, &manifoldContacts, numContacts, contactDist, triNormal);

		}
		else
		{

			if (status == physx::Gu::POLYDATA1)
			{
				const btFace& referencePolygon = polyData.m_faces[feature1];

				const Vec3V contactNormal = V3Normalize(M33TrnspsMulV3(polyMap->shape2Vertex, V3LoadU(&glm::vec3(referencePolygon.m_plane[0], referencePolygon.m_plane[1], referencePolygon.m_plane[2])[0])));
				const Vec3V nContactNormal = V3Neg(contactNormal);
				const FloatV cosTheta = V3Dot(nContactNormal, triNormal);

				if (FAllGrtr(cosTheta, threshold))
				{
					patchNormal = triNormal;
					generatedTriangleContacts(localTriangle, triangleIndex, triFlags, polyData, referencePolygon, polyMap, &manifoldContacts, numContacts, contactDist, triNormal);
				}
				else
				{
					//ML : defer the contacts generation
					DeferredPolyData* data = _deferred_data.add(1);
					data->mTriangleIndex = triangleIndex;
					data->mFeatureIndex = feature1;
					data->triFlags = triFlags;
					data->mInds[0] = triIndices[0];
					data->mInds[1] = triIndices[1];
					data->mInds[2] = triIndices[2];
					V3StoreU(localTriangle.verts[0], data->mVerts[0]);
					V3StoreU(localTriangle.verts[1], data->mVerts[1]);
					V3StoreU(localTriangle.verts[2], data->mVerts[2]);
					return true;

				}
			}
			else
			{
				feature1 = (uint32)getPolygonIndex(polyData, polyMap, minNormal);
				const btFace * referencePolygon = &polyData.m_faces[feature1];

				const Vec3V contactNormal = V3Normalize(M33TrnspsMulV3(polyMap->shape2Vertex, V3LoadU(&glm::vec3(referencePolygon->m_plane[0], referencePolygon->m_plane[1], referencePolygon->m_plane[2])[0])));
				const Vec3V nContactNormal = V3Neg(contactNormal);

				//if the minimum sperating axis is edge case, we don't defer it because it is an activeEdge
				patchNormal = nContactNormal;
				generatedPolyContacts(polyData, *referencePolygon, localTriangle, triangleIndex, triFlags, polyMap, &manifoldContacts, numContacts, contactDist, contactNormal);
			}
		}

	}
	
	return true;
}

bool ot_terrain_contact_common::testTriangleFaceNormal(const physx::Gu::TriangleV & triangle, const btConvexPolyhedron & polyData, physx::Gu::SupportLocalImpl<physx::Gu::TriangleV>* triMap, physx::Gu::SupportLocal * polyMap, const Ps::aos::FloatVArg contactDist, Ps::aos::FloatV & minOverlap, uint32 & feature, Ps::aos::Vec3V & faceNormal, const physx::Gu::FeatureStatus faceStatus, physx::Gu::FeatureStatus & status)
{
	PX_UNUSED(triMap);
	PX_UNUSED(polyData);

	using namespace Ps::aos;


	FloatV min1, max1;
	const FloatV eps = FEps();
	const BoolV bTrue = BTTTT();

	const Vec3V triangleLocNormal = triangle.normal();

	const FloatV min0 = V3Dot(triangleLocNormal, triangle.verts[0]);
	const FloatV max0 = min0;

	polyMap->doSupport(triangleLocNormal, min1, max1);

	const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

	if (BAllEq(con, bTrue))
		return false;

	minOverlap = FSub(FSub(max0, min1), eps);
	status = faceStatus;
	feature = 0;
	faceNormal = triangleLocNormal;

	return true;
}

bool ot_terrain_contact_common::testPolyFaceNormal(const physx::Gu::TriangleV & triangle, const btConvexPolyhedron & polyData, physx::Gu::SupportLocalImpl<physx::Gu::TriangleV>* triMap, physx::Gu::SupportLocal * polyMap, const Ps::aos::FloatVArg contactDist, Ps::aos::FloatV & minOverlap, uint32 & feature, Ps::aos::Vec3V & faceNormal, const physx::Gu::FeatureStatus faceStatus, physx::Gu::FeatureStatus & status)
{
	PX_UNUSED(triangle);

	using namespace Ps::aos;
	FloatV _minOverlap = FMax();
	uint32  _feature = 0;
	Vec3V  _faceNormal = faceNormal;
	FloatV min0, max0;
	FloatV min1, max1;
	const BoolV bTrue = BTTTT();
	const FloatV eps = FEps();

/*	if (polyMap->isIdentityScale)
	{*/
		//in the local space of polyData0
		for (uint32 i = 0; i<polyData.m_faces.size(); ++i)
		{
			const btFace& polygon = polyData.m_faces[i];

			const glm::vec3 minVertGlm(polyData.m_vertices[0].x(), polyData.m_vertices[0].y(), polyData.m_vertices[0].z());

			const Vec3V minVert = V3LoadU(&minVertGlm[0]); // toto mozno bude zle
			const FloatV planeDist = FLoad(polygon.m_plane[3]); // aj toto mozno bude zle
			//shapeSpace and vertexSpace are the same
			const Vec3V planeNormal = V3LoadU(&glm::vec3(polygon.m_plane[0], polygon.m_plane[1], polygon.m_plane[2])[0]);

			//ML::avoid lHS, don't use the exiting function
			min0 = V3Dot(planeNormal, minVert);
			max0 = FNeg(planeDist);

			triMap->doSupportFast(planeNormal, min1, max1);

			const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

			if (BAllEq(con, bTrue))
				return false;

			const FloatV tempOverlap = FSub(max0, min1);

			if (FAllGrtr(_minOverlap, tempOverlap))
			{
				_minOverlap = tempOverlap;
				_feature = i;
				_faceNormal = planeNormal;
			}
		}
	/*}
	else
	{

		//in the local space of polyData0
		for (uint i = 0; i<polyData.m_faces.size(); ++i)
		{
			const Gu::HullPolygonData& polygon = polyData.mPolygons[i];

			const Vec3V minVert = V3LoadU(polyData.mVerts[polygon.mMinIndex]);
			const FloatV planeDist = FLoad(polygon.mPlane.d);
			const Vec3V vertexSpacePlaneNormal = V3LoadU(polygon.mPlane.n);
			//transform plane n to shape space
			const Vec3V shapeSpacePlaneNormal = M33TrnspsMulV3(polyMap->shape2Vertex, vertexSpacePlaneNormal);

			const FloatV magnitude = FRsqrtFast(V3LengthSq(shapeSpacePlaneNormal)); //FRecip(V3Length(shapeSpacePlaneNormal));

																					//ML::avoid lHS, don't use the exiting function
			min0 = FMul(V3Dot(vertexSpacePlaneNormal, minVert), magnitude);
			max0 = FMul(FNeg(planeDist), magnitude);

			//normalize the shapeSpacePlaneNormal
			const Vec3V planeN = V3Scale(shapeSpacePlaneNormal, magnitude);

			triMap->doSupportFast(planeN, min1, max1);

			const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

			if (BAllEq(con, bTrue))
				return false;

			const FloatV tempOverlap = FSub(max0, min1);

			if (FAllGrtr(_minOverlap, tempOverlap))
			{
				_minOverlap = tempOverlap;
				_feature = i;
				_faceNormal = planeN;
			}
		}
	}
	*/
	if (FAllGrtr(minOverlap, FAdd(_minOverlap, eps)))
	{
		faceNormal = _faceNormal;
		minOverlap = _minOverlap;
		status = faceStatus;
	}

	feature = _feature;

	return true;
}

bool ot_terrain_contact_common::testPolyEdgeNormal(const physx::Gu::TriangleV & triangle, const uint8 triFlags, const btConvexPolyhedron &, const btConvexPolyhedron & polyData, physx::Gu::SupportLocalImpl<physx::Gu::TriangleV>* triMap, physx::Gu::SupportLocal * polyMap, const Ps::aos::FloatVArg contactDist, Ps::aos::FloatV & minOverlap, Ps::aos::Vec3V & minNormal, const physx::Gu::FeatureStatus edgeStatus, physx::Gu::FeatureStatus & status)
{
	using namespace Ps::aos;
	FloatV overlap = minOverlap;
	FloatV min0, max0;
	FloatV min1, max1;
	const BoolV bTrue = BTTTT();
	const FloatV zero = FZero();
	const Vec3V eps2 = V3Splat(FLoad(1e-6));

	const Vec3V v0 = M33MulV3(polyMap->shape2Vertex, triangle.verts[0]);
	const Vec3V v1 = M33MulV3(polyMap->shape2Vertex, triangle.verts[1]);
	const Vec3V v2 = M33MulV3(polyMap->shape2Vertex, triangle.verts[2]);

	physx::Gu::TriangleV vertexSpaceTriangle(v0, v1, v2);

	uint32 nbTriangleAxes = 0;
	Vec3V triangleAxes[3];
	for (int8 kStart = 0, kEnd = 2; kStart<3; kEnd = kStart++)
	{
		bool active = (triFlags & (1 << (kEnd + 3))) != 0; // toto skontrolovat

		if (active)
		{
			const Vec3V p00 = vertexSpaceTriangle.verts[kStart];
			const Vec3V p01 = vertexSpaceTriangle.verts[kEnd];
			//change to shape space
			triangleAxes[nbTriangleAxes++] = V3Sub(p01, p00);
		}
	}

	if (nbTriangleAxes == 0)
		return true;


	//create localTriPlane in the vertex space
	const Vec3V vertexSpaceTriangleNormal = vertexSpaceTriangle.normal();
	const FloatV vertexSpaceTriangleD = FNeg(V3Dot(vertexSpaceTriangleNormal, triangle.verts[0]));

	for (uint32 i = 0; i<polyData.m_faces.size(); ++i)
	{
		const btFace& polygon = polyData.m_faces[i];
		//const PxU8* inds = &polygon.m_indices.;
		const Vec3V vertexSpacePlaneNormal = V3LoadU(&glm::vec3(polygon.m_plane[0], polygon.m_plane[1], polygon.m_plane[2])[0]);

		//fast culling. 
		if (FAllGrtr(V3Dot(vertexSpacePlaneNormal, vertexSpaceTriangleNormal), zero))
			continue;

		// Loop through polygon vertices == polygon edges;
		for (uint32 lStart = 0, lEnd = uint32(polygon.m_indices.size() - 1); lStart<polygon.m_indices.size(); lEnd = uint32(lStart++))
		{
			//in the vertex space
			const Vec3V p10 = V3LoadU(&glm::vec3(polyData.m_vertices[polygon.m_indices[lStart]].x(), polyData.m_vertices[polygon.m_indices[lStart]].y(), polyData.m_vertices[polygon.m_indices[lStart]].z())[0]); // toto potom prepisat!!!!
			const Vec3V p11 = V3LoadU(&glm::vec3(polyData.m_vertices[polygon.m_indices[lEnd]].x(), polyData.m_vertices[polygon.m_indices[lEnd]].y(), polyData.m_vertices[polygon.m_indices[lEnd]].z())[0]);
			const FloatV signDistP10 = FAdd(V3Dot(p10, vertexSpaceTriangleNormal), vertexSpaceTriangleD);
			const FloatV signDistP11 = FAdd(V3Dot(p11, vertexSpaceTriangleNormal), vertexSpaceTriangleD);

			const BoolV con0 = BOr(FIsGrtrOrEq(contactDist, signDistP10), FIsGrtrOrEq(contactDist, signDistP11));
			if (BAllEq(con0, bTrue))
			{
				const Vec3V dir = V3Sub(p11, p10);

				for (uint32 k = 0; k < nbTriangleAxes; ++k)
				{
					const Vec3V currentPolyEdge = triangleAxes[k];
					const Vec3V v = V3Cross(dir, currentPolyEdge);
					const Vec3V absV = V3Abs(v);

					if (!V3AllGrtr(eps2, absV))
					{
						//transform the v back to the space space
						const Vec3V shapeSpaceV = M33TrnspsMulV3(polyMap->shape2Vertex, v);
						const Vec3V n0 = V3Normalize(shapeSpaceV);
						triMap->doSupportFast(n0, min0, max0);
						polyMap->doSupport(n0, min1, max1);
						const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));
						if (BAllEq(con, bTrue))
							return false;

						const FloatV tempOverlap = FSub(max0, min1);

						if (FAllGrtr(overlap, tempOverlap))
						{
							overlap = tempOverlap;
							minNormal = n0;
							status = edgeStatus;
						}

					}
				}
			}
		}
	}
	minOverlap = overlap;

	return true;
}

int32 ot_terrain_contact_common::getPolygonIndex(const btConvexPolyhedron & polyData, physx::Gu::SupportLocal * map, const Ps::aos::Vec3VArg normal)
{
	using namespace Ps::aos;

	//normal is in shape space, need to transform the vertex space
	const Vec3V n = M33TrnspsMulV3(map->vertex2Shape, normal);
	const Vec3V nnormal = V3Neg(n);
	const Vec3V planeN = Vec3V_From_Vec4V(V4LoadU(&glm::vec3(polyData.m_faces[0].m_plane[0], polyData.m_faces[0].m_plane[2], polyData.m_faces[0].m_plane[3])[0]));
	FloatV minProj = V3Dot(n, planeN);

	const FloatV zero = FZero();
	const BoolV bTrue = BTTTT();
	int32 closestFaceIndex = 0;

	for (uint32 i = 1; i< polyData.m_faces.size(); ++i)
	{
		Vec3V planeNi = V3LoadU(&glm::vec3(polyData.m_faces[i].m_plane[0], polyData.m_faces[i].m_plane[1], polyData.m_faces[i].m_plane[2])[0]);
		const FloatV proj = V3Dot(n, planeNi);
		if (FAllGrtr(minProj, proj))
		{
			minProj = proj;
			closestFaceIndex = (int32)i;
		}
	}

	return closestFaceIndex;
}

void ot_terrain_contact_common::generatedTriangleContacts(const physx::Gu::TriangleV & triangle, const uint32 triangleIndex, const uint8 _triFlags, const btConvexPolyhedron & polyData1, const btFace & incidentPolygon, physx::Gu::SupportLocal * map1, coid::dynarray<contact_point>* manifoldContacts, uint32 & numContacts, const Ps::aos::FloatVArg contactDist, const Ps::aos::Vec3VArg contactNormal)
{
	using namespace Ps::aos;

	uint8 triFlags = _triFlags;
	const uint32 previousContacts = numContacts;

	const FloatV zero = FZero();
	const BoolV bTrue = BTTTT();

	const Mat33V rot = physx::Gu::findRotationMatrixFromZAxis(contactNormal);
	//incidentPolygon.m_indices
//	const PxU8* inds1 = polyData1.mPolygonVertexRefs + incidentPolygon.mVRef8;

	Vec3V points0In0[3];
	coid::dynarray<Vec3V> points1In0(incidentPolygon.m_indices.size());
	coid::dynarray<FloatV> points1In0TValue(incidentPolygon.m_indices.size());
	coid::dynarray<bool> points1In0Penetration(incidentPolygon.m_indices.size());;


	points0In0[0] = triangle.verts[0];
	points0In0[1] = triangle.verts[1];
	points0In0[2] = triangle.verts[2];



	//Transform all the verts from vertex space to shape space
	//map1->populateVerts(inds1, incidentPolygon.mNbVerts, polyData1.mVerts, points1In0); to dole je nahrada za toto
	for (int i = 0; i < incidentPolygon.m_indices.size(); i++) {
		btVector3 v = polyData1.m_vertices[incidentPolygon.m_indices[i]];
		*points1In0.add(1) = V3LoadU(&glm::vec3(v[0], v[1], v[2])[0]);
	
	}
	
	PX_ASSERT(incidentPolygon.mNbVerts <= 64);

	Vec3V eps = Vec3V_From_FloatV(FEps());
	Vec3V max = Vec3V_From_FloatV(FMax());
	Vec3V nmax = V3Neg(max);

	//transform reference polygon to 2d, calculate min and max
	Vec3V rPolygonMin = max;
	Vec3V rPolygonMax = nmax;
	for (uint32 i = 0; i<3; ++i)
	{
		points0In0[i] = M33MulV3(rot, points0In0[i]);
		rPolygonMin = V3Min(rPolygonMin, points0In0[i]);
		rPolygonMax = V3Max(rPolygonMax, points0In0[i]);
	}


	rPolygonMin = V3Sub(rPolygonMin, eps);
	rPolygonMax = V3Add(rPolygonMax, eps);

	const FloatV d = V3GetZ(points0In0[0]);
	const FloatV rd = FAdd(d, contactDist);

	Vec3V iPolygonMin = max;
	Vec3V iPolygonMax = nmax;


	uint32 inside = 0;
	for (uint32 i = 0; i<incidentPolygon.m_indices.size(); ++i)
	{
		const Vec3V vert1 = points1In0[i]; //this still in polyData1's local space
		points1In0[i] = M33MulV3(rot, vert1);
		const FloatV z = V3GetZ(points1In0[i]);
		points1In0TValue[i] = FSub(z, d);
		points1In0[i] = V3SetZ(points1In0[i], d);
		iPolygonMin = V3Min(iPolygonMin, points1In0[i]);
		iPolygonMax = V3Max(iPolygonMax, points1In0[i]);
		if (FAllGrtr(rd, z))
		{
			points1In0Penetration[i] = true;

			if (physx::Gu::contains(points0In0, 3, points1In0[i], rPolygonMin, rPolygonMax))
			{
				inside++;


				const FloatV t = V3Dot(contactNormal, V3Sub(triangle.verts[0], vert1));
				const Vec3V projectPoint = V3ScaleAdd(contactNormal, t, vert1);
				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), FNeg(t));
				float tmp[4];
				contact_point cp;
				_mm_store_ps(tmp, t);
				cp.depth = -tmp[0];
				_mm_store_ps(tmp, contactNormal);
				cp.normal = glm::vec3(tmp[0], tmp[1], tmp[2]);
				_mm_store_ps(tmp, projectPoint);
				cp.point = glm::vec3(tmp[0], tmp[1], tmp[2]);

				*_contact_point_cache.add(1) = cp;

				PX_ASSERT(numContacts <= 64);
			}
		}

	}



	if (inside == incidentPolygon.m_indices.size())
	{
		return;
	}

	inside = 0;
	iPolygonMin = V3Sub(iPolygonMin, eps);
	iPolygonMax = V3Add(iPolygonMax, eps);

	const Vec3V incidentNormal = V3Normalize(M33TrnspsMulV3(map1->shape2Vertex, V3LoadU(&glm::vec3(incidentPolygon.m_plane[0], incidentPolygon.m_plane[1], incidentPolygon.m_plane[2])[0])));
	const FloatV iPlaneD = V3Dot(incidentNormal, M33MulV3(map1->vertex2Shape, V3LoadU(&glm::vec3(polyData1.m_vertices[incidentPolygon.m_indices[0]][0], polyData1.m_vertices[incidentPolygon.m_indices[0]][1], polyData1.m_vertices[incidentPolygon.m_indices[0]][2])[0])));

	for (uint32 i = 0; i<3; ++i)
	{
		if (physx::Gu::contains(&points1In0[0], incidentPolygon.m_indices.size(), points0In0[i], iPolygonMin, iPolygonMax))
		{
			const Vec3V vert0 = M33TrnspsMulV3(rot, points0In0[i]);
			const FloatV t = FSub(V3Dot(incidentNormal, vert0), iPlaneD);

			if (FAllGrtr(t, contactDist))
				continue;

			const Vec3V projPoint = V3NegScaleSub(incidentNormal, t, vert0);

			const Vec3V v = V3Sub(projPoint, vert0);
			const FloatV t3 = V3Dot(v, contactNormal);

			const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), t3);
			float tmp[4];
			contact_point cp;
			_mm_store_ps(tmp, t3);
			cp.depth = tmp[0];
			_mm_store_ps(tmp, contactNormal);
			cp.normal = glm::vec3(tmp[0], tmp[1], tmp[2]);
			_mm_store_ps(tmp, vert0);
			cp.point = glm::vec3(tmp[0], tmp[1], tmp[2]);
			*_contact_point_cache.add(1) = cp;
		}

	}


	if (inside == 3)
		return;


	for (uint32 a = 0; a < 2; ++a)
	{

#ifdef	__SPU__
		bool allEdgesNotActives = false;
#else
		bool allEdgesNotActives = ((triFlags >> 3) == 0);
#endif
		if (!allEdgesNotActives)
		{
			//(2) segment intesection
			for (uint32 rStart = 0, rEnd = 2; rStart < 3; rEnd = rStart++)
			{
#ifdef	__SPU__
				bool active = true;
#else
				bool active = (triFlags & (1 << (rEnd + 3))) != 0;
#endif
				if (!active)
					continue;


				const Vec3V rpA = points0In0[rStart];
				const Vec3V rpB = points0In0[rEnd];

				const Vec3V rMin = V3Min(rpA, rpB);
				const Vec3V rMax = V3Max(rpA, rpB);

				for (uint32 iStart = 0, iEnd = uint32(incidentPolygon.m_indices.size() - 1); iStart < incidentPolygon.m_indices.size(); iEnd = iStart++)
				{
					if ((!points1In0Penetration[iStart] && !points1In0Penetration[iEnd]))//|| (points1In0[i].status == POINT_OUTSIDE && points1In0[incidentIndex].status == POINT_OUTSIDE))
						continue;

					const Vec3V ipA = points1In0[iStart];
					const Vec3V ipB = points1In0[iEnd];

					const Vec3V iMin = V3Min(ipA, ipB);
					const Vec3V iMax = V3Max(ipA, ipB);

					const BoolV tempCon = BOr(V3IsGrtr(iMin, rMax), V3IsGrtr(rMin, iMax));
					const BoolV con = BOr(BGetX(tempCon), BGetY(tempCon));

					if (BAllEq(con, bTrue))
						continue;

					FloatV a1 = physx::Gu::signed2DTriArea(rpA, rpB, ipA);
					FloatV a2 = physx::Gu::signed2DTriArea(rpA, rpB, ipB);

					if (FAllGrtr(zero, FMul(a1, a2)))
					{
						FloatV a3 = physx::Gu::signed2DTriArea(ipA, ipB, rpA);
						FloatV a4 = physx::Gu::signed2DTriArea(ipA, ipB, rpB);

						if (FAllGrtr(zero, FMul(a3, a4)))
						{

							//these two segment intersect in 2d
							const FloatV t = FMul(a1, FRecip(FSub(a2, a1)));

							const Vec3V ipAOri = V3SetZ(points1In0[iStart], FAdd(points1In0TValue[iStart], d));
							const Vec3V ipBOri = V3SetZ(points1In0[iEnd], FAdd(points1In0TValue[iEnd], d));

							const Vec3V pBB = V3NegScaleSub(V3Sub(ipBOri, ipAOri), t, ipAOri);
							const Vec3V pAA = V3SetZ(pBB, d);
							const Vec3V pA = M33TrnspsMulV3(rot, pAA);
							const Vec3V pB = M33TrnspsMulV3(rot, pBB);
							const FloatV pen = FSub(V3GetZ(pBB), V3GetZ(pAA));

							if (FAllGrtr(pen, contactDist))
								continue;

							const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), pen);

							float tmp[4];
							contact_point cp;
							_mm_store_ps(tmp, pen);
							cp.depth = tmp[0];
							_mm_store_ps(tmp, contactNormal);
							cp.normal = glm::vec3(tmp[0], tmp[1], tmp[2]);
							_mm_store_ps(tmp, pA);
							cp.point = glm::vec3(tmp[0], tmp[1], tmp[2]);

							*_contact_point_cache.add(1) = cp;

							PX_ASSERT(numContacts <= 64);
						}
					}
				}

			}
		}

		if (previousContacts != numContacts)
		{
			break;
		}
		//invert bits in the active edge flags - inactive edges become active and actives become inactive. Re-run, generating edge contacts with inactive edges
		triFlags = uint8(triFlags ^ 0xff);
	}
}

void ot_terrain_contact_common::generatedPolyContacts(const btConvexPolyhedron & polyData0, const btFace & referencePolygon, const physx::Gu::TriangleV & triangle, const uint32 triangleIndex, const uint8 triFlags, physx::Gu::SupportLocal * map0, coid::dynarray<contact_point> * manifoldContacts, uint32 & numContacts, const Ps::aos::FloatVArg contactDist, const Ps::aos::Vec3VArg contactNormal)
{
	PX_UNUSED(triFlags);

	using namespace Ps::aos;

	const FloatV zero = FZero();
	const BoolV bTrue = BTTTT();


	//const PxU8* inds0 = polyData0.mPolygonVertexRefs + referencePolygon.mVRef8;

	const Vec3V nContactNormal = V3Neg(contactNormal);

	//this is the matrix transform all points to the 2d plane
	const Mat33V rot = physx::Gu::findRotationMatrixFromZAxis(contactNormal);

	coid::dynarray<Vec3V> points0In0(referencePolygon.m_indices.size()); 
	Vec3V points1In0[3];
	FloatV points1In0TValue[3];

	bool points1In0Penetration[3];

	//Transform all the verts from vertex space to shape space
	//map0->populateVerts(inds0, referencePolygon.mNbVerts, polyData0.mVerts, points0In0);
	for (int i = 0; i < referencePolygon.m_indices.size(); i++) {
		btVector3 v = polyData0.m_vertices[referencePolygon.m_indices[i]];
		*points0In0.add(1) = V3LoadU(&glm::vec3(v[0], v[1], v[2])[0]);
	}

	points1In0[0] = triangle.verts[0];
	points1In0[1] = triangle.verts[1];
	points1In0[2] = triangle.verts[2];

	//the first point in the reference plane
	const Vec3V referencePoint = points0In0[0];

	Vec3V eps = Vec3V_From_FloatV(FEps());
	Vec3V max = Vec3V_From_FloatV(FMax());
	Vec3V nmax = V3Neg(max);

	//transform reference polygon to 2d, calculate min and max
	Vec3V rPolygonMin = max;
	Vec3V rPolygonMax = nmax;
	for (uint32 i = 0; i<referencePolygon.m_indices.size(); ++i)
	{
		//points0In0[i].vertext = M33TrnspsMulV3(rot, Vec3V_From_PxVec3(polyData0.mVerts[inds0[i]]));
		points0In0[i] = M33MulV3(rot, points0In0[i]);
		rPolygonMin = V3Min(rPolygonMin, points0In0[i]);
		rPolygonMax = V3Max(rPolygonMax, points0In0[i]);
	}

	rPolygonMin = V3Sub(rPolygonMin, eps);
	rPolygonMax = V3Add(rPolygonMax, eps);



	const FloatV d = V3GetZ(points0In0[0]);

	const FloatV rd = FAdd(d, contactDist);

	Vec3V iPolygonMin = max;
	Vec3V iPolygonMax = nmax;

	uint32 inside = 0;
	for (uint32 i = 0; i<3; ++i)
	{
		const Vec3V vert1 = points1In0[i]; //this still in polyData1's local space
		points1In0[i] = M33MulV3(rot, vert1);
		const FloatV z = V3GetZ(points1In0[i]);
		points1In0TValue[i] = FSub(z, d);
		points1In0[i] = V3SetZ(points1In0[i], d);
		iPolygonMin = V3Min(iPolygonMin, points1In0[i]);
		iPolygonMax = V3Max(iPolygonMax, points1In0[i]);
		if (FAllGrtr(rd, z))
		{
			points1In0Penetration[i] = true;

			//ML : check to see whether all the points of triangles in 2D space are within reference polygon's range
			if (physx::Gu::contains(&points0In0[0], referencePolygon.m_indices.size(), points1In0[i], rPolygonMin, rPolygonMax))
			{
				inside++;

				//calculate projection point
				const FloatV t = V3Dot(contactNormal, V3Sub(vert1, referencePoint));
				const Vec3V projectPoint = V3NegScaleSub(contactNormal, t, vert1);

				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(nContactNormal), t);

				float tmp[4];
				contact_point cp;
				_mm_store_ps(tmp, t);
				cp.depth = tmp[0];
				_mm_store_ps(tmp, nContactNormal);
				cp.normal = glm::vec3(tmp[0], tmp[1], tmp[2]);
				_mm_store_ps(tmp, vert1);
				cp.point = glm::vec3(tmp[0], tmp[1], tmp[2]);

				*_contact_point_cache.add(1) = cp;

				PX_ASSERT(numContacts <= 64);
			}
		}

	}


	if (inside == 3)
	{
		return;
	}

	inside = 0;
	iPolygonMin = V3Sub(iPolygonMin, eps);
	iPolygonMax = V3Add(iPolygonMax, eps);

	const Vec3V incidentNormal = triangle.normal();
	const FloatV iPlaneD = V3Dot(incidentNormal, triangle.verts[0]);
	const FloatV one = FOne();
	for (uint32 i = 0; i<referencePolygon.m_indices.size(); ++i)
	{
		if (physx::Gu::contains(points1In0, 3, points0In0[i], iPolygonMin, iPolygonMax))
		{
			const Vec3V vert0 = M33TrnspsMulV3(rot, points0In0[i]);

			const FloatV t = FSub(V3Dot(incidentNormal, vert0), iPlaneD);

			if (FAllGrtr(t, contactDist))
				continue;

			const Vec3V projPoint = V3NegScaleSub(incidentNormal, t, vert0);

			FloatV u, w;
			physx::Gu::barycentricCoordinates(projPoint, triangle.verts[0], triangle.verts[1], triangle.verts[2], u, w);
			const BoolV con = BAnd(FIsGrtrOrEq(u, zero), BAnd(FIsGrtrOrEq(w, zero), FIsGrtrOrEq(one, FAdd(u, w))));

			if (BAllEq(con, bTrue))
			{
				const Vec3V v = V3Sub(projPoint, vert0);
				const FloatV t3 = V3Dot(v, contactNormal);

				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(nContactNormal), t3);

				float tmp[4];
				contact_point cp;
				_mm_store_ps(tmp, t3);
				cp.depth = tmp[0];
				_mm_store_ps(tmp, nContactNormal);
				cp.normal = glm::vec3(tmp[0], tmp[1], tmp[2]);
				_mm_store_ps(tmp, projPoint);
				cp.point = glm::vec3(tmp[0], tmp[1], tmp[2]);

				*_contact_point_cache.add(1) = cp;

				PX_ASSERT(numContacts <= 64);
			}

		}

	}

	if (inside == referencePolygon.m_indices.size())
		return;



	//Always generate segment contacts
	//(2) segment intesection
	for (uint32 iStart = 0, iEnd = 2; iStart < 3; iEnd = iStart++)
	{
		if ((!points1In0Penetration[iStart] && !points1In0Penetration[iEnd]))
			continue;

		const Vec3V ipA = points1In0[iStart];
		const Vec3V ipB = points1In0[iEnd];

		const Vec3V iMin = V3Min(ipA, ipB);
		const Vec3V iMax = V3Max(ipA, ipB);

		for (uint32 rStart = 0, rEnd = uint32(referencePolygon.m_indices.size() - 1); rStart < referencePolygon.m_indices.size(); rEnd = rStart++)
		{

		const Vec3V rpA = points0In0[rStart];
			const Vec3V rpB = points0In0[rEnd];

			const Vec3V rMin = V3Min(rpA, rpB);
			const Vec3V rMax = V3Max(rpA, rpB);

			const BoolV tempCon = BOr(V3IsGrtr(iMin, rMax), V3IsGrtr(rMin, iMax));
			const BoolV con = BOr(BGetX(tempCon), BGetY(tempCon));

			if (BAllEq(con, bTrue))
				continue;


			FloatV a1 = physx::Gu::signed2DTriArea(rpA, rpB, ipA);
			FloatV a2 = physx::Gu::signed2DTriArea(rpA, rpB, ipB);


			if (FAllGrtr(zero, FMul(a1, a2)))
			{
				FloatV a3 = physx::Gu::signed2DTriArea(ipA, ipB, rpA);
				FloatV a4 = physx::Gu::signed2DTriArea(ipA, ipB, rpB);

				if (FAllGrtr(zero, FMul(a3, a4)))
				{

					//these two segment intersect
					const FloatV t = FMul(a1, FRecip(FSub(a2, a1)));

					const Vec3V ipAOri = V3SetZ(points1In0[iStart], FAdd(points1In0TValue[iStart], d));
					const Vec3V ipBOri = V3SetZ(points1In0[iEnd], FAdd(points1In0TValue[iEnd], d));

					const Vec3V pBB = V3NegScaleSub(V3Sub(ipBOri, ipAOri), t, ipAOri);
					const Vec3V pAA = V3SetZ(pBB, d);
					const Vec3V pA = M33TrnspsMulV3(rot, pAA);
					const Vec3V pB = M33TrnspsMulV3(rot, pBB);
					const FloatV pen = FSub(V3GetZ(pBB), V3GetZ(pAA));

					if (FAllGrtr(pen, contactDist))
						continue;


					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(nContactNormal), pen);

					float tmp[4];
					contact_point cp;
					_mm_store_ps(tmp, pen);
					cp.depth = tmp[0];
					_mm_store_ps(tmp, nContactNormal);
					cp.normal = glm::vec3(tmp[0], tmp[1], tmp[2]);
					_mm_store_ps(tmp, pB);
					cp.point = glm::vec3(tmp[0], tmp[1], tmp[2]);

					*_contact_point_cache.add(1) = cp;
				}
			}
		}
	}
}

void ot_terrain_contact_common::addContactsToPatch(const Ps::aos::Vec3VArg patchNormal, const uint32 previousNumContacts, const uint32 _numContacts)
{
	using namespace Ps::aos;
	const Vec3V patchNormalInTriangle = _mesh_to_covex.rotateInv(patchNormal);
	uint32 numContacts = _numContacts;

	if (numContacts > 4)
	{
		//if the current created manifold contacts are more than 4 points, we will reduce the total numContacts to 4
		numContacts = reduceContacts(previousNumContacts, numContacts);
		_contact_point_cache.resize(previousNumContacts + numContacts);
	}

	//calculate the maxPen and transform the patch normal and localPointB into mesh's local space
	FloatV maxPen = FMax();
	for (uint32 i = previousNumContacts; i<_contact_point_cache.size(); ++i)
	{
		FloatV d = FLoad(_contact_point_cache[i].depth);
		glm::vec3 tmp;
		_mm_store_ps(&_contact_point_cache[i].normal[0], patchNormalInTriangle);
		Vec3V point = V3LoadU(&glm::vec3(_contact_point_cache[i].point)[0]);
		point = _mesh_to_covex.transformInv(point);
		_mm_store_ps(&tmp[0], point);
		_contact_point_cache[i].point = tmp;
		maxPen = FMin(maxPen, d);
	}

	const float sq_replace_breaking_threshold = 0.00112500007 * 0.00112500007;

	//get rid of duplicate manifold contacts for the remaining contacts
	for (uint32 i = previousNumContacts; i<_contact_point_cache.size(); ++i)
	{
		for (uint32 j = i + 1; j<_contact_point_cache.size(); ++j)
		{
			glm::vec3 dif = glm::vec3(_contact_point_cache[j].point - _contact_point_cache[i].point);
			float d = glm::dot(dif, dif);
			if (sq_replace_breaking_threshold > d)
			{
				_contact_point_cache[j] = _contact_point_cache[_contact_point_cache.size() - 1];
				j--;
				_contact_point_cache.del(_contact_point_cache.size() - 1, 1);
			}
		}
	}

	//Based on the patch normal and add the newly avaiable manifold points to the corresponding patch
	/*addManifoldPointToPatch(patchNormalInTriangle, maxPen, previousNumContacts);

	PX_ASSERT(mNumContactPatch <PCM_MAX_CONTACTPATCH_SIZE);
	if (mNumContacts >= 16)
	{
		PX_ASSERT(mNumContacts <= 64);
		processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE);
	}*/
}

uint32 ot_terrain_contact_common::reduceContacts(uint32 startIndex, uint32 numPoints)
{
	using namespace Ps::aos;
	coid::dynarray<bool> chosen;
	chosen.resize(numPoints);
	memset(chosen.begin().ptr(),0, numPoints*sizeof(bool));
	FloatV max = Ps::aos::FMax();
	FloatV maxDis = max;
	int32 index = -1;
	contact_point newManifold[4];

	//keep the deepest point
	for (uint32 i = 0; i<numPoints; ++i)
	{
		const FloatV pen = FLoad(_contact_point_cache[startIndex + i].depth);
		if (FAllGrtr(maxDis, pen))
		{
			maxDis = pen;
			index = (int32)i;
		}
	}
	//keep the deepest points in the first position
	newManifold[0] = _contact_point_cache[startIndex + index];
	chosen[index] = true;


	//calculate the furthest away points
	Vec3V v = V3Sub(V3LoadU(&glm::vec3(_contact_point_cache[startIndex].point)[0]), V3LoadU(&glm::vec3(newManifold[0].point)[0]));
	maxDis = V3Dot(v, v);
	index = 0;

	for (uint32 i = 1; i<numPoints; ++i)
	{
		v = V3Sub(V3LoadU(&glm::vec3(_contact_point_cache[startIndex+i].point)[0]), V3LoadU(&glm::vec3(newManifold[0].point)[0]));
		const FloatV d = V3Dot(v, v);
		if (FAllGrtr(d, maxDis))
		{
			maxDis = d;
			index = (int32)i;
		}
	}

	//PX_ASSERT(chosen[index] == false);
	newManifold[1] = _contact_point_cache[startIndex + index];
	chosen[index] = true;


	maxDis = FNeg(max);
	index = -1;
	v = V3Sub(V3LoadU(&glm::vec3(newManifold[1].point)[0]), V3LoadU(&glm::vec3(newManifold[0].point)[0]));
	Vec3V norm = V3Normalize(V3Cross(v, V3LoadU(&newManifold[0].normal[0])));
	FloatV minDis = max;
	int32 index1 = -1;


	//calculate the point furthest way to the segment
	for (uint32 i = 0; i<numPoints; ++i)
	{
		if (!chosen[i])
		{
			v = V3Sub(V3LoadU(&glm::vec3(_contact_point_cache[startIndex +i].point)[0]), V3LoadU(&glm::vec3(newManifold[0].point)[0]));
			const FloatV d = V3Dot(v, norm);
			if (FAllGrtr(d, maxDis))
			{
				maxDis = d;
				index = (int32)i;
			}

			if (FAllGrtr(minDis, d))
			{
				minDis = d;
				index1 = (int32)i;
			}
		}
	}

	//PX_ASSERT(chosen[index] == false && chosen[index1]== false);

	chosen[index] = true;
	newManifold[2] = _contact_point_cache[startIndex + index];

	const FloatV temp = FMul(minDis, maxDis);
	if (FAllGrtr(temp, Ps::aos::FZero()))
	{
		//chose the something further away from newManifold[2] 
		maxDis = FNeg(max);
		for (uint32 i = 0; i<numPoints; ++i)
		{
			if (!chosen[i])
			{
				v = V3Sub(V3LoadU(&glm::vec3(_contact_point_cache[startIndex + i].point)[0]), V3LoadU(&glm::vec3(newManifold[0].point)[0]));
				const FloatV d = V3Dot(v, norm);
				if (FAllGrtr(d, maxDis))
				{
					maxDis = d;
					index1 = (int32)i;
				}
			}
		}

	}

	newManifold[3] = _contact_point_cache[startIndex + index1];
	chosen[index1] = true;

	//copy the new manifold back
	for (uint32 i = 0; i<4; ++i)
	{
		_contact_point_cache[startIndex + i] = newManifold[i];
	}

	return 4;
}

void ot_terrain_contact_common::addManifoldPointToPatch(const glm::vec3 currentPatchNormal, const float maxPen, const uint32 previousNumContacts)
{
/*	using namespace Ps::aos;

	bool foundPatch = false;
	//we have existing patch
	if (previousNumContacts > _contact_point_cache.size())
	{
		//if the direction between the last existing patch normal and the current patch normal are within acceptance epsilon, which means we will be
		//able to merge the last patch's contacts with the current patch's contacts. This is just to avoid to create an extra patch. We have some logic
		//later to refine the patch again
		if (FAllGrtr(V3Dot(mContactPatch[mNumContactPatch - 1].mPatchNormal, currentPatchNormal), mAcceptanceEpsilon))
		{
			//get the last patch
			PCMContactPatch& patch = mContactPatch[mNumContactPatch - 1];

			//remove duplicate contacts
			for (PxU32 i = patch.mStartIndex; i<patch.mEndIndex; ++i)
			{
				for (PxU32 j = previousNumContacts; j<mNumContacts; ++j)
				{
					Vec3V dif = V3Sub(mManifoldContacts[j].mLocalPointB, mManifoldContacts[i].mLocalPointB);
					FloatV d = V3Dot(dif, dif);
					if (FAllGrtr(mSqReplaceBreakingThreshold, d))
					{
						if (FAllGrtr(V4GetW(mManifoldContacts[i].mLocalNormalPen), V4GetW(mManifoldContacts[j].mLocalNormalPen)))
						{
							//The new contact is deeper than the old contact so we keep the deeper contact
							mManifoldContacts[i] = mManifoldContacts[j];
						}
						mManifoldContacts[j] = mManifoldContacts[mNumContacts - 1];
						mNumContacts--;
						j--;
					}
				}
			}
			patch.mEndIndex = mNumContacts;
			patch.mPatchMaxPen = FMin(patch.mPatchMaxPen, maxPen);
			foundPatch = true;
		}
	}

	//If there are no existing patch which match the currentPatchNormal, we will create a new patch
	if (!foundPatch)
	{
		mContactPatch[mNumContactPatch].mStartIndex = previousNumContacts;
		mContactPatch[mNumContactPatch].mEndIndex = mNumContacts;
		mContactPatch[mNumContactPatch].mPatchMaxPen = maxPen;
		mContactPatch[mNumContactPatch++].mPatchNormal = currentPatchNormal;
	}*/
}
#endif

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

    current_processed_triangle->t_flags;
    
    const double3 * offset = current_processed_triangle->parent_offset_p;
    const float3 e01 = current_processed_triangle->b - current_processed_triangle->a;
    const float3 e02 = current_processed_triangle->c - current_processed_triangle->a;
    const float e01_len = glm::length(e01);
    const float e02_len = glm::length(e02);
    const float3 ap(cp.m_localPointB[0] - offset->x - current_processed_triangle->a.x, 
        cp.m_localPointB[1] - offset->y - current_processed_triangle->a.y,
        cp.m_localPointB[2] - offset->z - current_processed_triangle->a.z);
    const float u = glm::dot(ap, e01) / e01_len;
    const float v = glm::dot(ap, e02) / e02_len;

    if (selectNormal(u, v, current_processed_triangle->t_flags)) {
        const float3 n = glm::normalize(glm::cross(e01,e02));
        cp.m_normalWorldOnB = btVector3(n.x,n.y,n.z);
    }

    return true;
}
