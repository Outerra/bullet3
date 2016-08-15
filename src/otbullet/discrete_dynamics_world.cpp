#include "discrete_dynamics_world.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include <LinearMath/btIDebugDraw.h>

#include <ot/glm/coal.h>

#include "ot_terrain_contact_common.h"

#include <comm/timer.h>

#include <ot/glm/glm_ext.h>

//static const float g_temp_tree_rad = .20f;

const float g_sigma_coef = 1.f;

namespace ot {

	void discrete_dynamics_world::internalSingleStepSimulation(btScalar timeStep)
	{
#ifdef _PROFILING_ENABLED
        static coid::nsec_timer timer;
        static coid::nsec_timer timer1;
        timer.reset();
        timer1.reset();
        reset_stats();

#endif // _PROFILING_ENABLED

        if (0 != m_internalPreTickCallback) {
			(*m_internalPreTickCallback)(this, timeStep);
		}
        
		///apply gravity, predict motion
		predictUnconstraintMotion(timeStep);

		btDispatcherInfo& dispatchInfo = getDispatchInfo();

		dispatchInfo.m_timeStep = timeStep;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_debugDraw = getDebugDrawer();


		createPredictiveContacts(timeStep);

		///perform collision detection
		performDiscreteCollisionDetection();

#ifdef _PROFILING_ENABLED
        _stats.before_ot_phase_time_ms = timer.time_ns() * 1e-6f;
#endif // _PROFILING_ENABLED

		//perform outerra terrain collision detecion

//		ot_terrain_collision_step_cleanup();
		ot_terrain_collision_step();
		process_tree_collisions(timeStep);

#ifdef _PROFILING_ENABLED
        timer.reset();
#endif // _PROFILING_ENABLED

		calculateSimulationIslands();


		getSolverInfo().m_timeStep = timeStep;



		///solve contact and other joint constraints
		solveConstraints(getSolverInfo());

		///CallbackTriggers();

		///integrate transforms

		integrateTransforms(timeStep);

		///update vehicle simulation
		updateActions(timeStep);

		updateActivationState(timeStep);

		if (0 != m_internalTickCallback) {
			(*m_internalTickCallback)(this, timeStep);
		}

#ifdef _PROFILING_ENABLED
        _stats.after_ot_phase_time_ms = timer.time_ns() * 1e-6f;
        _stats.total_time_ms = timer1.time_ns() * 1e-6f;
#endif // _PROFILING_ENABLED

	}

    void discrete_dynamics_world::removeRigidBody(btRigidBody * body)
    {
        const uint32 m_id = body->getTerrainManifoldHandle();
        if ( m_id != 0xffffffff) {
            btPersistentManifold ** m_ptr = _manifolds.get_item(m_id);
            _manifolds.del(m_ptr);
            m_dispatcher1->releaseManifold(*m_ptr);
        }
		
		body->setTerrainManifoldHandle(0xffffffff);

        btDiscreteDynamicsWorld::removeRigidBody(body);
    }

	void discrete_dynamics_world::ot_terrain_collision_step()
	{
        static uint32 frame_count;

#ifdef _PROFILING_ENABLED
        static coid::nsec_timer timer;
#endif // _PROFILING_ENABLED

        
        if (m_debugDrawer) {
            _debug_terrain_triangles.clear();
            _debug_terrain_trees.reset();
            _debug_terrain_trees_active.reset();
        }

        LOCAL_SINGLETON(ot_terrain_contact_common) common_data = new ot_terrain_contact_common(0.00f,this,_pb_wrap);
		for (int i = 0; i < m_collisionObjects.size(); i++) {
			_cow_internal.clear();
            _compound_processing_stack.clear();
			btCollisionObject * obj = m_collisionObjects[i];
            btRigidBody * rb = 0;
            if (!obj->isStaticObject()) {
                rb = reinterpret_cast<btRigidBody *>(obj);
            }


			if (!rb || (obj->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE &&
					obj->getCollisionShape()->getShapeType() != CAPSULE_SHAPE_PROXYTYPE &&
					!obj->getCollisionShape()->isConvex() &&
					obj->getCollisionShape()->getShapeType() != COMPOUND_SHAPE_PROXYTYPE))
			{
				continue;
			}


            btPersistentManifold * manifold;
            if (rb->getTerrainManifoldHandle() == 0xffffffff) {
                manifold = getDispatcher()->getNewManifold(obj, _planet_body);
                btPersistentManifold ** manifold_h_ptr = _manifolds.add();
                *manifold_h_ptr = manifold;
                uints manifold_handle = _manifolds.get_item_id(manifold_h_ptr);
                rb->setTerrainManifoldHandle(manifold_handle);
                manifold->setContactBreakingThreshold(obj->getCollisionShape()->getContactBreakingThreshold(gContactBreakingThreshold));
            }
            else {
                manifold = *_manifolds.get_item(rb->getTerrainManifoldHandle());
            }

            if (rb->getActivationState() == ISLAND_SLEEPING) {
                continue;
            }

			btCollisionObjectWrapper planet_wrapper(0, _planet_body->getCollisionShape(), _planet_body, btTransform::getIdentity(), -1, -1);
			btCollisionObjectWrapper collider_wrapper(0, obj->getCollisionShape(), obj, obj->getWorldTransform(), -1, -1);
			btManifoldResult res(&collider_wrapper, &planet_wrapper);
			

            int cached_points = manifold->getNumContacts();

			if (obj->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
				btCompoundShape * cs = reinterpret_cast<btCompoundShape *>(obj->getCollisionShape());
				new (_compound_processing_stack.add_uninit(1)) compound_processing_entry(cs, obj->getWorldTransform());
				compound_processing_entry curr;
				while (_compound_processing_stack.pop(curr)) {
					if (curr._shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
						btCompoundShape * curr_cs = reinterpret_cast<btCompoundShape *>(curr._shape);
						for (int j = 0; j < curr_cs->getNumChildShapes(); j++) {
							new (_compound_processing_stack.add_uninit(1)) compound_processing_entry(curr_cs->getChildShape(j),curr._world_trans * curr_cs->getChildTransform(j));
						}
					}
					else {
						new (_cow_internal.add_uninit(1)) btCollisionObjectWrapperCtorArgs(&collider_wrapper,curr._shape,obj,curr._world_trans,-1,-1);
					}
				}
			}
			else {
				new (_cow_internal.add_uninit(1)) btCollisionObjectWrapperCtorArgs(0, obj->getCollisionShape(), obj, obj->getWorldTransform(), -1, -1);

			}

            res.setPersistentManifold(manifold);

			for (uints j = 0; j < _cow_internal.size(); j++) {

				btCollisionObjectWrapper internal_obj_wrapper(_cow_internal[j]._parent,
					_cow_internal[j]._shape,
					obj,
					_cow_internal[j]._worldTransform,
					_cow_internal[j]._partId,
					_cow_internal[j]._index);


                common_data->set_internal_obj_wrapper(&internal_obj_wrapper);

				btVector3 sc = internal_obj_wrapper.getWorldTransform().getOrigin();
				//int face = ot::xyz_to_cubeface(&sc.m_floats[0]);
				//int levs = 0;
				//float dist;
				//terrain_mesh::auxdata & aux = terrain_mesh::aux();
				_from = double3(sc.x(), sc.y(), sc.z());
				//_col_shape = bt::csSphere;

				if (internal_obj_wrapper.getCollisionShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE) {
					const btSphereShape * sph = reinterpret_cast<const btSphereShape*>(internal_obj_wrapper.getCollisionShape());
					_rad = float(sph->getRadius() + 0.02);
                    _lod_dim = _rad;
					common_data->prepare_sphere_collision(&res, _from, float(sph->getRadius()), 0.02f);
				}
				else if (internal_obj_wrapper.getCollisionShape()->getShapeType() == CAPSULE_SHAPE_PROXYTYPE) {
					const btCapsuleShape * caps = reinterpret_cast<const btCapsuleShape*>(internal_obj_wrapper.getCollisionShape());
					float cap_rad = float(caps->getRadius());
					float cap_hheight = float(caps->getHalfHeight());
					_rad = cap_rad + cap_hheight + 0.04f;
                    _lod_dim = cap_rad;

					btVector3 main_axis = internal_obj_wrapper.getWorldTransform().getBasis().getColumn(caps->getUpAxis());
					btVector3 p0 = sc + (main_axis * cap_hheight);
					btVector3 p1 = sc - (main_axis * cap_hheight);

					common_data->prepare_capsule_collision(&res, glm::dvec3(p0.x(), p0.y(), p0.z()), glm::dvec3(p1.x(), p1.y(), p1.z()), cap_rad, float(caps->getMargin()));
				}
				else if (internal_obj_wrapper.getCollisionShape()->isConvex()) {
					btTransform t = internal_obj_wrapper.getWorldTransform();
					btQuaternion q = t.getRotation();
					btVector3 p = t.getOrigin();
                    btVector3 min;
                    btVector3 max;
					btScalar rad;
					internal_obj_wrapper.getCollisionShape()->getBoundingSphere(min, rad);
                    internal_obj_wrapper.getCollisionShape()->getAabb(t, min, max);

                    _rad = (float)rad;

                    min = (max - min) / 2.0;
                    _lod_dim = (float)min[min.minAxis()];
					common_data->prepare_bt_convex_collision(&res, &internal_obj_wrapper);
				}
				else {
					continue;
				}

				_triangles.clear();
                _tree_batches.clear();

                get_obb(internal_obj_wrapper.getCollisionShape(), internal_obj_wrapper.getWorldTransform(), _from, _basis);

                //if(!_sphere_intersect(_context, _from , _rad , _lod_dim, _triangles, _trees)) {
                if (!_aabb_intersect(_context, _from, _basis, _lod_dim, _triangles, _tree_batches)) {
                    continue;
                }

				if (_triangles.size() > 0) {

                    if (m_debugDrawer) {
                        _triangles.for_each([&](const bt::triangle& t) {
                            *_debug_terrain_triangles.push() = t;
                        });
                    }

#ifdef _PROFILING_ENABLED
                    timer.reset();

#endif // _PROFILING_ENABLED
                    common_data->process_triangle_cache(_triangles);
#ifdef _PROFILING_ENABLED
                    _stats.triangles_processed_count += _triangles.size();
                    _stats.triangle_processing_time_ms += timer.time_ns() * 0.000001f;
#endif // _PROFILING_ENABLED

				}

                if (_tree_batches.size() > 0) {
                    prepare_tree_collision_pairs(obj, _tree_batches, frame_count);
                }

				common_data->process_collision_points();
			}

            int num_contacts = manifold->getNumContacts();

            res.refreshContactPoints();

            if (manifold->getNumContacts() == 0) {
                getDispatcher()->releaseManifold(manifold);
                _manifolds.get_item(rb->getTerrainManifoldHandle());
                _manifolds.del(_manifolds.get_item(rb->getTerrainManifoldHandle()));
                rb->setTerrainManifoldHandle(0xffffffff);
            }
		}

        ++frame_count;
	}

    void discrete_dynamics_world::prepare_tree_collision_pairs(btCollisionObject * cur_obj, const coid::dynarray<bt::tree_batch*>& tree_batches_cache, uint32 frame)
    {
        for (uints i = 0; i < tree_batches_cache.size(); i++) {
            bt::tree_batch * tb = tree_batches_cache[i];

            if (tb->last_frame_used == 0xffffffff) {
                build_tb_collision_info(tb);
            }

            tb->last_frame_used = frame;

            for (uint8 j = 0; j < tb->tree_count; j++) {
                if (tb->trees[j].spring_force_uv[0] == -128 && tb->trees[j].spring_force_uv[1] != -128) // broken tree
                    continue;

                if (m_debugDrawer) {
                    bt::tree ** slot = _debug_terrain_trees.find_or_insert_value_slot(tb->trees[j].identifier, 0);
                    *slot = &tb->trees[j];
                }

                float3 p = float3(glm::normalize(tb->trees[j].pos)) * tb->trees[j].height;
                float3 cen_rel(_from - tb->trees[j].pos);
                if (coal::distance_point_segment_sqr(cen_rel, float3(0, 0, 0), p) < glm::pow(tb->info(j)->tree_inf->radius + _rad,2.f)) {
                    tree_collision_pair tcp(cur_obj, tb->info(j));
                    tree_collision_pair * cached_tcp = _tree_collision_pairs.find_if([&](const tree_collision_pair& _tcp) {
                        return tcp == _tcp;
                    });

                    if (!cached_tcp) {
                        cached_tcp = _tree_collision_pairs.add();
                        cached_tcp->init_with(cur_obj,tcp.tree_col_info,tb->trees[j]);
                        cached_tcp->manifold = getDispatcher()->getNewManifold(cached_tcp->obj, &cached_tcp->tree_col_info->obj);
                    }
                    
                    cached_tcp->reused = true;
                }
            }
        }
    }

    void discrete_dynamics_world::build_tb_collision_info(bt::tree_batch * tb)
    {
        for (uint8 i = 0; i < tb->tree_count; i++) {
            bt::tree_collision_info * tci = tb->info(i);
            bt::tree & t = tb->trees[i];
            double3 normal = glm::normalize(t.pos);
            double3 pos = t.pos + normal * (double)t.height * .5;
            quat rot = glm::make_quat(float3(0.f, 1.f, 0.f), (float3)normal);
            btTransform t_trans;
            t_trans.setOrigin(btVector3(pos.x,pos.y,pos.z));
            t_trans.setRotation(btQuaternion(rot.x,rot.y,rot.z,rot.w));
            tci->tree_inf = &t;
            btCapsuleShape * t_cap = new (&tci->shape) btCapsuleShape(t.radius, t.height);
            btCollisionObject * t_col = new (&tci->obj) btCollisionObject();
            t_col->setCollisionShape(t_cap);
            t_col->setWorldTransform(t_trans);
        }
    }

    void discrete_dynamics_world::process_tree_collisions(btScalar time_step)
    {
        _tree_collision_pairs.for_each([&](tree_collision_pair&  tcp) {
            btDispatcher * dispatcher = getDispatcher();
            btPersistentManifold * manifold = tcp.manifold;
            DASSERT(manifold);

            if (!tcp.reused) {
                dispatcher->releaseManifold(manifold);
                _tree_collision_pairs.del(&tcp);
                return;
            }

            btRigidBody * rb_obj = btRigidBody::upcast(tcp.obj);
            btCollisionObjectWrapper obj1_wrapper(0, tcp.obj->getCollisionShape(), tcp.obj, tcp.obj->getWorldTransform(), -1, -1);
            btCollisionObjectWrapper obj2_wrapper(0, &tcp.tree_col_info->shape, &tcp.tree_col_info->obj, tcp.tree_col_info->obj.getWorldTransform(), -1, -1);

            btManifoldResult res(&obj1_wrapper, &obj2_wrapper);
            res.setPersistentManifold(manifold);

            btCollisionAlgorithm * algo = dispatcher->findAlgorithm(&obj1_wrapper, &obj2_wrapper, manifold);

            if (algo) {
                algo->processCollision(&obj1_wrapper, &obj2_wrapper, getDispatchInfo(), &res);
                algo->~btCollisionAlgorithm();
                dispatcher->freeCollisionAlgorithm(algo);
            }

            tcp.reused = false;
            res.refreshContactPoints();

            if (!tcp.tc_ctx.collision_started) {
                if (manifold->getNumContacts()){ 
                    btManifoldPoint& min_cp = manifold->getContactPoint(0);
                    for (int i = 1; i < manifold->getNumContacts(); i++) {
                        btManifoldPoint& tmp_cp = manifold->getContactPoint(i);
                        if (tmp_cp.getDistance() < 0 && tmp_cp.getDistance() > min_cp.getDistance())
                            min_cp = tmp_cp;
                    }

                    if (min_cp.getDistance() > 0) {
                        manifold->clearManifold();
                        return;
                    }
                    
                    const float l = float(tcp.tree_col_info->shape.getHalfHeight() + min_cp.m_localPointB[tcp.tree_col_info->shape.getUpAxis()]);
                    const float dp = float(rb_obj->getLinearVelocity().length() / (rb_obj->getInvMass()));

                    float f = dp * tcp.tc_ctx.max_collision_duration_inv;
                    float sig = (f*l*tcp.tree_col_info->tree_inf->radius) / tcp.tree_col_info->tree_inf->I;

                    if (tcp.tree_col_info->tree_inf->sig_max <= sig) {
                        tcp.tc_ctx.custom_handling = true;
                        tcp.tc_ctx.braking_force = (tcp.tree_col_info->tree_inf->sig_max * tcp.tree_col_info->tree_inf->I) / (l*tcp.tree_col_info->tree_inf->radius);
                        tcp.tc_ctx.force_apply_pt = min_cp.m_localPointA;
                        tcp.tc_ctx.force_dir = -rb_obj->getLinearVelocity().normalized();  
                        tcp.tc_ctx.orig_tree_dir = rb_obj->getWorldTransform().getOrigin().normalized();
                        tcp.tc_ctx.l = l;
                     }

                    tcp.tc_ctx.collision_started = true;
                }
            }

            if (tcp.tc_ctx.custom_handling) {
                if (tcp.tc_ctx.collision_duration < tcp.tc_ctx.max_collision_duration) {
                    float3 displacement = _tree_collision(rb_obj, tcp.tc_ctx,float(time_step));
                    if (m_debugDrawer) {
                        bool is_new = false;
                        uint16 tree_iden = tcp.tree_col_info->tree_inf->identifier;
                        tree_flex_inf * slot = _debug_terrain_trees_active.find_or_insert_value_slot_uninit(tree_iden, &is_new);
                        new(slot) tree_flex_inf(displacement, tree_iden);
                    }
                }

                manifold->clearManifold();
            }
        });
    }

    void discrete_dynamics_world::get_obb(const btCollisionShape * cs,const btTransform& t, double3& cen, float3x3& basis)
    {
        btVector3 min, max;
        cs->getAabb(btTransform::getIdentity(), min, max);
        btVector3 bt_cen = (min + max) * btScalar(0.5);
        btVector3 half = max - bt_cen;
        bt_cen += t.getOrigin();
        btMatrix3x3 bt_basis;
        bt_basis[0] = t.getBasis() * btVector3(half[0], 0, 0);
        bt_basis[1] = t.getBasis() * btVector3(0, half[1], 0);
        bt_basis[2] = t.getBasis() * btVector3(0, 0, half[2]);

        cen = double3( bt_cen[0], bt_cen[1], bt_cen[2] );
        basis[0] = float3(bt_basis[0][0], bt_basis[0][1], bt_basis[0][2]);
        basis[1] = float3(bt_basis[1][0], bt_basis[1][1], bt_basis[1][2]);
        basis[2] = float3(bt_basis[2][0], bt_basis[2][1], bt_basis[2][2]);
    }

    void discrete_dynamics_world::oob_to_aabb(const btVector3 & src_cen, const btMatrix3x3 & src_basis, const btVector3 & dst_cen, const btMatrix3x3 & dst_basis, btVector3 & aabb_cen, btVector3 & aabb_half)
    {
        aabb_cen = src_cen - dst_cen;
        aabb_half[0] = dst_basis[0].dot(src_basis[0]) + dst_basis[0].dot(src_basis[1]) + dst_basis[0].dot(src_basis[2]);
        aabb_half[1] = dst_basis[1].dot(src_basis[0]) + dst_basis[0].dot(src_basis[1]) + dst_basis[0].dot(src_basis[2]);
        aabb_half[2] = dst_basis[2].dot(src_basis[0]) + dst_basis[0].dot(src_basis[1]) + dst_basis[0].dot(src_basis[2]);
    }

    void discrete_dynamics_world::debugDrawWorld()
    {
        if (!m_debugDrawer) {
            return;
        }

        btDiscreteDynamicsWorld::debugDrawWorld();

        btVector3 cl_white(1, 1, 1);

        for (uints i = 0; i < _debug_terrain_triangles.size(); i++) {
            btVector3 parent_offset(_debug_terrain_triangles[i].parent_offset_p->x, 
                _debug_terrain_triangles[i].parent_offset_p->y, 
                _debug_terrain_triangles[i].parent_offset_p->z);
            
            btVector3 a(_debug_terrain_triangles[i].a.x ,
                _debug_terrain_triangles[i].a.y, 
                _debug_terrain_triangles[i].a.z);
            
            btVector3 b(_debug_terrain_triangles[i].b.x, 
                _debug_terrain_triangles[i].b.y, 
                _debug_terrain_triangles[i].b.z);
            
            btVector3 c(_debug_terrain_triangles[i].c.x, 
                _debug_terrain_triangles[i].c.y,
                _debug_terrain_triangles[i].c.z);

            a += parent_offset;
            b += parent_offset;
            c += parent_offset;

            m_debugDrawer->drawLine(a, b,cl_white);
            m_debugDrawer->drawLine(b, c, cl_white);
            m_debugDrawer->drawLine(c, a, cl_white);
        }

        _debug_terrain_trees.for_each([&](const bt::tree * t) {

                
                
                const tree_flex_inf* tfi = _debug_terrain_trees_active.find_value(t->identifier);
                float3 displacement = (tfi) ? tfi->_flex : float3(0);

                btVector3 bt_p1(t->pos.x, t->pos.y, t->pos.z);
                btVector3 bt_norm(bt_p1.normalized());
                btVector3 bt_p2 = bt_p1 + bt_norm * t->height + btVector3(displacement.x,displacement.y,displacement.z);

                m_debugDrawer->drawLine(bt_p1, bt_p2, cl_white);
        });
    }

    discrete_dynamics_world::discrete_dynamics_world(btDispatcher * dispatcher,
		btBroadphaseInterface * pairCache, 
		btConstraintSolver * constraintSolver, 
		btCollisionConfiguration * collisionConfiguration,
        fn_ext_collision ext_collider,
        fn_process_tree_collision ext_tree_col,
		const void* context)
		: btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
        , _sphere_intersect(ext_collider)
        , _tree_collision(ext_tree_col)
		, _context(context)

        , _debug_draw_terrain(false)
        , _debug_terrain_triangles(1024)
        , _debug_terrain_trees(1024)
	{
		btTriangleShape * ts = new btTriangleShape();
		ts->setMargin(0.0f);
        btRigidBody::btRigidBodyConstructionInfo info(0, 0, ts);

		_planet_body = new btRigidBody(info);
		_planet_body->setRestitution(0.0f);

		//_tree_cache.reserve(1024);
		//ptree_cache = &_tree_cache;
		_cow_internal.reserve(128,false);
		_compound_processing_stack.reserve(128, false);
		//_pb_wrap = new btCollisionObjectWrapper(0,ts, _planet_body,btTransform(),-1,-1);
	}


}// end namespace ot