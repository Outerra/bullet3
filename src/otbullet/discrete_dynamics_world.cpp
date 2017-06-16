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
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include <LinearMath/btIDebugDraw.h>

#include <ot/glm/coal.h>

#include "ot_terrain_contact_common.h"

#include <comm/timer.h>

#include <ot/glm/glm_ext.h>

/// tmp ////
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
/// /////////

//static const float g_temp_tree_rad = .20f;

const float g_sigma_coef = 1.f;

#ifdef _DEBUG

#include <fstream>

void ot::discrete_dynamics_world::dump_triangle_list_to_obj(const char * fname, float off_x, float off_y, float off_z, float rx, float ry, float rz, float rw) {
    coid::charstr buf;
    uint vtx_count = 0;
    quat q(rw, rx, ry, rz);
    q = glm::inverse(q);


    _triangles.for_each([&](const bt::triangle& t) {
        float4 off(t.parent_offset_p->x - off_x, t.parent_offset_p->y - off_y, t.parent_offset_p->z - off_z,0);
        
        float4 p = q * (float4(t.a,1) + off);
        buf << "v " << t.a.x << " " << t.a.y << " " << t.a.z << "\n";

        p = q * (float4(t.b,1) + off);
        buf << "v " << t.b.x << " " << t.b.y << " " << t.b.z << "\n";

        p = q * (float4(t.c,1) + off);
        buf << "v " << t.c.x << " " << t.c.y << " " << t.c.z << "\n";

        vtx_count += 3;
        buf << "f " << vtx_count - 2 << " " << vtx_count - 1 << " " << vtx_count << "\n";
    });

    std::ofstream ofs;
    ofs.open(fname);
    ofs << buf.c_str();
    ofs.close();
}
#endif


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

        //LOCAL_SINGLETON(ot_terrain_contact_common) common_data = new ot_terrain_contact_common(0.00f,this,_pb_wrap);
        if(!_common_data)
            _common_data = new ot_terrain_contact_common(0.00f,this,_pb_wrap);

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
           
            if (rb->getActivationState() == ISLAND_SLEEPING) {
                continue;
            }

            btPersistentManifold * manifold;
            if (rb->getTerrainManifoldHandle() == UMAX32) {
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
 

			btCollisionObjectWrapper planet_wrapper(0, _planet_body->getCollisionShape(), _planet_body, btTransform::getIdentity(), -1, -1);
			btCollisionObjectWrapper collider_wrapper(0, obj->getCollisionShape(), obj, obj->getWorldTransform(), -1, -1);
			btManifoldResult res(&collider_wrapper, &planet_wrapper);
			
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
            manifold->clearManifold();

			for (uints j = 0; j < _cow_internal.size(); j++) {

				btCollisionObjectWrapper internal_obj_wrapper(_cow_internal[j]._parent,
					_cow_internal[j]._shape,
					obj,
					_cow_internal[j]._worldTransform,
					_cow_internal[j]._partId,
					_cow_internal[j]._index);


                _common_data->set_internal_obj_wrapper(&internal_obj_wrapper);

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
					_common_data->prepare_sphere_collision(&res, _from, float(sph->getRadius()), 0.02f);
                    gContactAddedCallback = friction_combiner_cbk;
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

					_common_data->prepare_capsule_collision(&res, glm::dvec3(p0.x(), p0.y(), p0.z()), glm::dvec3(p1.x(), p1.y(), p1.z()), cap_rad, float(caps->getMargin()));
                    gContactAddedCallback = friction_combiner_cbk;
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

                    min = (max - min) * 0.5;
                    _lod_dim = min[min.minAxis()];
					_common_data->prepare_bt_convex_collision(&res, &internal_obj_wrapper);
                    gContactAddedCallback = GJK_contact_added;
				}
				else {
					continue;
				}

				_triangles.clear();
                _tree_batches.clear();

                get_obb(internal_obj_wrapper.getCollisionShape(), internal_obj_wrapper.getWorldTransform(), _from, _basis);

                //if(!_sphere_intersect(_context, _from , _rad , _lod_dim, _triangles, _trees)) {
                _relocation_offset = _tb_cache.get_array().ptr();
                if (!_aabb_intersect(_context, _from, _basis, _lod_dim, _triangles, _tree_batches, _tb_cache)) {
                    DASSERT(_tree_batches.size() == 0);
                    continue;
                }
                
                if (_relocation_offset != _tb_cache.get_array().ptr()) {
                    repair_tree_batches();
                    repair_tree_collision_pairs();
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
                    _common_data->process_triangle_cache(_triangles);
#ifdef _PROFILING_ENABLED
                    _stats.triangles_processed_count += _triangles.size();
                    _stats.triangle_processing_time_ms += timer.time_ns() * 0.000001f;
#endif // _PROFILING_ENABLED

				}

                if (_tree_batches.size() > 0) {
                    prepare_tree_collision_pairs(obj, _tree_batches, frame_count);
                }

				_common_data->process_collision_points();
			}

            if (manifold->getNumContacts() == 0) {
                getDispatcher()->releaseManifold(manifold);
                _manifolds.get_item(rb->getTerrainManifoldHandle());
                _manifolds.del(_manifolds.get_item(rb->getTerrainManifoldHandle()));
                rb->setTerrainManifoldHandle(UMAX32);
            }
		}

        ++frame_count;
        gContactAddedCallback = nullptr;
	}

    void discrete_dynamics_world::prepare_tree_collision_pairs(btCollisionObject * cur_obj, const coid::dynarray<uint>& tree_batches_cache, uint32 frame)
    {        
        for (uints i = 0; i < tree_batches_cache.size(); i++) {
            uint bid = tree_batches_cache[i];
            bt::tree_batch * tb = _tb_cache.get_item(bid) ;

            if (tb->last_frame_used == UMAX32) {
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
                if (coal::distance_point_segment_sqr(cen_rel, float3(0, 0, 0), p) < glm::pow(tb->trees[j].radius + _rad,2.f)) {
                    tree_collision_pair tcp(cur_obj, bid, j);
                    tree_collision_pair * cached_tcp = _tree_collision_pairs.find_if([&](const tree_collision_pair& _tcp) {
                        return tcp == _tcp;
                    });

                    if (!cached_tcp) {
                        cached_tcp = _tree_collision_pairs.add();
                        cached_tcp->init_with(cur_obj, bid, j, tb->trees[j]);
                        cached_tcp->manifold = getDispatcher()->getNewManifold(cached_tcp->obj, &tb->info(j)->obj);
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
            btCapsuleShape * t_cap = new (&tci->shape) btCapsuleShape(t.radius, t.height);
            btCollisionObject * t_col = new (&tci->obj) btCollisionObject();
            t_col->setCollisionShape(t_cap);
            t_col->setWorldTransform(t_trans);
        }
    }

    void discrete_dynamics_world::process_tree_collisions(btScalar time_step)
    {
        _tree_collision_pairs.for_each([&](tree_collision_pair&  tcp) {
            if (!tcp.active) {
                tcp.tree->spring_force_uv[0] = 0;
                tcp.tree->spring_force_uv[1] = 0;
                _tree_collision_pairs.del(&tcp);
                return;
            }

            if (!tcp.reused) {
                dispatcher->releaseManifold(manifold);
                _tree_collision_pairs.del(&tcp);
                return;
            }

            btRigidBody * rb_obj = btRigidBody::upcast(tcp.obj);
            btCollisionObjectWrapper obj1_wrapper(0, tcp.obj->getCollisionShape(), tcp.obj, tcp.obj->getWorldTransform(), -1, -1);
            bt::tree_collision_info* tree_col_info = get_tree_collision_info(tcp);
            bt::tree* tree_inf = get_tree(tcp);
            btCollisionObjectWrapper obj2_wrapper(0, &tree_col_info->shape, &tree_col_info->obj, tree_col_info->obj.getWorldTransform(), -1, -1);

            btManifoldResult res(&obj1_wrapper, &obj2_wrapper);
            res.setPersistentManifold(manifold);

            btCollisionAlgorithm * algo = dispatcher->findAlgorithm(&obj1_wrapper, &obj2_wrapper, manifold);

            if (algo) {
                algo->processCollision(&obj1_wrapper, &obj2_wrapper, getDispatchInfo(), &res);
                algo->~btCollisionAlgorithm();
                dispatcher->freeCollisionAlgorithm(algo);
            }
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
                    
                    const float l = float(tree_col_info->shape.getHalfHeight() + min_cp.m_localPointB[tree_col_info->shape.getUpAxis()]);
                    const float dp = float(rb_obj->getLinearVelocity().length() / (rb_obj->getInvMass()));

                    float f = dp * tcp.tc_ctx.max_collision_duration_inv;
                    float sig = (f*l*tree_inf->radius) / tree_inf->I;

                    if (tree_inf->sig_max <= sig) {
                        tcp.tc_ctx.custom_handling = true;
                        tcp.tc_ctx.braking_force = (tree_inf->sig_max * tree_inf->I) / (l*tree_inf->radius);
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
                    float3 displacement = _tree_collision(rb_obj, tcp.tc_ctx,float(time_step),_tb_cache);
                    if (m_debugDrawer) {
                        bool is_new = false;
                        uint16 tree_iden = tree_inf->identifier;
                        tree_flex_inf * slot = _debug_terrain_trees_active.find_or_insert_value_slot_uninit(tree_iden, &is_new);
                        new(slot) tree_flex_inf(displacement, tree_iden);
                    }
                }

                manifold->clearManifold();
            }
        });
    }

    void discrete_dynamics_world::repair_tree_collision_pairs()
    {
        _tree_collision_pairs.for_each([&](tree_collision_pair& tcp) {
            bt::tree_collision_info* tci = get_tree_collision_info(tcp);
            tcp.manifold->setBodies(tcp.obj, &tci->obj);
        });
    }

    void discrete_dynamics_world::repair_tree_batches()
    {
        _tb_cache.for_each([&](bt::tree_batch& tb) {
            if (tb.last_frame_used != 0xffffffff) {
                for (uint i = 0; i < tb.tree_count; i++) {
                    bt::tree_collision_info* tci = tb.info(i);
                    tci->obj.setCollisionShape(&tci->shape);
                }
            }
        });
    }

    bt::tree_collision_info * discrete_dynamics_world::get_tree_collision_info(const tree_collision_pair& tcp)
    {
        uint tree_id = tcp.tree_col_info;
        uint bid = tree_id >> 4;
        uint8 tid = tree_id & 0xf;
        bt::tree_batch* tb = _tb_cache.get_item(bid);
        return tb->info(tid);
    }

    bt::tree * discrete_dynamics_world::get_tree(const tree_collision_pair & tcp)
    {
        uint tree_id = tcp.tree_col_info;
        uint bid = tree_id >> 4;
        uint8 tid = tree_id & 0xf;
        bt::tree_batch* tb = _tb_cache.get_item(bid);
        return &tb->trees[tid];
    }

    void discrete_dynamics_world::query_volume_sphere(const double3& pos, float rad, coid::dynarray<btCollisionObject *>& result)
    {
#ifdef _DEBUG
        bt32BitAxisSweep3 * broad = dynamic_cast<bt32BitAxisSweep3 *>(m_broadphasePairCache);
        DASSERT(broad!=nullptr);
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
        
        while (_processing_stack.pop(cur_node)) {
            const btVector3& bt_aabb_cen = cur_node->volume.Center();
            const btVector3& bt_aabb_half = cur_node->volume.Extents();
            glm::double3 aabb_cen(bt_aabb_cen[0], bt_aabb_cen[1], bt_aabb_cen[2]);
            glm::double3 aabb_half(bt_aabb_half[0], bt_aabb_half[1], bt_aabb_half[2]);
            if (coal::intersects_sphere_aabb(pos, (double)rad, aabb_cen, aabb_half, (double*)nullptr)) {
                if (cur_node->isleaf()) {
                    if(cur_node->data){
                        btDbvtProxy* dat = reinterpret_cast<btDbvtProxy*>(cur_node->data);
                        result.push(reinterpret_cast<btCollisionObject*>(dat->m_clientObject));
                    }
                }
                else {
                    _processing_stack.push(cur_node->childs[0]);
                    _processing_stack.push(cur_node->childs[1]);
                }
            };
        }
    }

    void discrete_dynamics_world::query_volume_frustum(const double3 & pos, const float4 * f_planes_norms, uint8 nplanes, bool include_partial,coid::dynarray<btCollisionObject*>& result)
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
            glm::double3 aabb_cen(bt_aabb_cen[0], bt_aabb_cen[1], bt_aabb_cen[2]);
            glm::float3 aabb_half(bt_aabb_half[0], bt_aabb_half[1], bt_aabb_half[2]);

            if (coal::intersects_frustum_aabb(aabb_cen, aabb_half, pos, f_planes_norms, nplanes, true)) {
                if (cur_node->isleaf()) {
                    if(cur_node->data){
                        btDbvtProxy* dat = reinterpret_cast<btDbvtProxy*>(cur_node->data);
                        btCollisionObject* leaf_obj = reinterpret_cast<btCollisionObject*>(dat->m_clientObject);

                        const btVector3& cen = leaf_obj->getWorldTransform().getOrigin();
                        const float3 aabb_pos(float(cen[0] - pos.x), float(cen[1] - pos.y), float(cen[2] - pos.z));
                        bool passes = true;
                        for (uint8 p = 0; p < nplanes; p++) {
                            float3 n(f_planes_norms[p]);
                            btVector3 min, max;
                            btTransform t(btMatrix3x3(n.x, n.y, n.z, 0., 0., 0., 0., 0., 0.));
                            leaf_obj->getCollisionShape()->getAabb(t,min,max);
                            const float np = float(max[0] - min[0]) * 0.5f;
                            const float mp = glm::dot(n, aabb_pos) + f_planes_norms[p].w;
                            if ((include_partial ? mp + np : mp - np) < 0.0f) {
                                passes = false;
                                break;
                            }
                        }

                        if (passes) {
                            result.push(leaf_obj);
                        }
                    }
                }
                else {
                    _processing_stack.push(cur_node->childs[0]);
                    _processing_stack.push(cur_node->childs[1]);
                }
            };
        }
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
                btVector3 bt_p2 = bt_p1 + bt_norm * t->height + btVector3(displacement.x,displacement.y,displacement.z)*t->max_flex;

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

        , _debug_terrain_triangles(1024)
        , _debug_terrain_trees(1024)
        , _relocation_offset(0)
	{
		btTriangleShape * ts = new btTriangleShape();
		ts->setMargin(0.0f);
        btRigidBody::btRigidBodyConstructionInfo info(0, 0, ts);

		_planet_body = new btRigidBody(info);
		_planet_body->setRestitution(0.0f);
        _planet_body->setCollisionFlags(_planet_body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		_cow_internal.reserve(128,false);
		_compound_processing_stack.reserve(128, false);
	}


}// end namespace ot
