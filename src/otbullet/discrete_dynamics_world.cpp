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


#include <ot/glm/coal.h>

#include "ot_terrain_contact_common.h"

#include <comm/timer.h>
#include <comm/dynarray.h>

#include <ot/glm/glm_ext.h>

static const float g_temp_tree_rad = .2f;

namespace ot {

	void discrete_dynamics_world::internalSingleStepSimulation(btScalar timeStep)
	{

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

		//perform outerra terrain collision detecion

//		ot_terrain_collision_step_cleanup();
		ot_terrain_collision_step();
		process_tree_collisions();

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
	}

    void discrete_dynamics_world::removeRigidBody(btRigidBody * body)
    {
        const uint32 m_id = body->getTerrainManifoldHandle();
        if ( m_id != 0xffffffff) {
            btPersistentManifold ** m_ptr = _manifolds.get_item(m_id);
            _manifolds.del(m_ptr);
            m_dispatcher1->releaseManifold(*m_ptr);
        }

        btDiscreteDynamicsWorld::removeRigidBody(body);
    }

	void discrete_dynamics_world::ot_terrain_collision_step()
	{
        static uint32 frame_count;
        ot_terrain_contact_common common_data(0.00f,this,_pb_wrap);
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

                if (_cow_internal[j]._shape->getShapeType() == CYLINDER_SHAPE_PROXYTYPE) {
                    continue;
                }

                common_data.set_internal_obj_wrapper(&internal_obj_wrapper);

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
					common_data.prepare_sphere_collision(&res, _from, float(sph->getRadius()), 0.02f);
				}
				else if (internal_obj_wrapper.getCollisionShape()->getShapeType() == CAPSULE_SHAPE_PROXYTYPE) {
					const btCapsuleShape * caps = reinterpret_cast<const btCapsuleShape*>(internal_obj_wrapper.getCollisionShape());
					float cap_rad = float(caps->getRadius());
					float cap_hheight = float(caps->getHalfHeight());
					_rad = cap_rad + cap_hheight + 0.04f;

					btVector3 main_axis = internal_obj_wrapper.getWorldTransform().getBasis().getColumn(caps->getUpAxis());
					btVector3 p0 = sc + (main_axis * cap_hheight);
					btVector3 p1 = sc - (main_axis * cap_hheight);

					common_data.prepare_capsule_collision(&res, glm::dvec3(p0.x(), p0.y(), p0.z()), glm::dvec3(p1.x(), p1.y(), p1.z()), cap_rad, float(caps->getMargin()));
				}
				else if (internal_obj_wrapper.getCollisionShape()->isConvex()) {
					btTransform t = internal_obj_wrapper.getWorldTransform();
					btQuaternion q = t.getRotation();
					btVector3 p = t.getOrigin();
					btVector3 dummy;
					btScalar rad;
					internal_obj_wrapper.getCollisionShape()->getBoundingSphere(dummy, rad);
					_rad = (float)rad;

					common_data.prepare_bt_convex_collision(&res, &internal_obj_wrapper);
				}
				else {
					continue;
				}

				_triangles.clear();
                _trees.clear();

                if(!_sphere_intersect(_context, _from, _rad, _triangles, _trees))
                    continue;

				if (_triangles.size() > 0) {
					common_data.process_triangle_cache(_triangles);
				}

                if (_trees.size() > 0) {
                    process_trees_cache(obj, _trees, frame_count);
                }

				common_data.process_collision_points();
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

	void discrete_dynamics_world::ot_terrain_collision_step_cleanup()
	{
	/*	//zatial takto natvrdo
		_manifolds.for_each([&](btPersistentManifold * m) {
			getDispatcher()->releaseManifold(m);
		});

		_manifolds.clear();*/
	}

    void discrete_dynamics_world::process_trees_cache(btCollisionObject * cur_obj, const coid::dynarray<bt::tree_batch*>& trees_cache, uint32 frame)
    {
        for (uints i = 0; i < trees_cache.size(); i++) {
            bt::tree_batch * tb = trees_cache[i];
            if (tb->last_frame_used == 0xffffffff) {
                build_tb_collision_info(tb);
            }

            tb->last_frame_used = frame;

            for (uint8 j = 0; j < tb->tree_count; j++) {
                float3 p = float3(glm::normalize(tb->trees[j].pos)) * tb->trees[j].height;
                float3 cen_rel(_from - tb->trees[j].pos);
                if (coal::distance_point_segment_sqr(cen_rel, float3(0, 0, 0), p) < glm::pow(g_temp_tree_rad + _rad,2.f)) {
                    tree_collision_pair * tcp = _tree_collision_pairs.add();
                    tcp->obj = cur_obj;
                    tcp->tree = tb->info(j);
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
            btCapsuleShape * t_cap =  new (&tci->shape) btCapsuleShape(g_temp_tree_rad, t.height);
            btCollisionObject * t_col = new (&tci->obj) btCollisionObject();
            t_col->setCollisionShape(t_cap);
            t_col->setWorldTransform(t_trans);
        }
    }

	void discrete_dynamics_world::process_tree_collisions()
    {
        _tree_collision_pairs.for_each([&](tree_collision_pair&  tcp) {

            btCollisionObjectWrapper obj1_wrapper(0, tcp.obj->getCollisionShape(), tcp.obj, tcp.obj->getWorldTransform(), -1, -1);
            btCollisionObjectWrapper obj2_wrapper(0, &tcp.tree->shape, &tcp.tree->obj, tcp.tree->obj.getWorldTransform(), -1, -1);
            btPersistentManifold * manifold = getDispatcher()->getNewManifold(obj1_wrapper.getCollisionObject(), obj2_wrapper.getCollisionObject());
            manifold->clearManifold();
            *_manifolds.add() = manifold;
            btManifoldResult res(&obj1_wrapper, &obj2_wrapper);

            btCollisionAlgorithm * algo = getDispatcher()->findAlgorithm(&obj1_wrapper, &obj2_wrapper,manifold);

            if (algo) {
                algo->processCollision(&obj1_wrapper, &obj2_wrapper, getDispatchInfo(), &res);
            }
        });


		_tree_collision_pairs.clear();
	}

	discrete_dynamics_world::discrete_dynamics_world(btDispatcher * dispatcher, 
		btBroadphaseInterface * pairCache, 
		btConstraintSolver * constraintSolver, 
		btCollisionConfiguration * collisionConfiguration,
        fn_ext_collision ext_collider,
		const void* context)
		: btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
        , _sphere_intersect(ext_collider)
		, _context(context)
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
