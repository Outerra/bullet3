#include "discrete_dynamics_world.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include <ot/glm/coal.h>

#include "ot_terrain_contact_common.h"

#include <comm/timer.h>
#include <comm/dynarray.h>

extern coid::slotalloc<tree_batch> * ptree_cache;


namespace ot {

	void discrete_dynamics_world::internalSingleStepSimulation(btScalar timeStep)
	{
		_frame_count++;

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

		ot_terrain_collision_step_cleanup();
		ot_terrain_collision_step();
		process_acp();

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

	void discrete_dynamics_world::ot_terrain_collision_step()
	{
		ot_terrain_contact_common common_data(0.00f,this,_pb_wrap);
		for (int i = 0; i < m_collisionObjects.size(); i++) {
			_cow_internal.clear();
			_compound_processing_stack.clear();
			btCollisionObject * obj = m_collisionObjects[i];
            btRigidBody * rb = 0;
            if (!obj->isStaticObject()) {
                rb = reinterpret_cast<btRigidBody *>(obj);
            }


			if ((!rb || rb->wantsSleeping()) ||
				(obj->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE &&
					obj->getCollisionShape()->getShapeType() != CAPSULE_SHAPE_PROXYTYPE &&
					!obj->getCollisionShape()->isConvex() &&
					obj->getCollisionShape()->getShapeType() != COMPOUND_SHAPE_PROXYTYPE))
			{
				continue;
			}

			btCollisionObjectWrapper planet_wrapper(0, _planet_body->getCollisionShape(), _planet_body, btTransform(), -1, -1);
			btCollisionObjectWrapper collider_wrapper(0, obj->getCollisionShape(), obj, obj->getWorldTransform(), -1, -1);
			btManifoldResult res(&planet_wrapper, &collider_wrapper);
			btPersistentManifold * manifold;
			manifold = getDispatcher()->getNewManifold(obj, _planet_body);
			manifold->clearManifold();

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
            *_manifolds.add(1) = manifold;

			for (int j = 0; j < _cow_internal.size(); j++) {
				btCollisionObjectWrapper internal_obj_wrapper(_cow_internal[j]._parent,
					_cow_internal[j]._shape,
					_cow_internal[j]._collisionObject,
					_cow_internal[j]._worldTransform,
					_cow_internal[j]._partId,
					_cow_internal[j]._index);

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
					common_data.prepare_sphere_collision(&res, _from, sph->getRadius(), 0.02f);
				}
				else if (internal_obj_wrapper.getCollisionShape()->getShapeType() == CAPSULE_SHAPE_PROXYTYPE) {
					const btCapsuleShape * caps = reinterpret_cast<const btCapsuleShape*>(internal_obj_wrapper.getCollisionShape());
					float cap_rad = float(caps->getRadius());
					float cap_hheight = float(caps->getHalfHeight());
					_rad = cap_rad + cap_hheight + 0.04f;

					btVector3 main_axis = internal_obj_wrapper.getWorldTransform().getBasis().getColumn(1);
					btVector3 p0 = sc + (main_axis * cap_hheight);
					btVector3 p1 = sc - (main_axis * cap_hheight);

					common_data.prepare_capsule_collision(&res, glm::dvec3(p0.x(), p0.y(), p0.z()), glm::dvec3(p1.x(), p1.y(), p1.z()), cap_rad, caps->getMargin());
				}
				else if (internal_obj_wrapper.getCollisionShape()->isConvex()) {
					btTransform t = internal_obj_wrapper.getWorldTransform();
					btQuaternion q = t.getRotation();
					btVector3 p = t.getOrigin();
					glm::quat rot(q.x(), q.y(), q.z(), q.w());
					glm::dvec3 pos(p.x(), p.y(), p.z());
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

                if(!_sphere_intersect(*_planet, _from, _rad, _triangles, _trees))
                    continue;

				if (_triangles.size() > 0) {
					common_data.process_triangle_cache(_triangles);
				}

				int num_contacts = manifold->getNumContacts();

				common_data.process_collision_points();

				if (manifold->getNumContacts() == 0) {
					getDispatcher()->releaseManifold(manifold);
					_manifolds.del(_manifolds.size() - 1, 1);
				}
			}
		}
	}

	void discrete_dynamics_world::ot_terrain_collision_step_cleanup()
	{
		//zatial takto natvrdo
		_manifolds.for_each([&](btPersistentManifold * m) {
			getDispatcher()->releaseManifold(m);
		});

		_manifolds.clear();
	}

	void discrete_dynamics_world::process_acp()
	{
/*
		for (int i = 0; i < _additional_pairs.size(); i++) {
			const raw_collision_pair& cp = _additional_pairs[i];

			btCollisionObjectWrapper obj1_wrapper(0, cp._obj1->getCollisionShape(), cp._obj1, cp._obj1->getWorldTransform(), -1, -1);
			btCollisionObjectWrapper obj2_wrapper(0, cp._obj2->getCollisionShape(), cp._obj2, cp._obj2->getWorldTransform(), -1, -1);
			btManifoldResult res(&obj1_wrapper, &obj2_wrapper);
			btPersistentManifold * manifold = getDispatcher()->getNewManifold(cp._obj1, cp._obj2);
			manifold->clearManifold();
			manifold->refreshContactPoints(cp._obj1->getWorldTransform(), cp._obj2->getWorldTransform());
			res.setPersistentManifold(manifold);
			*_manifolds.add(1) = manifold;

			btCollisionAlgorithm * algo =  getDispatcher()->findAlgorithm(&obj1_wrapper, &obj2_wrapper, manifold);

			if (algo) {
				algo->processCollision(&obj1_wrapper,&obj2_wrapper,getDispatchInfo(),&res);
			}
		}*/

		_additional_pairs.clear();
		/*_additional_pairs.for_each([&](raw_collision_pair cp) {
		
		});*/
	}

	discrete_dynamics_world::discrete_dynamics_world(btDispatcher * dispatcher, 
		btBroadphaseInterface * pairCache, 
		btConstraintSolver * constraintSolver, 
		btCollisionConfiguration * collisionConfiguration,
        fn_ext_collision ext_collider,
		const planet_qtree* context)
		: btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
        , _sphere_intersect(ext_collider)
		, _planet(context)
		//, _frame_count(0)
	{
		btTriangleShape * ts = new btTriangleShape();
		ts->setMargin(0.0f);
		_planet_body = new btRigidBody(0,0,ts);
		_planet_body->setCollisionFlags(0);
		_planet_body->setRestitution(1.0f);
		//_tree_cache.reserve(1024);
		//ptree_cache = &_tree_cache;
		_cow_internal.reserve(128,false);
		_compound_processing_stack.reserve(128, false);
		//_pb_wrap = new btCollisionObjectWrapper(0,ts, _planet_body,btTransform(),-1,-1);
	}


}// end namespace ot
