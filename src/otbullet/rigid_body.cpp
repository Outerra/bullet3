
#include "otbullet.hpp"
#include "physics_cfg.h"

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btDefaultMotionState.h>

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "discrete_dynamics_world.h"

//static btRigidBody _fixedObject( btRigidBody::btRigidBodyConstructionInfo(0,0,0) );

////////////////////////////////////////////////////////////////////////////////
class rigid_body_constraint : public btTypedConstraint
{
    bt::constraint_info* _info;

public:

    rigid_body_constraint(bt::constraint_info* ci, btRigidBody* rb)
        : btTypedConstraint(CONTACT_CONSTRAINT_TYPE, *rb)
        , _info(ci)
    {}

    virtual ~rigid_body_constraint()
    {}

    virtual void getInfo1( btTypedConstraint::btConstraintInfo1* info ) {
        return _info->getInfo1(info);
    }

    virtual void getInfo2( btTypedConstraint::btConstraintInfo2* info ) {
        return _info->getInfo2(info);
    }

    virtual	void setParam(int num, btScalar value, int axis = -1) {}
    virtual	btScalar getParam(int num, int axis = -1) const { return 0.0; }
};


////////////////////////////////////////////////////////////////////////////////
btRigidBody* physics::fixed_object()
{
    return &btTypedConstraint::getFixedBody();
}

////////////////////////////////////////////////////////////////////////////////
btRigidBody* physics::create_rigid_body( float mass, btCollisionShape* shape, void* usr1, void* usr2 )
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool dynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if(dynamic)
        shape->calculateLocalInertia(mass, localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

    //#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    btDefaultMotionState* ms = new btDefaultMotionState(startTransform);
#else
    btDefaultMotionState* ms = 0;
#endif

    btRigidBody::btRigidBodyConstructionInfo rbinfo(mass, ms, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbinfo);

    body->setUserPointer(usr1);
    body->m_userDataExt = usr2;

    return body;
}

////////////////////////////////////////////////////////////////////////////////
void physics::destroy_rigid_body( btRigidBody*& obj )
{
    delete obj;
    obj = 0;
}

////////////////////////////////////////////////////////////////////////////////
bool physics::add_rigid_body( btRigidBody* obj, unsigned int group, unsigned int mask,
    btActionInterface* action, bt::constraint_info* constraint )
{
//    if(action)
//		obj->setActivationState(DISABLE_DEACTIVATION);

    if (!_world->addRigidBody(obj, group, mask)) {
        return false;
    };

    if(action)
        _world->addAction(action);

    if(constraint) {
        rigid_body_constraint* btcon = new rigid_body_constraint(constraint, obj);
        constraint->_constraint = btcon;

        _world->addConstraint(btcon);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
void physics::remove_rigid_body( btRigidBody* obj, btActionInterface* action, bt::constraint_info* constraint )
{
    if(action)
        _world->removeAction(action);
    if(constraint) {
        rigid_body_constraint* btcon = constraint->_constraint;
        _world->removeConstraint(btcon);

        delete btcon;
    }

    const static btVector3 zero(0.0, 0.0, 0.0);
    obj->setAngularVelocity(zero);
    obj->setLinearVelocity(zero);

    _world->removeRigidBody(obj);
}

////////////////////////////////////////////////////////////////////////////////
void physics::pause_rigid_body( btRigidBody* obj, bool pause )
{
    obj->forceActivationState(pause ? DISABLE_SIMULATION : ACTIVE_TAG);

    if(pause) {
        btVector3 v(0,0,0);
        obj->setLinearVelocity(v);
        obj->setAngularVelocity(v);
    }
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_rigid_body_mass( btRigidBody* obj, float mass, const float inertia[3] )
{
    obj->setMassProps(mass, btVector3(inertia[0], inertia[1], inertia[2]));
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_rigid_body_gravity( btRigidBody* obj, const double gravity[3] )
{
    obj->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_rigid_body_transform( btRigidBody* obj, const btTransform& tr, const double gravity[3] )
{
    obj->setCenterOfMassTransform(tr);
    obj->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
    
}

////////////////////////////////////////////////////////////////////////////////
void physics::predict_rigid_body_transform( btRigidBody* obj, double dt, btTransform& tr )
{
    obj->predictIntegratedTransform(dt, tr);
}

////////////////////////////////////////////////////////////////////////////////
float physics::get_angular_factor(const btRigidBody* obj)
{
    return float(obj->getAngularFactor().x());
}

////////////////////////////////////////////////////////////////////////////////
void physics::set_angular_factor(btRigidBody* obj, float factor)
{
    obj->setAngularFactor(btVector3(factor, factor, factor));
}