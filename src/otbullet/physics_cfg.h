#pragma once

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

class rigid_body_constraint;

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
