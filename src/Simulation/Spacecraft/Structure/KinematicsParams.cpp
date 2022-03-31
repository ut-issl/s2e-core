#include "KinematicsParams.h"

KinematicsParams::KinematicsParams(Vector<3> cg_b, double mass, Matrix<3, 3> inertia_tensor)
    : cg_b_(cg_b), mass_(mass), inertia_tensor_(inertia_tensor) {}