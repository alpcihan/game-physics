#pragma once
#include <vector>
#include "Simulator.h"

using std::vector;

struct rigidBody {
	Vec3 boxCenter;
	Vec3 size;
	float mass;
	Quat orientation = {0,0,0,1};
	Vec3 lineerVelocity;
	Vec3 angularVelocity;
	Vec3 angularMomentum;
	Mat4 inverseInertiaTensor;//current inertia tensor
	Vec3 torq;//q
	Vec3 totalForce;//F
};
