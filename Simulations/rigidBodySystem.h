#pragma once
#include "Simulator.h"

//struct rigidBody {
//public:
////	rigidBody()=default;
////	rigidBody(const rigidBody&) = delete;
//
//	Vec3 boxCenter;
//	Vec3 size;
//	float mass;
//	Quat orientation = {0,0,0,1};
//	Vec3 lineerVelocity;
//	Vec3 angularVelocity;
//	Vec3 angularMomentum;
//	Mat4 inverseInertiaTensor;//current inertia tensor
//	Vec3 torq;//q
//	Vec3 totalForce;//F
//	float onedivMass;
//	bool isStatic = false;
//};

struct SphericalCollisionInfo {
	bool isValid;
	Vec3 rb1VelocityChange, rb2VelocityChange;
};


struct rigidBody {
public:
	//	rigidBody()=default;
	//	rigidBody(const rigidBody&) = delete;

	Vec3 center;
	float radius;
	float mass;
	Quat orientation = { 0,0,0,1 };
	Vec3 lineerVelocity;
	Vec3 angularVelocity;
	Vec3 angularMomentum;
	Mat4 inverseInertiaTensor;//current inertia tensor
	Vec3 torq;//q
	Vec3 totalForce;//F
	float onedivMass;
	bool isStatic = false;
};
