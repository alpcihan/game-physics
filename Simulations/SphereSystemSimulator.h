#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "spheresystem.h"
#include "DiffusionSimulator.h"
#include "RigidBodySystemSimulator.h"
#include <unordered_map>


#define NAIVEACC 0
#define GRIDACC 1


class Entity{
public:

	Entity(){}
	~Entity(){}

	void clear() {

		m_rigidBodies.clear();
		m_rigidBodies.shrink_to_fit();

	}

	void addRigidBody(Vec3 position, float radius, int mass, Vec3 initialVelocity=Vec3(0.0)) {

		//m_rigidBodies.push_back(rigidBody);
		rigidBody newRb;
		newRb.center = position;
		newRb.radius = radius;
		newRb.mass = mass;
		newRb.angularVelocity = Vec3(0.0);
		newRb.lineerVelocity = initialVelocity;
		newRb.totalForce = 0;
		newRb.onedivMass = 1.0 / mass;

		// Change interial vals later
		Vec3 sphericalInertiaVals(0.0);
		sphericalInertiaVals.x = (1.0 / 12.0) * mass * (2 * radius * radius);
		sphericalInertiaVals.y = (1.0 / 12.0) * mass * (2 * radius * radius);
		sphericalInertiaVals.z = (1.0 / 12.0) * mass * (2 * radius * radius);

		newRb.inverseInertiaTensor.initScaling(sphericalInertiaVals.x, sphericalInertiaVals.y, sphericalInertiaVals.z); //set inertia tensor values //!!!!!!!!!!!possible error!!!!!!!!!!!!!!!!!!
		newRb.inverseInertiaTensor = newRb.inverseInertiaTensor.inverse();

		m_rigidBodies.push_back(newRb);

	}
	vector<rigidBody> getRigidBody() {
		return m_rigidBodies;
	}

	void updateRigidBodies(const vector<rigidBody>& rigidbodies) {

		m_rigidBodies = rigidbodies;

	}

private:

	vector<rigidBody> m_rigidBodies;

};

/// <summary>
/// 
/// This Simulator is the parent simulator which conatins RigidBodySimulator and DiffusionSimulator
/// 
/// </summary>
class SphereSystemSimulator:public Simulator{
public:
	// Construtors
	SphereSystemSimulator();
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void clearRigidBodies();

	void addTarget(uint32_t n_x, uint32_t n_y);
	void addBullet();
	void setScene();

	void updateEntities(vector<vector<rigidBody>> updatedEntities);
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	
protected:
	//int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	//static std::function<float(float)> m_Kernels[5];
	
	//int m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)

	vector<Entity> m_rigidBodies;
	
	RigidBodySystemSimulator* m_pRigidBodySimulator;
	DiffusionSimulator* m_pDiffusionSimulator;

	uint32_t bulletsEntityIdx;

	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	// SphereSystem * m_pSphereSystemGrid; 

};

#endif