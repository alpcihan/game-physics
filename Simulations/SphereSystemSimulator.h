#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "spheresystem.h"
#include "DiffusionSimulator.h"
#include "RigidBodySystemSimulator.h"
#include <unordered_map>

#define NAIVEACC 0
#define GRIDACC 1

enum EntityType {
	DEFAULT,
	BULLET,
	TARGET
};

class Entity{
public:

	Entity():m_EntityType(EntityType::DEFAULT) {}
	Entity(EntityType type):m_EntityType(type){}
	~Entity(){}

	void clear() {
		m_rigidBodies.clear();



		//m_rigidBodies.shrink_to_fit();
	}

	void draw(DrawingUtilitiesClass* DUC) {

		for (size_t i = 0; i < m_rigidBodies.size(); ++i) {
			DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
			DUC->drawSphere(m_rigidBodies[i].center, Vec3(m_rigidBodies[i].radius));
		}
	}

	size_t addRigidBody(Vec3 position, float radius, int mass, bool isStatic=false, Vec3 initialVelocity=Vec3(0.0)) {

		//m_rigidBodies.push_back(rigidBody);
		rigidBody newRb;
		newRb.center = position;
		newRb.radius = radius;
		newRb.mass = mass;
		newRb.angularVelocity = Vec3(0.0);
		newRb.isStatic = isStatic;
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

		return m_rigidBodies.size() - 1;

	}

	vector<rigidBody>& getRigidBodies() {//make it get rigid bodies
		return m_rigidBodies;
	}

	rigidBody& getRigidBody(size_t idx) {
		return m_rigidBodies[idx];
	}

	size_t getNumberOfRigidBodies() {
		return m_rigidBodies.size();
	}

	void updateRigidBodies(vector<rigidBody>& rigidbodies) { //TODO: Can be applied by move instead of '='
		m_rigidBodies = rigidbodies;//.insert(m_rigidBodies.begin(),rigidbodies.begin(), rigidbodies.end());
	}
	EntityType getEntityType() { return m_EntityType; } //if needed

private:

	vector<rigidBody> m_rigidBodies;
	EntityType m_EntityType;//if needed
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

	void addTarget(uint16_t n_x, uint16_t n_y);
	void addBullet();
	void setScene();

	void updateEntities(vector<rigidBody>& updatedEntities);
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext); //TODO: Draw bullet and target seperately 
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	void rotateCameraBy(Vec3 rotation);
	
protected:
	//int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	//static std::function<float(float)> m_Kernels[5];
	//int m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)

	std::unordered_map<EntityType,Entity> m_entities;
	
	RigidBodySystemSimulator* m_pRigidBodySimulator;
	DiffusionSimulator* m_pDiffusionSimulator;

	Grid* m_targetGrid;
	uint16_t grid_w,grid_h;

	Point2D m_oldmouse;
	Point2D m_mouse;
};

#endif