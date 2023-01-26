#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2


class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testcase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timestep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	//void addRigidBody(Vec3 position, Vec3 size, int mass);
	size_t addRigidBody(Vec3 position, float radius, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	Vec3 getTotalForce(int i);
	float getMass(int i);
	void applyGravityToAll();
	void addEntities(vector<rigidBody>& entities);
	void clearRigidBodies();

	void implementEuler(int i, float timeStep);
	void updateOrientation(int i, float timestep);
	void updateAngularVelocity(int i,float timestep);
	Vec3 getWorldSpaceVelocity(int i, Vec3 loc);
	Mat4 getObject2WorldSpaceMatrix(const rigidBody& object);
	void applyForceOfCollusions(float timestep);

	SphericalCollisionInfo checkSphericalCollision(rigidBody& rb1, rigidBody& rb2);

	/*void setDemo1();
	void setDemo2();
	void setDemo3();
	void setDemo4();*/
	void setProjectDemo();

	using UpdateCallback = std::function<void(const std::vector<vector<rigidBody>>&)>;
	void setUpdateCallback(UpdateCallback callback) { m_updateCallback = callback; }

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	vector<vector<rigidBody>> rigidBodies; //TODO: RigidBodies can be given from outside of the simulator and functions can get the rigid bodies as input
	vector<rigidBody> temp_RigidBodies;
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	Vec3 f_gravityAcc = Vec3(0, -0.2, 0);

	UpdateCallback m_updateCallback;
	};
#endif