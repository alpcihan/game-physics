#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"



RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	this->m_iTestCase = 0;
	
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (this->m_iTestCase) {
	case 0:
		//Demo1
		break;
	case 1:
		//Demo2
		break;
	case 2:
		//Demo3
		break;
	case 3:
		//Demo4
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

Mat4 RigidBodySystemSimulator::getObject2WorldSpaceMatrix(const rigidBody& object) {

	Mat4 transform, scale, translate, rotation;
	scale.initScaling(object.size.x, object.size.y, object.size.z);
	translate.initTranslation(object.boxCenter.x, object.boxCenter.y, object.boxCenter.z);
	rotation = object.orientation.getRotMat();
	transform = scale * rotation * translate;

	return transform;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

	for (unsigned int i=0; i < this->getNumberOfRigidBodies(); ++i) {

		Mat4 transform=getObject2WorldSpaceMatrix(rigidBodies[i]);
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(transform);
	}

}

void RigidBodySystemSimulator::notifyCaseChanged(int testcase)
{
	switch (testcase) {
	case 0:
		//Demo1
		std::cout << "Demo1\n";
		setDemo1();
		break;
	case 1:
		//Demo2
		std::cout << "Demo2\n";
		setDemo2();
		break;
	case 2:
		//Demo3
		std::cout << "Demo3\n";
		setDemo3();
		break;
	case 3:
		//Demo4
		std::cout << "Demo4\n";
		setDemo4();
		break;
	default:
		break;
	}

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);

		Vec3 inputstartView = Vec3((float)m_trackmouse.x, (float)m_trackmouse.y, 0);
		Vec3 inputstartWorld = worldViewInv.transformVector(inputstartView);
		// find a proper scale!
		float inputScale = 0.0001f;
		inputWorld = inputWorld * inputScale;

		for (int i = 0; i < this->getNumberOfRigidBodies(); ++i) {

			this->applyForceOnBody(i, inputstartWorld, inputWorld);

		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timestep)
{

	for (unsigned int i = 0; i < this->getNumberOfRigidBodies(); ++i) {
		implementEuler(i, timestep);
		updateOrientation(i, timestep);
		updateAngularVelocity(i, timestep);

		rigidBodies[i].totalForce = 0;
		rigidBodies[i].torq = 0;
	}

	for (int i = 0; i < this->getNumberOfRigidBodies()-1; ++i) {
		for (unsigned int j = i+1; j < this->getNumberOfRigidBodies(); ++j) {
			auto info =  checkCollisionSAT(getObject2WorldSpaceMatrix(rigidBodies[i]), getObject2WorldSpaceMatrix(rigidBodies[j]));
			if(info.isValid)std::cout << "collided!!!!!!!!!" << "\n";
		}
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidBodies[i].boxCenter;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidBodies[i].lineerVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidBodies[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	rigidBodies[i].totalForce += force;
	rigidBodies[i].torq += cross(loc- rigidBodies[i].boxCenter, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidBody newElement;
	newElement.boxCenter = position;
	newElement.size = size;
	newElement.mass = mass;
	newElement.angularVelocity = Vec3(0.0);
	newElement.lineerVelocity = Vec3(0.0);
	newElement.totalForce = 0;

	Vec3 cuboidInertiaVals(0.0);
	cuboidInertiaVals.x = (1.0 / 12.0) * mass * (size.y * size.y + size.z * size.z);
	cuboidInertiaVals.y = (1.0 / 12.0) * mass * (size.x * size.x + size.z * size.z);
	cuboidInertiaVals.z = (1.0 / 12.0) * mass * (size.x * size.x + size.y * size.y);

	newElement.inverseInertiaTensor.initScaling(cuboidInertiaVals.x, cuboidInertiaVals.y, cuboidInertiaVals.z); //set inertia tensor values //!!!!!!!!!!!possible error!!!!!!!!!!!!!!!!!!
	newElement.inverseInertiaTensor = newElement.inverseInertiaTensor.inverse();

	rigidBodies.push_back(newElement);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodies[i].lineerVelocity = velocity;
}

Vec3 RigidBodySystemSimulator::getTotalForce(int i)
{
	return rigidBodies[i].totalForce;
}

float RigidBodySystemSimulator::getMass(int i)
{
	return rigidBodies[i].mass;
}

void RigidBodySystemSimulator::implementEuler(int i, float timeStep)
{
	rigidBodies[i].boxCenter += timeStep * rigidBodies[i].lineerVelocity;
	rigidBodies[i].lineerVelocity += timeStep * rigidBodies[i].totalForce / rigidBodies[i].mass;
}

void RigidBodySystemSimulator::updateOrientation(int i, float timestep)
{
	auto w = rigidBodies[i].angularVelocity;
	Quat w_q = Quat(w.x, w.y, w.z, 0);
	
	rigidBodies[i].orientation += (timestep / 2) * rigidBodies[i].orientation * w_q;
	rigidBodies[i].orientation = rigidBodies[i].orientation.unit();
}

void RigidBodySystemSimulator::updateAngularVelocity(int i,float timestep)
{
	rigidBody& object = rigidBodies[i];

	// update angular momentum
	object.angularMomentum += timestep * object.torq;

	// calculate current inertia tensor
	Mat4 rotation = object.orientation.getRotMat();
	Mat4 rotation_t = Mat4(rotation);
	rotation_t.transpose();
	Mat4 currentInverseInertia = rotation * object.inverseInertiaTensor * rotation_t;

	// update angular velocity
	rigidBodies[i].angularVelocity = currentInverseInertia * object.angularMomentum;
}

Vec3 RigidBodySystemSimulator::getWorldSpaceVelocity(int i,Vec3 loc)
{
	return rigidBodies[i].lineerVelocity + cross(rigidBodies[i].angularVelocity, loc - rigidBodies[i].boxCenter);
}

void RigidBodySystemSimulator::setDemo1()
{
	rigidBodies.clear();

	addRigidBody(Vec3(0.0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5f));
	applyForceOnBody(0,Vec3(0.3,0.5,0.25),Vec3(1,1,0));

	simulateTimestep(2.0);
	std::cout << getWorldSpaceVelocity(0, Vec3(-0.3, -0.5, -0.25))<<"\n";

	rigidBodies.clear();
}

void RigidBodySystemSimulator::setDemo2()
{
	rigidBodies.clear();

	addRigidBody(Vec3(0.0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5f));
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
}
void RigidBodySystemSimulator::setDemo3()
{
	rigidBodies.clear();

	addRigidBody(Vec3(0.0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5f));
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(10, 1, 0));

	addRigidBody(Vec3(1.0,0.5,0.0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(1, Quat(Vec3(0, 0, 1), M_PI * 0.5f));
	applyForceOnBody(1, Vec3(0.3, 0.5, 0.25), Vec3(-10, 1, 0));
	

}
void RigidBodySystemSimulator::setDemo4()
{
}