#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	this->m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	//return "Demo1,Demo2,Demo3,Demo4";
	return "Demo0";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (this->m_iTestCase) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::reset()
{
	clearRigidBodies();
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

Mat4 RigidBodySystemSimulator::getObject2WorldSpaceMatrix(const rigidBody& object) {

	Mat4 transform, scale, translate, rotation;
	scale.initScaling(object.radius);
	translate.initTranslation(object.center.x, object.center.y, object.center.z);
	rotation = object.orientation.getRotMat();
	transform = scale * rotation * translate;

	return transform;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	rigidBody rb;
	for ( int i=0; i < this->getNumberOfRigidBodies(); ++i) {
		//Mat4 transform=getObject2WorldSpaceMatrix(m_rigidBodies[i]);
		rb = m_rigidBodies[i];
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		//DUC->drawRigidBody(transform);
		
		DUC->drawSphere(rb.center, Vec3(rb.radius));
	}

}

void RigidBodySystemSimulator::notifyCaseChanged(int testcase)
{
	switch (testcase) {
	case 0:
		std::cout << "Demo0\n";
		setProjectDemo();
		break;
	//case 1:
	//	//Demo1
	//	std::cout << "Demo1\n";
	//	setDemo1();
	//	break;
	//case 2:
	//	//Demo2
	//	std::cout << "Demo2\n";
	//	setDemo2();
	//	break;
	//case 3:
	//	//Demo3
	//	std::cout << "Demo3\n";
	//	setDemo3();
	//	break;
	//case 4:
	//	//Demo4
	//	std::cout << "Demo4\n";
	//	setDemo4();
	//	break;
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
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		
		// find a proper scale!
		float inputScale = 0.01f;
		Vec3 inputWorldScaled = inputWorld * inputScale;
		inputWorldScaled *= Vec3(1, -1, 1);

		Vec3 dir = norm(inputWorldScaled);
		
		for (int i = 0; i < this->getNumberOfRigidBodies(); ++i) {
			if (m_rigidBodies[i].isStatic) continue;

			m_rigidBodies[i].totalForce += inputWorldScaled;

			Vec3 pos = norm(inputWorld - m_rigidBodies[i].center) + m_rigidBodies[i].center;
			m_rigidBodies[i].torq += cross(pos, dir*0.01);
		}
	}
}


//SphericalCollisionInfo RigidBodySystemSimulator::checkSphericalCollision(rigidBody& rb1, rigidBody& rb2) {
//
//	SphericalCollisionInfo info;
//
//	info.isValid = false;
//	info.rb1VelocityChange = 0;
//	info.rb2VelocityChange = 0;
//
//	if (rb1.isStatic && rb2.isStatic)
//		return info;
//
//
//	Vec3 centerDiff = rb2.center - rb1.center;
//	float euclideanDistance = sqrt(centerDiff.squaredDistanceTo(Vec3(0.0)));
//
//
//	if (euclideanDistance >= (rb1.radius + rb2.radius))
//		return info;
//		
//
//	Vec3 direction = centerDiff / euclideanDistance;
//
//
//	Vec3 velocityDiff = rb2.lineerVelocity - rb1.lineerVelocity;
//	float relativeVel = dot(velocityDiff, direction);
//
//	if (relativeVel >= 0)
//		return info;
//
//	float s1 = (2 * rb2.mass * relativeVel) / (rb1.mass + rb2.mass);
//	float s2 = (relativeVel * (rb2.mass - rb1.mass)) / (rb1.mass + rb2.mass);
//
//	info.isValid = true;
//
//	if (!rb1.isStatic)
//		info.rb1VelocityChange = (direction * s1)/2;
//
//	if (!rb2.isStatic)
//		info.rb2VelocityChange = (direction * (s2 - relativeVel))/2;
//	
//	return info;
//}

SphericalCollisionInfo RigidBodySystemSimulator::checkSphericalCollision(rigidBody& rb1, rigidBody& rb2) {

	SphericalCollisionInfo info;

	info.isValid = false;
	info.rb1VelocityChange = 0;
	info.rb2VelocityChange = 0;

	if (rb1.isStatic && rb2.isStatic)
		return info;


	Vec3 centerDiff = rb2.center - rb1.center;
	float euclideanDistance = sqrt(centerDiff.squaredDistanceTo(Vec3(0.0)));


	if (euclideanDistance >= (rb1.radius + rb2.radius))
		return info;


	Vec3 direction = centerDiff / euclideanDistance;


	Vec3 velocityDiff = rb2.lineerVelocity - rb1.lineerVelocity;
	float relativeVel = dot(velocityDiff, direction);

	if (relativeVel >= 0)
		return info;

	float bounciness = 0.6f; // bounciness factor

	info.isValid = true;

	if (rb1.isStatic && !rb2.isStatic) {
		float j = -(1 + bounciness) * relativeVel / ((1 / rb2.mass));
		Vec3 impulse = j * direction;
		info.rb2VelocityChange = impulse / rb2.mass;
		return info;
	}

	if (!rb1.isStatic && rb2.isStatic) {
		float j = -(1 + bounciness) * relativeVel / ((1 / rb1.mass));
		Vec3 impulse = j * direction;
		info.rb1VelocityChange = impulse / rb1.mass;
		return info;
	}

	
	float j = -(1 + bounciness) * relativeVel / ((1 / rb1.mass) + (1 / rb2.mass));
	Vec3 impulse = j * direction;
	info.rb1VelocityChange = impulse / rb1.mass;
	info.rb2VelocityChange = -impulse / rb2.mass;
	return info;


	//SphericalCollisionInfo info;
	////check if the two spheres are intersecting
	////float distance = (rb1.center - rb2.center).magnitude();
	//Vec3 centerDiff = rb2.center - rb1.center;
	//float distance = sqrt(centerDiff.squaredDistanceTo(Vec3(0.0)));

	//float sumRadius = rb1.radius + rb2.radius;
	//if (distance < sumRadius) {
	//	info.isValid = true;
	//	if (rb1.isStatic && rb2.isStatic)
	//		return info;

	//	Vec3 relativeVelocity = rb2.lineerVelocity - rb1.lineerVelocity;
	//	float velocityAlongNormal = dot(relativeVelocity, (rb1.center - rb2.center)) / distance;
	//	if (velocityAlongNormal > 0) {
	//		//spheres are moving away from each other, no collision
	//		return info;
	//	}

	//	float impulseScalar = 2.0f * rb1.onedivMass * rb2.onedivMass / (rb1.onedivMass + rb2.onedivMass) * velocityAlongNormal;
	//	Vec3 impulse = (rb1.center - rb2.center) * impulseScalar;

	//	if (!rb1.isStatic)
	//		info.rb1VelocityChange = impulse * rb1.onedivMass;
	//	if (!rb2.isStatic)
	//		info.rb2VelocityChange = -impulse * rb2.onedivMass;
	//}

	//return info;

}



void RigidBodySystemSimulator::applyForceOfCollusions(float timestep) {
	// Collusion calculation of rigid bodies

	for (size_t k = 0; k < m_rigidBodies.size(); ++k) {		
		for (size_t l = k+1; l < m_rigidBodies.size(); ++l) {	
			SphericalCollisionInfo info = checkSphericalCollision(m_rigidBodies[k], m_rigidBodies[l]);

			if (info.isValid) {
				m_rigidBodies[k].lineerVelocity += info.rb1VelocityChange;
				m_rigidBodies[l].lineerVelocity += info.rb2VelocityChange;
			}
		}

	}
}

void RigidBodySystemSimulator::simulateTimestep(float timestep)
{	

	for (int i = 0; i < m_rigidBodies.size(); ++i) {
		implementEuler(i, timestep);
		//applyGravityToAll();
		updateOrientation(i, timestep);
		updateAngularVelocity(i, timestep);

		m_rigidBodies[i].totalForce = 0;
		m_rigidBodies[i].torq = 0;
	}

	applyForceOfCollusions(timestep);
	if (m_updateCallback) {
		m_updateCallback(m_rigidBodies);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

//void RigidBodySystemSimulator::onClick(int x, int y)
//{
//	m_trackmouse.x = x;
//	m_trackmouse.y = y;
//
//	Vec3 cameraPos = Vec3(DUC->g_camera.GetEyePt());
//
//
//	Vec3 cameraFrontVec = - cameraPos / sqrt(cameraPos.squaredDistanceTo(Vec3(0.0)));
//	Vec3 bulletPosition = cameraPos + 0.7 * cameraFrontVec;
//	Vec3 bulletVelocity = 7 * cameraFrontVec;
//	
//	//std::cout << Vec3() << std::endl;
//	
//
//	int bulletIdx = addRigidBody(bulletPosition, 0.07, 2.0);
//	std::cout << bulletIdx;
//	setVelocityOf(bulletIdx, bulletVelocity);
//}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_rigidBodies[i].center;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_rigidBodies[i].lineerVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_rigidBodies[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_rigidBodies[i].totalForce += force;
	m_rigidBodies[i].torq += cross(loc- m_rigidBodies[i].center, force);
}

//void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
//{
//	rigidBody newElement;
//	newElement.boxCenter = position;
//	newElement.size = size;
//	newElement.mass = mass;
//	newElement.angularVelocity = Vec3(0.0);
//	newElement.lineerVelocity = Vec3(0.0);
//	newElement.totalForce = 0;
//	newElement.onedivMass = 1.0 / mass;
//
//	Vec3 cuboidInertiaVals(0.0);
//	cuboidInertiaVals.x = (1.0 / 12.0) * mass * (size.y * size.y + size.z * size.z);
//	cuboidInertiaVals.y = (1.0 / 12.0) * mass * (size.x * size.x + size.z * size.z);
//	cuboidInertiaVals.z = (1.0 / 12.0) * mass * (size.x * size.x + size.y * size.y);
//
//	newElement.inverseInertiaTensor.initScaling(cuboidInertiaVals.x, cuboidInertiaVals.y, cuboidInertiaVals.z); //set inertia tensor values //!!!!!!!!!!!possible error!!!!!!!!!!!!!!!!!!
//	newElement.inverseInertiaTensor = newElement.inverseInertiaTensor.inverse();
//
//	rigidBodies.push_back(newElement);
//}

size_t RigidBodySystemSimulator::addRigidBody(Vec3 position, float radius, int mass)
{
	rigidBody newRb;
	newRb.center = position;
	newRb.radius = radius;
	newRb.mass = mass;
	newRb.angularVelocity = Vec3(0.0);
	newRb.lineerVelocity = Vec3(0.0);
	newRb.totalForce = 0;
	newRb.onedivMass = 1.0 / mass;

	// Change interial vals later
	Vec3 sphericalInertiaVals(0.0);
	sphericalInertiaVals.x = (1.0 / 12.0) * mass * (2 * radius * radius);
	sphericalInertiaVals.y = (1.0 / 12.0) * mass * (2 * radius * radius);
	sphericalInertiaVals.z = (1.0 / 12.0) * mass * (2 * radius * radius);

	newRb.inverseInertiaTensor.initScaling(sphericalInertiaVals.x, sphericalInertiaVals.y, sphericalInertiaVals.z); //set inertia tensor values //!!!!!!!!!!!possible error!!!!!!!!!!!!!!!!!!
	newRb.inverseInertiaTensor = newRb.inverseInertiaTensor.inverse();

	//rigidBodies.push_back(newRb); //the rigid bodies are addded in the spheresystemsimulator
	return m_rigidBodies.size() - 1;
	
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_rigidBodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_rigidBodies[i].lineerVelocity = velocity;
}

Vec3 RigidBodySystemSimulator::getTotalForce(int i)
{
	return m_rigidBodies[i].totalForce;
}

float RigidBodySystemSimulator::getMass(int i)
{
	return m_rigidBodies[i].mass;
}

void RigidBodySystemSimulator::applyGravityToAll() {
	
	for (size_t i = 0; i < this->getNumberOfRigidBodies(); ++i) {
		applyForceOnBody(i, m_rigidBodies[i].center, m_rigidBodies[i].mass * f_gravityAcc);
	}
}

void RigidBodySystemSimulator::addEntities(const vector<rigidBody>& Entity)
{
	m_rigidBodies.insert(m_rigidBodies.end(),Entity.begin(),Entity.end());
}

void RigidBodySystemSimulator::addEntity(const rigidBody& entity)
{
	m_rigidBodies.push_back(entity);
}


void RigidBodySystemSimulator::clearRigidBodies()
{
	// Clear the elements of the vector 
	m_rigidBodies.clear();
}

void RigidBodySystemSimulator::implementEuler(int i, float timeStep)
{
	m_rigidBodies[i].center += timeStep * m_rigidBodies[i].lineerVelocity;
	m_rigidBodies[i].lineerVelocity += timeStep * m_rigidBodies[i].totalForce / m_rigidBodies[i].mass;
}

void RigidBodySystemSimulator::updateOrientation(int i, float timestep)
{
	auto w = m_rigidBodies[i].angularVelocity;
	Quat w_q = Quat(w.x, w.y, w.z, 0);
	
	m_rigidBodies[i].orientation += (timestep / 2) * m_rigidBodies[i].orientation * w_q;
	m_rigidBodies[i].orientation = m_rigidBodies[i].orientation.unit();
}

void RigidBodySystemSimulator::updateAngularVelocity(int i,float timestep)
{
	rigidBody& object = m_rigidBodies[i];

	// update angular momentum
	object.angularMomentum += timestep * object.torq;

	// calculate current inertia tensor
	Mat4 rotation = object.orientation.getRotMat();
	Mat4 rotation_t = Mat4(rotation);
	rotation_t.transpose();
	Mat4 currentInverseInertia = rotation * object.inverseInertiaTensor * rotation_t;

	// update angular velocity
	m_rigidBodies[i].angularVelocity = currentInverseInertia * object.angularMomentum;
}

Vec3 RigidBodySystemSimulator::getWorldSpaceVelocity(int i,Vec3 loc)
{
	return m_rigidBodies[i].lineerVelocity + cross(m_rigidBodies[i].angularVelocity, loc - m_rigidBodies[i].center);
}

//void RigidBodySystemSimulator::setDemo1()
//{
//	rigidBodies.clear(); 
//
//	addRigidBody(Vec3(0.0), Vec3(1, 0.6, 0.5), 2);
//	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5f));
//	applyForceOnBody(0,Vec3(0.3,0.5,0.25),Vec3(1,1,0));
//
//	simulateTimestep(2.0);
//}
//
//void RigidBodySystemSimulator::setDemo2()
//{
//	rigidBodies.clear();
//
//	addRigidBody(Vec3(0.0), Vec3(1, 0.6, 0.5), 2);
//	setOrientationOf(0, Quat(Vec3(0.1, 0.5, 0.3), M_PI * 0.275f));
//	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
//}
//void RigidBodySystemSimulator::setDemo3()
//{
//	rigidBodies.clear();
//
//	addRigidBody(Vec3(0.25, 1, 0), Vec3(1.0, 0.6, 0.5), 2.0);
//	addRigidBody(Vec3(-0.30, -1, 0), Vec3(1.0, 0.6, 0.5), 2.0);
//
//	setVelocityOf(0, Vec3(0, -1, 0));
//	setVelocityOf(1, Vec3(0, 1, 0));
//
//	setOrientationOf(1, Quat(Vec3(0, 1, 0), M_PI * .157));
//
//}
//void RigidBodySystemSimulator::setDemo4()
//{
//	rigidBodies.clear();
//
//	addRigidBody(Vec3(0,0,0), Vec3(1.0,0.6,0.5), 2.0);
//
//	addRigidBody(Vec3(2,2,2), Vec3(1.0,0.6,0.5), 2.0);
//	setVelocityOf(1, Vec3(-1, -1, -1));
//	setOrientationOf(1, Quat(Vec3(0.8, 0.7, 0), M_PI * .35));
//
//	addRigidBody(Vec3(3,0,0), Vec3(1.0,0.6,0.5), 2.0);
//	setVelocityOf(2, Vec3(-1, 0, 0));
//	setOrientationOf(2, Quat(Vec3(1, 0, 1), M_PI * .25));
//
//	addRigidBody(Vec3(-2, -0.5, -2), Vec3(1.0,0.6,0.5), 2.0);
//	setVelocityOf(3, Vec3(1, 0.1, 1));
//	setOrientationOf(3, Quat(Vec3(0, 1, 1), M_PI * .4));
//
//	addRigidBody(Vec3(0, -1, 0), Vec3(5.0, 0.25, 5.0), 2.0);
//	rigidBodies[4].isStatic = true;
//
//	addRigidBody(Vec3(-2.5, 1.5, 0), Vec3(0.25, 5, 5.0), 2.0);
//	rigidBodies[5].isStatic = true;
//}

void RigidBodySystemSimulator::setProjectDemo()
{
	std::cout << "Project Demo!\n";
	clearRigidBodies();

	addRigidBody(Vec3(-0.6, 0, 0), 0.2, 2.0);
	setVelocityOf(0, Vec3(0.3, 0, 0));
	addRigidBody(Vec3(0.6, 0, 0), 0.2, 2.0);
	
}

