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
		//Mat4 transform=getObject2WorldSpaceMatrix(temp_RigidBodies[i]);
		rb = temp_RigidBodies[i];
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
			if (temp_RigidBodies[i].isStatic) continue;

			temp_RigidBodies[i].totalForce += inputWorldScaled;

			Vec3 pos = norm(inputWorld - temp_RigidBodies[i].center) + temp_RigidBodies[i].center;
			temp_RigidBodies[i].torq += cross(pos, dir*0.01);
		}
	}
}


SphericalCollisionInfo RigidBodySystemSimulator::checkSphericalCollision(rigidBody& rb1, rigidBody& rb2) {

	SphericalCollisionInfo info;

	info.isValid = false;
	info.rb1VelocityChange = 0;
	info.rb2VelocityChange = 0;

	Vec3 centerDiff = rb2.center - rb1.center;
	float euclideanDistance = sqrt(centerDiff.squaredDistanceTo(Vec3(0.0)));


	if (euclideanDistance >= (rb1.radius + rb2.radius))
		return info;
		

	Vec3 direction = centerDiff / euclideanDistance;


	Vec3 velocityDiff = rb2.lineerVelocity - rb1.lineerVelocity;
	float relativeVel = dot(velocityDiff, direction);

	if (relativeVel >= 0)
		return info;

	float s1 = (2 * rb2.mass * relativeVel) / (rb1.mass + rb2.mass);
	float s2 = (relativeVel * (rb2.mass - rb1.mass)) / (rb1.mass + rb2.mass);

	info.isValid = true;

	info.rb1VelocityChange = (direction * s1)/2;
	info.rb2VelocityChange = (direction * (s2 - relativeVel))/2;
	
	return info;
}


void RigidBodySystemSimulator::applyForceOfCollusions(float timestep) {
	// Collusion calculation of entities

	for (uint32_t i = 0; i < rigidBodies.size(); ++i) {						// i represents the first entity to check collusion
		vector<rigidBody>& first_entity = rigidBodies[i];

		// External collusion calculations of first_entity
		for (uint32_t j = i + 1; j < rigidBodies.size(); ++j) {				//j represents the second entity to check collusion
			vector<rigidBody>& second_entity = rigidBodies[j];

			for (uint32_t k = 0; k < rigidBodies[i].size(); ++k) {			//k represents the rigid body elements of the first entity
				for (uint32_t l = 0; l < rigidBodies[j].size(); ++l) {		//l represents the rigid body elements of the second entity
					SphericalCollisionInfo info = checkSphericalCollision(first_entity[k], second_entity[l]);

					if (info.isValid) {
						first_entity[k].lineerVelocity += info.rb1VelocityChange;
						second_entity[l].lineerVelocity += info.rb2VelocityChange;
					}
				}

			}
		}
		

		// Internal collusion calculation of first_entity
		for (uint32_t internal_i = 0; internal_i < rigidBodies[i].size(); ++internal_i) {
			for (uint32_t internal_j = internal_i + 1; internal_j < rigidBodies[i].size(); ++internal_j) {

				SphericalCollisionInfo info = checkSphericalCollision(first_entity[internal_i], first_entity[internal_j]);

				if (info.isValid) {
					first_entity[internal_i].lineerVelocity += info.rb1VelocityChange;
					first_entity[internal_j].lineerVelocity += info.rb2VelocityChange;
				}

			}

		}
	}

}

void RigidBodySystemSimulator::simulateTimestep(float timestep)
{	

	for (uint32_t j = 0; j < rigidBodies.size(); ++j) {
		temp_RigidBodies = rigidBodies[j];

		for (int i = 0; i < rigidBodies[j].size(); ++i) {
			implementEuler(i, timestep);
			updateOrientation(i, timestep);
			updateAngularVelocity(i, timestep);

			temp_RigidBodies[i].totalForce = 0;
			temp_RigidBodies[i].torq = 0;
		}
		
	}
	if (m_updateCallback) {
		m_updateCallback(rigidBodies);
	}
	applyForceOfCollusions(timestep);
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
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return temp_RigidBodies[i].center;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return temp_RigidBodies[i].lineerVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return temp_RigidBodies[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	temp_RigidBodies[i].totalForce += force;
	temp_RigidBodies[i].torq += cross(loc- temp_RigidBodies[i].center, force);
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
	return rigidBodies.size() - 1;
	
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	temp_RigidBodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	temp_RigidBodies[i].lineerVelocity = velocity;
}

Vec3 RigidBodySystemSimulator::getTotalForce(int i)
{
	return temp_RigidBodies[i].totalForce;
}

float RigidBodySystemSimulator::getMass(int i)
{
	return temp_RigidBodies[i].mass;
}

void RigidBodySystemSimulator::applyGravityToAll() {
	
	for (int i = 0; i < this->getNumberOfRigidBodies(); ++i) {
		applyForceOnBody(i, temp_RigidBodies[i].center, temp_RigidBodies[i].mass * f_gravityAcc);
	}
}

void RigidBodySystemSimulator::addEntities(vector<rigidBody>& Entity)
{
	rigidBodies.push_back(Entity);
}

void RigidBodySystemSimulator::implementEuler(int i, float timeStep)
{
	temp_RigidBodies[i].center += timeStep * temp_RigidBodies[i].lineerVelocity;
	temp_RigidBodies[i].lineerVelocity += timeStep * temp_RigidBodies[i].totalForce / temp_RigidBodies[i].mass;

	applyGravityToAll();
}

void RigidBodySystemSimulator::updateOrientation(int i, float timestep)
{
	auto w = temp_RigidBodies[i].angularVelocity;
	Quat w_q = Quat(w.x, w.y, w.z, 0);
	
	temp_RigidBodies[i].orientation += (timestep / 2) * temp_RigidBodies[i].orientation * w_q;
	temp_RigidBodies[i].orientation = temp_RigidBodies[i].orientation.unit();
}

void RigidBodySystemSimulator::updateAngularVelocity(int i,float timestep)
{
	rigidBody& object = temp_RigidBodies[i];

	// update angular momentum
	object.angularMomentum += timestep * object.torq;

	// calculate current inertia tensor
	Mat4 rotation = object.orientation.getRotMat();
	Mat4 rotation_t = Mat4(rotation);
	rotation_t.transpose();
	Mat4 currentInverseInertia = rotation * object.inverseInertiaTensor * rotation_t;

	// update angular velocity
	temp_RigidBodies[i].angularVelocity = currentInverseInertia * object.angularMomentum;
}

Vec3 RigidBodySystemSimulator::getWorldSpaceVelocity(int i,Vec3 loc)
{
	return temp_RigidBodies[i].lineerVelocity + cross(temp_RigidBodies[i].angularVelocity, loc - temp_RigidBodies[i].center);
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
	rigidBodies.clear();

	addRigidBody(Vec3(-0.6, 0, 0), 0.2, 2.0);
	setVelocityOf(0, Vec3(0.3, 0, 0));

	addRigidBody(Vec3(0.6, 0, 0), 0.2, 2.0);
	
}

