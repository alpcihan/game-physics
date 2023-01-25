#include "SphereSystemSimulator.h"

//std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
//	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
//	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
//	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
//	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
//	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
//};

// SphereSystemSimulator member functions

SphereSystemSimulator::SphereSystemSimulator()
{
	m_pDiffusionSimulator = new DiffusionSimulator();
	m_pRigidBodySimulator = new RigidBodySystemSimulator();

	m_pRigidBodySimulator->setUpdateCallback(std::bind(&SphereSystemSimulator::updateEntities, this, std::placeholders::_1));
	setScene();

	/*Entity bulletsEntity;
	m_rigidBodies.push_back(bulletsEntity);*/
}

const char* SphereSystemSimulator::getTestCasesStr()
{
	return nullptr;
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void SphereSystemSimulator::reset()
{
	m_pRigidBodySimulator->reset();
	//TODO: write here 
}

void SphereSystemSimulator::addTarget(uint32_t n_x, uint32_t n_y)
{
	Entity targetEntity;
	float scale = 0.1f;
	for (uint32_t i = 0; i < n_x; ++i) {
		for (uint32_t j = 0; j < n_y; ++j) {

			float x = (i - n_x * 0.5) * scale, y = (j- n_y* 0.5) * scale;
			targetEntity.addRigidBody(Vec3(x,y,0.0), 0.1, 2);
		}
	}

	m_rigidBodies.push_back(targetEntity);
}


void SphereSystemSimulator::addBullet()
{
	Vec3 cameraPos = Vec3(0.0, -3.0, 0.0);//Vec3(DUC->g_camera.GetEyePt());//IF we call from addBullet from constructer the DUC is not initialised yet thats why gives error

	Vec3 cameraFrontVec = -cameraPos / sqrt(cameraPos.squaredDistanceTo(Vec3(0.0)));
	Vec3 bulletPosition = cameraPos + 0.7 * cameraFrontVec;
	Vec3 bulletVelocity = 7 * cameraFrontVec;

	m_rigidBodies[bulletsEntityIdx].addRigidBody(bulletPosition, 0.07, 2.0, bulletVelocity);

}



void SphereSystemSimulator::setScene()
{

	addTarget(12, 12);
	Entity bullet;//empty bullet entity
	m_rigidBodies.push_back(bullet);
	bulletsEntityIdx = m_rigidBodies.size() - 1;

	addBullet();
	for (auto entity : m_rigidBodies) {

		m_pRigidBodySimulator->addEntities(entity.getRigidBody());
	}
}

void SphereSystemSimulator::updateEntities(vector<vector<rigidBody>> updatedEntities)
{
	for (uint32_t i = 0; i < updatedEntities.size();++i) {
		m_rigidBodies[i].updateRigidBodies(updatedEntities[i]);
	}
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	

	for (uint32_t i = 0; i < m_rigidBodies.size(); ++i) {

		vector<rigidBody> temp_RigidBodies = m_rigidBodies[i].getRigidBody();	// Rigidbodies of the entity i

		for (uint32_t j = 0; j < temp_RigidBodies.size(); ++j) {
			rigidBody& rb=temp_RigidBodies[j];
			DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

			DUC->drawSphere(rb.center, Vec3(rb.radius));
		}

	}

	//rigidBody rb;
	//for (int i = 0; i < this->getNumberOfRigidBodies(); ++i) {
	//	//Mat4 transform=getObject2WorldSpaceMatrix(temp_RigidBodies[i]);
	//	rb = temp_RigidBodies[i];
	//	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	//	//DUC->drawRigidBody(transform);

	//	DUC->drawSphere(rb.center, Vec3(rb.radius));
	//}
}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	//m_pDiffusionSimulator->simulateTimestep(timeStep);
	//check the heat values and set the point validity
	m_pRigidBodySimulator->simulateTimestep(timeStep);

}

void SphereSystemSimulator::onClick(int x, int y)
{
}

void SphereSystemSimulator::onMouse(int x, int y)
{
}
