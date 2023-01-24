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

	Entity bulletsEntity;
	m_rigidBodies.push_back(bulletsEntity);
}




void SphereSystemSimulator::addTarget(uint32_t n_x, uint32_t n_y)
{
	Entity targetEntity;

	for (uint32_t i = 0; i < n_x; ++i) {
		for (uint32_t j = 0; j < n_y; ++j) {
			targetEntity.addRigidBody(Vec3(0.0), 0.1, 2);
		}
	}

	m_rigidBodies.push_back(targetEntity);
}

void SphereSystemSimulator::addBullet()
{

	Vec3 cameraPos = Vec3(DUC->g_camera.GetEyePt());


	Vec3 cameraFrontVec = -cameraPos / sqrt(cameraPos.squaredDistanceTo(Vec3(0.0)));
	Vec3 bulletPosition = cameraPos + 0.7 * cameraFrontVec;
	Vec3 bulletVelocity = 7 * cameraFrontVec;


	m_rigidBodies[bulletsEntityIdx].addRigidBody(bulletPosition, 0.07, 2.0, bulletVelocity);
}



void SphereSystemSimulator::setScene()
{
	for (auto entity : m_rigidBodies) {

		m_pRigidBodySimulator->addEntities(entity.getRigidBody());
	}
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	rigidBody rb;

	for (uint32_t i = 0; i < m_rigidBodies.size(); ++i) {

		vector<rigidBody> temp_RigidBodies = m_rigidBodies[i].getRigidBody();	// Rigidbodies of the entity i

		for (uint32_t j = 0; j < temp_RigidBodies.size(); ++j) {
			rb = temp_RigidBodies[j];
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

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	//m_pDiffusionSimulator->simulateTimestep(timeStep);
	//check the heat values and set the point validity
	m_pRigidBodySimulator->simulateTimestep(timeStep);

}
