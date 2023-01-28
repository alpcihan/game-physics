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
	this->m_iTestCase = 0;	
	grid_w = 32;
	grid_h = 32;
	m_pDiffusionSimulator = new DiffusionSimulator(grid_w,grid_h);
	m_pRigidBodySimulator = new RigidBodySystemSimulator();

	m_pRigidBodySimulator->setUpdateCallback(std::bind(&SphereSystemSimulator::updateEntities, this, std::placeholders::_1));

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
	clearRigidBodies();
	m_pRigidBodySimulator->reset();
	//TODO: write here 
}

void SphereSystemSimulator::clearRigidBodies() {
	// Clear the elements of the nested vectors
	for (auto& entity: m_entities) {
		entity.second.clear();
	}
}

void SphereSystemSimulator::addTarget(uint16_t n_x, uint16_t n_y)
{
	float scale = 0.1f;
	for (uint16_t i = 0; i < n_x; ++i) {
		for (uint16_t j = 0; j < n_y; ++j) {

			float x = (i - n_x * 0.5) * scale, y = (j- n_y* 0.5) * scale;
			m_entities[EntityType::TARGET].addRigidBody(Vec3(x, y, 0.0), 0.1, 2);
		}
	}
}

void SphereSystemSimulator::addBullet()// TODO: Define the function with initial bullet speed(which can be set from UI) and position(from eye pointer) 
{
	Vec3 cameraPos = Vec3(0.0, -3.0, 0.0);//Vec3(DUC->g_camera.GetEyePt());//IF we call from addBullet from constructer the DUC is not initialised yet thats why gives error

	Vec3 cameraFrontVec = -cameraPos / sqrt(cameraPos.squaredDistanceTo(Vec3(0.0)));
	Vec3 bulletPosition = cameraPos + 0.7 * cameraFrontVec;
	Vec3 bulletVelocity = 7 * cameraFrontVec;

	m_entities[EntityType::BULLET].addRigidBody(bulletPosition, 0.07, 2.0, bulletVelocity);

}

void SphereSystemSimulator::setScene()
{
	reset();
	addTarget(12, 12);
	addBullet();
	for (auto& entity : m_entities) {
		m_pRigidBodySimulator->addEntities(entity.second.getRigidBody());
	}
}

void SphereSystemSimulator::updateEntities(vector<rigidBody> &rigidBodyBuffer)
{
	//// Assert that the number of rigid bodies in the buffer is not less than the total number of rigid bodies in all entities
	//assert(rigidBodyBuffer.size() >= m_totalNumRigidBodies);

	// Create an iterator pointing to the beginning of the buffer
	auto bufferIdx = rigidBodyBuffer.begin();

	for (auto& entity : m_entities) {

		// Get the end iterator of the current entity's rigid bodies in the buffer
		auto bufferIdxEndOfEntity = bufferIdx + entity.second.getNumberOfRigidBodies();

		vector<rigidBody> temp_rigidBodies;
		temp_rigidBodies.insert(temp_rigidBodies.end(), bufferIdx, bufferIdxEndOfEntity);
		entity.second.updateRigidBodies(temp_rigidBodies);
		bufferIdx = bufferIdxEndOfEntity;
	}

	// Assert that the buffer iterator reached the end of the buffer
	assert(bufferIdx == rigidBodyBuffer.end());
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

	for (auto& entity: m_entities) {
		//entity.second.draw(DUC);
		vector<rigidBody>& temp_rigidBodies=entity.second.getRigidBody();
		

		for (size_t i = 0; i < temp_rigidBodies.size(); ++i) {
			Real t = m_targetGrid->get(i / grid_w, i % grid_w);
			Vec3 color = (t, 0, -t);

			DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, color);
			DUC->drawSphere(temp_rigidBodies[i].center, Vec3(temp_rigidBodies[i].radius));
		}
	
	}

}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase) {
	case 0:
		std::cout << "Demo0\n";
		setScene();
		break;
	
	default:
		break;
	}
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	m_targetGrid= m_pDiffusionSimulator->diffuseTemperatureExplicit(timeStep);
	//check the heat values and set the point validity
	m_pRigidBodySimulator->simulateTimestep(timeStep);

}

void SphereSystemSimulator::onClick(int x, int y)
{
}

void SphereSystemSimulator::onMouse(int x, int y)
{
}
