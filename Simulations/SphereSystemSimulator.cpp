#include "SphereSystemSimulator.h"
#include "DiffusionSimulator.h"

#define PI 3.141593

SphereSystemSimulator::SphereSystemSimulator()
{
	this->m_iTestCase = 0;	
	grid_w = 13;
	grid_h = 13;
	m_bulletVelocityScaler = 100.0f;
	m_heatImpact = 0.5f;
	m_pDiffusionSimulator = new DiffusionSimulator(grid_w, grid_h);
	m_pRigidBodySimulator = new RigidBodySystemSimulator(&m_destroyVec, grid_w, grid_h);

	m_pRigidBodySimulator->setUpdateCallback(std::bind(&SphereSystemSimulator::updateEntities, this, std::placeholders::_1));

	m_destroyVec = std::vector<bool>(grid_w * grid_h * 6, false);
}

const char* SphereSystemSimulator::getTestCasesStr()
{
	return nullptr;
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Bullet Velocity", TW_TYPE_FLOAT, &m_bulletVelocityScaler, "min=1.0");
	TwAddVarRW(DUC->g_pTweakBar, "Heat Impact", TW_TYPE_FLOAT, &m_heatImpact, "min=0.01");

}

void SphereSystemSimulator::reset()
{
	clearRigidBodies();
	m_pRigidBodySimulator->reset();
	m_pDiffusionSimulator->reset();

	for (auto& difSim : m_pDiffusionSimulators) {
		difSim->reset();
	}

	m_mouse.x = m_mouse.y = 0;
	m_oldmouse.x = m_oldmouse.y = 0;
}

void SphereSystemSimulator::clearRigidBodies() {
	// Clear the elements of the nested vectors
	for (auto& entity: m_entities) {
		entity.second.clear();
	}

	for (auto& entity : m_entities) {
		entity.second.clear();
	}
}

void SphereSystemSimulator::addTarget(uint32_t n_x, uint32_t n_y, Vec3 centerPos = Vec3(0.0f), Vec3 rotation = Vec3(0.0f, 0.0f, 0.0f), EntityType target = EntityType::TARGET0)
{
	float radius = 0.1;
	rotation *= (PI/180);
	XMMATRIX rotationMat = XMMatrixRotationRollPitchYawFromVector(rotation.toDirectXVector());

	m_pDiffusionSimulators.push_back(new DiffusionSimulator(grid_w, grid_h));


	Vec3 topLeftPos = centerPos + Vec3(-int(n_x-1) / 2.0f, -int(n_y-1) / 2.0f, 0.0f) * radius;
	for (size_t i = 0; i < n_x; ++i) {
		for (size_t j = 0; j < n_y; ++j) {

			Vec3 pos = topLeftPos + Vec3(radius * i, radius * j, 0);
			//pos = rotationMat * pos.toDirectXVector();
			pos = Vec3(XMVector3Transform((pos-centerPos).toDirectXVector(), rotationMat)) + centerPos;
			

			m_entities[target].addRigidBody(pos, radius, 2, true);
		}
	}
}

void SphereSystemSimulator::addBullet()
{
	Vec3 cameraPos = Vec3(DUC->g_camera.GetEyePt()); //Vec3(0.0, -3.0, 0.0);//IF we call from addBullet from constructer the DUC is not initialised yet thats why gives error

	Vec3 cameraFrontVec = -cameraPos / sqrt(cameraPos.squaredDistanceTo(Vec3(0.0)));
	Vec3 bulletPosition = cameraPos + 0.7 * cameraFrontVec;
	Vec3 bulletVelocity = m_bulletVelocityScaler * cameraFrontVec;

	size_t idx = m_entities[EntityType::BULLET].addRigidBody(bulletPosition, 0.07, 2.0, false, bulletVelocity);
	m_pRigidBodySimulator->addEntity(m_entities[EntityType::BULLET].getRigidBody(idx));
}

void SphereSystemSimulator::setScene()
{
	reset();
	addTarget(grid_w, grid_h, Vec3(-0.7, 0, 0), Vec3(0, 90, 0), EntityType::TARGET0);
	addTarget(grid_w, grid_h, Vec3(0.7, 0, 0), Vec3(0, 90, 0), EntityType::TARGET1);
	addTarget(grid_w, grid_h, Vec3(0, 0, 0.7), Vec3(0, 0, 90), EntityType::TARGET2);
	addTarget(grid_w, grid_h, Vec3(0, 0, -0.7), Vec3(0, 0, 90), EntityType::TARGET3);
	addTarget(grid_w, grid_h, Vec3(0, 0.7, 0), Vec3(90, 0, 0), EntityType::TARGET4);
	addTarget(grid_w, grid_h, Vec3(0, -0.7, 0), Vec3(90, 0, 0), EntityType::TARGET5);


	for (auto& entity : m_entities) {
		m_pRigidBodySimulator->addEntities(entity.second.getRigidBodies());
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

		vector<rigidBody>& temp_rigidBodies=entity.second.getRigidBodies();
		Vec3 color(1.0f);
		
		for (size_t i = 0; i < temp_rigidBodies.size(); ++i) {
			
			if (isSphereDestroyed(i, entity.first)) continue;

			if (entity.first > EntityType::BULLET) {

				Real t = m_pDiffusionSimulators[entity.first-2]->getGrid().get(i);
				color = (-t, 0, t);

			}

			DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(0, 0, 0), 1.0, color);
			DUC->drawSphere(temp_rigidBodies[i].center, Vec3(temp_rigidBodies[i].radius));
		}
	}
}

void SphereSystemSimulator::updateTargetHeat(uint32_t i) {
	for (size_t j = EntityType::TARGET0; j < m_pDiffusionSimulators.size() + 2; ++j) {
		auto& target_RBs = m_entities[EntityType(j)].getRigidBodies();
		Grid& targetGrid = m_pDiffusionSimulators[j - 2]->getGrid();

		for (size_t i = 0; i < target_RBs.size(); ++i) {
			if (target_RBs[i].participatedCollusion) {
				Real current_Temp = targetGrid.get(i);
				setHeat(targetGrid, i, current_Temp + m_heatImpact);
			}

			if (targetGrid.get(i) > 1.0)
			{
				destroySphere(i, (EntityType)j);
				targetGrid.setPointStatus(i, false);
			}
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

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed) {}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	//m_pDiffusionSimulator->simulateTimestep(timeStep);
	for (auto difSim : m_pDiffusionSimulators) {
		difSim->simulateTimestep(timeStep);
	}
	//check the heat values and set the point validity
	m_pRigidBodySimulator->simulateTimestep(timeStep);
	updateTargetHeat(0);
}

void SphereSystemSimulator::onClick(int x, int y) { addBullet(); }

void SphereSystemSimulator::onMouse(int x, int y){}

void SphereSystemSimulator::rotateCameraBy(Vec3 rotation) {}

void SphereSystemSimulator::destroySphere(uint32_t i, EntityType type)
{
	static const uint32_t count = m_destroyVec.size() / 6;

	if (type < EntityType::TARGET0 || m_destroyVec[count * (type - 1) + i]) return;

	m_destroyVec[count * (type - 1) + i] = true;
	std::cout << "destroyed: " << count * (type - 1) + i << std::endl;
}

bool SphereSystemSimulator::isSphereDestroyed(uint32_t i, EntityType type)
{
	static const uint32_t count = m_destroyVec.size() / 6;

	if (type < EntityType::TARGET0) false;

	return m_destroyVec[count * (type - 1) + i];
}