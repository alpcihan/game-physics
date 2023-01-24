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
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	m_pRigidBodySimulator->initUI(DUC);
	m_pDiffusionSimulator->initUI(DUC);
}

void SphereSystemSimulator::reset()
{
	m_Target.clear();
	m_bullets.clear();

	m_pRigidBodySimulator->reset();
	m_pDiffusionSimulator->reset();

}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

	for (uint32_t i = 0; i < m_Target.size(); ++i) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawSphere(m_Target[i].center, m_Target[i].radius);
	}
	for (uint32_t i = 0; i < m_bullets.size(); ++i) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawSphere(m_bullets[i].center, m_bullets[i].radius);
	}
	
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	m_pDiffusionSimulator->simulateTimestep(timeStep);
	//check the heat values and set the point validity
	m_pRigidBodySimulator->simulateTimestep(timeStep);

}
