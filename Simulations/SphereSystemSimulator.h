#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "spheresystem.h"
#include "DiffusionSimulator.h"
#include "RigidBodySystemSimulator.h"


#define NAIVEACC 0
#define GRIDACC 1

/// <summary>
/// 
/// This Simulator is the parent simulator which conatins RigidBodySimulator and DiffusionSimulator
/// It runs the simulator steps and shares the simulator informations with the SphereSystem via simulator refenrences.
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
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	
protected:
	//// Attributes
	//Vec3 externalForce;
	//Point2D m_mouse;
	//Point2D m_trackmouse;
	//Point2D m_oldtrackmouse;
	//float m_fMass;
	//float m_fRadius;
	//float m_fForceScaling;
	//float m_fDamping;
	////int   m_iNumSpheres;
	//
	//int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	//static std::function<float(float)> m_Kernels[5];
	
	//int   m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)
	
	SphereSystem * m_pTarget;
	RigidBodySystemSimulator* m_pRigidBodySimulator;
	DiffusionSimulator* m_pDiffusionSimulator;
	// add your own sphere system member
	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	// SphereSystem * m_pSphereSystemGrid; 

};

#endif