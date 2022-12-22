#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(uint32_t w, uint32_t h);
	
	uint32_t w() const { return m_w; }
	uint32_t h() const { return m_h; }
	
	void set(uint32_t w, uint32_t h, Real value) { m_T[m_w * h + w] = value; }
	const std::vector<Real>& get() const { return m_temps; }
	Real get(uint32_t w, uint32_t h) const { return m_temps[m_w * h + w]; }
	
	void applyUpdates();

private:
	std::vector<Real> m_temps;
	std::vector<Real> m_T;
	uint32_t m_w, m_h;
};

class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();
	~DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(Real dTime);
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step
	float m_alpha = 1;
	int m_nx = 16;
	int m_ny = 16;
};

void fillT(Grid* T);

#endif