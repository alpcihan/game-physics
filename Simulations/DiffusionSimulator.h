#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

// impement your own grid class for saving grid data
class Grid
{
public:
	struct PointInfo {
		bool isActive;
		Real temp;
	};

	// Construtors
	Grid(uint32_t w, uint32_t h);

	uint32_t w() const { return m_w; }
	uint32_t h() const { return m_h; }

	void set(uint32_t w, uint32_t h, Real value) { m_T[m_w * h + w].temp = value; }
	const std::vector<PointInfo> &get() const { return m_temps; }
	Real get(uint32_t w, uint32_t h) const { return m_temps[m_w * h + w].temp; }
	Real get(uint32_t i) const { return m_temps[i].temp; }
	bool getPointStatus(uint32_t w, uint32_t h) const { return m_temps[m_w * h + w].isActive; }
	void setPointStatus(uint32_t w, uint32_t h, bool status) { m_temps[m_w * h + w].isActive = status; m_T[m_w * h + w].isActive = status;}

	void applyUpdates();

private:
	std::vector<PointInfo> m_temps;
	std::vector<PointInfo> m_T;
	uint32_t m_w, m_h;
};

class DiffusionSimulator : public Simulator
{
public:
	// Construtors
	DiffusionSimulator();
	~DiffusionSimulator();

	// Functions
	const char *getTestCasesStr();
	void initUI(DrawingUtilitiesClass *DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext *pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed){};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid *diffuseTemperatureExplicit(Real dTime);
	void diffuseTemperatureImplicit(Real dTime);
	Grid getGrid() { return (*T); }

private:
	// Attributes
	Vec3 m_vfMovableObjectPos;
	Vec3 m_vfMovableObjectFinalPos;
	Vec3 m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
	float m_alpha = 1;
	int m_nx = 16;
	int m_ny = 16;
	Grid* T; // save results of every timestep
};

void fillT(Grid *T);

#endif