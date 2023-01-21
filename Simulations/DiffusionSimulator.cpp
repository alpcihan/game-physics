#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(uint32_t w = 16, uint32_t h = 16)
	: m_w(w), m_h(h)
{
	SpherePoint initVals;
	initVals.isActive = true;
	initVals.temp = 0;

	m_temps = std::vector<SpherePoint>(w * h, initVals);
	m_T = std::vector<SpherePoint>(w * h, initVals);
}

void Grid::applyUpdates()
{
	std::vector<SpherePoint> _T = std::move(m_temps);
	m_temps = std::move(m_T);
	m_T = std::move(_T);
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
	T = new Grid();
	fillT(T);
}

DiffusionSimulator::~DiffusionSimulator()
{
	delete T;
	T = nullptr;
}

const char *DiffusionSimulator::getTestCasesStr()
{
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	fillT(T);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass *DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		TwAddVarRW(DUC->g_pTweakBar, "nx", TW_TYPE_INT32, &m_nx, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "ny", TW_TYPE_INT32, &m_ny, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &m_alpha, "step=0.1 min=0.1");
		break;
	case 1:
		cout << "Implicit solver!\n";
		TwAddVarRW(DUC->g_pTweakBar, "nx", TW_TYPE_INT32, &m_nx, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "ny", TW_TYPE_INT32, &m_ny, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &m_alpha, "step=0.1 min=0.1");
		break;
	default:
		break;
	}
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		delete T;
		T = new Grid(m_nx, m_ny);
		reset();
		break;
	case 1:
		cout << "Implicit solver!\n";
		delete T;
		T = new Grid(m_nx, m_ny);
		reset();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid *DiffusionSimulator::diffuseTemperatureExplicit(Real dTime)
{
	// Iterate through all non-boundary grid points
	for (int w = 1; w < T->w() - 1; w++)
	{
		for (int h = 1; h < T->h() - 1; h++)
		{

			if (T->getPointStatus(w,h) == false) {
				T->set(w, h, 0);
				continue;
			}
				
			// Get the temperature values at the current grid point and its neighbors
			Real u, uxp, uxn, uyp, uyn;
			u = T->get(w, h);
			uxp = T->get(w + 1, h);
			uxn = T->get(w - 1, h);
			uyp = T->get(w, h + 1);
			uyn = T->get(w, h - 1);

			// Calculate the second derivatives of the temperature in the x and y directions using finite differences
			Real d2x, d2y;
			d2x = uxp - 2 * u + uxn;
			d2y = uyp - 2 * u + uyn;

			// Update the temperature at the current grid point using the explicit Euler method
			Real result = u + m_alpha * dTime * (d2x + d2y);
			T->set(w, h, result);
		}
	}

	// Apply the temperature updates to the grid
	T->applyUpdates();

	// Return the updated temperature grid
	return T;
}

void setupB(std::vector<Real> &b, Grid *T)
{ // add your own parameters
	// to be implemented
	// set vector B[sizeX*sizeY]
	for (int i = 0; i < T->w() * T->h(); i++)
	{
		b.at(i) = T->get(i);
	}
}

void fillT(Grid *grid)
{ // add your own parameters
	// to be implemented
	// fill T with solved vector x
	// make sure that the temperature in boundary cells stays zero
	std::mt19937 eng;
	std::uniform_real_distribution<double> randTemp(-1.0f, 1.0f);

	for (int w = 1; w < grid->w() - 1; w++)
	{
		for (int h = 1; h < grid->h() - 1; h++)
		{
#ifdef DEBUG // UE for testing

			if (w >= (grid->w() / 2 - grid->w() / 4) && w <= (grid->w() / 2 + grid->w() / 4) && h >= (grid->h() / 2 - grid->h() / 4) && w <= (grid->h() / 2 + grid->h() / 4)) {

				grid->setPointStatus(w,h, false);
				grid->set(w, h, 0);
				continue;
			}
#endif // DEBUG
			grid->set(w, h, randTemp(eng));
		}
	}

	grid->applyUpdates();
}

void setupA(SparseMatrix<Real> &A, double factor, Grid *T)
{
	// Set the diagonal value for boundary cells to 1.0
	int w = T->w();
	int h = T->h();

	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			if (i == 0 || j == 0 || i == w - 1 || j == h - 1)
			{
				A.set_element(i * h + j, i * h + j, 1);
			}
		}
	}

	// Set the coefficients for the non-boundary cells
	for (int i = 1; i < w - 1; i++)
	{
		for (int j = 1; j < h - 1; j++)
		{
			A.set_element(i * h + j, (i - 1) * h + j, -factor);
			A.set_element(i * h + j, i * h + (j - 1), -factor);
			A.set_element(i * h + j, i * h + j, 1 + 4 * factor);
			A.set_element(i * h + j, i * h + (j + 1), -factor);
			A.set_element(i * h + j, (i + 1) * h + j, -factor);
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit(Real dTime)
{ // add your own parameters
	// solve A T = b
	// to be implemented
	const int N = T->w() * T->h(); // N = sizeX*sizeY
	SparseMatrix<Real> *A = new SparseMatrix<Real>(N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, m_alpha * dTime, T);
	setupB(*b, T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j)
	{
		x[j] = 0.;
	}

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 2);
	// x contains the new temperature values

	for (int i = 1; i < T->w() - 1; i++)
	{
		for (int j = 1; j < T->h() - 1; j++)
		{
			T->set(i, j, x[j * T->w() + i]);
		}
	}

	T->applyUpdates();
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	// visualization
	float scale = 0.1;
	Vec3 radius(scale);

	for (int w = 0; w < T->w(); w++)
	{
		for (int h = 0; h < T->h(); h++)
		{
			if (T->getPointStatus(w, h) == false) {
				continue;
			}
			
			Real t = T->get(w, h);
			Vec3 color = Vec3(t, 0, -t);

			float x = (w - T->w() * 0.5) * scale, y = (h - T->h() * 0.5) * scale;
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, color);
			DUC->drawSphere(Vec3(x, y, 0), radius);
		}
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext *pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;



}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
