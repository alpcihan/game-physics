#pragma once
#include<vector>
#include "RigidBodySystemSimulator.h"
#include "DiffusionSimulator.h"

class SphereSystem {

	public:
		SphereSystem(DiffusionSimulator& diffuseSim, RigidBodySystemSimulator& rigidSim, uint32_t w, uint32_t h) : DiffusionSimulator(diffuseSim), RigidBodySimulator(rigidSim){
			Points.resize(w * h);
		}
		void UpdatePointsInfo();//updates point informations w.r.t. simulator info
		void drawSpheres();
	private:
		vector<rigidBody> Points;
		DiffusionSimulator& DiffusionSimulator;
		RigidBodySystemSimulator& RigidBodySimulator;
};