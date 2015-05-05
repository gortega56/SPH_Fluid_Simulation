#pragma once
#include "Game.h"
#include "GameObject.h"
#include "Collider.h"
#include <vector>
#include <unordered_map>

struct SPHParticle
{
	Transform	mTransform;
	RigidBody	mRigidBody;
	vec3		mAcceleration;
	float		mDensity;
	float		mPressure;

	SPHParticle() : mTransform(Transform()), mRigidBody(RigidBody()), mDensity(0.0f) {};
};

struct SPHCell
{
	vector<SPHParticle> particles;
};

class SPHFluidSimulation :
	public Game
{
public:
	int			mGameObjectCount;
	int			mColliderCount;
	int			mMeshCount;
	int			mGeometryCount;
	int			mTransformCount;
	int			mRigidBodyCount;

	GameObject* mGameObjects;	// Particles
	Mesh*		mMesh;
	Collider*   mColliders;
	Geometry*	mGeometry;
	Transform*	mTransform;
	RigidBody*	mRigidBodies;
	
	SPHCell* mSPHGrid;
	unordered_map<int, vector<int>> mSPHCellIndexMap;

	SPHFluidSimulation();
	~SPHFluidSimulation();

	void init();
	void updateScene(double secondsElapsed);
	void renderScene(double secondsElapsed);
	void handleEvents(GLFWwindow* window);

	void initParticleGrid();
	void updateParticleGrid();
	void updateParticlesForces();
	void updateParticlesPressureDensity();
	void applyBoundingVolumeForce();
	void stepSimulation(double secondsElapsed);
	void integrateCellParticles(double deltaTime);

	void SPHFluidSimulation::NormalizedGridPosition(vec3 worldPosition, int* indices);
	float SmoothKernelPoly6(float r2, float h, float h2);
	float SmoothKernelPoly6Laplacian(float r2, float h, float h2);
	vec3  SmoothKernelPoly6Gradient(vec3 rDiff, float r2, float h, float h2);
	vec3  SmoothKernelSpikyGradient(vec3 rDiff, float r, float h);
	float SmoothKernelViscosityLaplacian(float r, float h);
};

