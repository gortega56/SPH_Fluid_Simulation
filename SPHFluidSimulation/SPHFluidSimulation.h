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
	RigidBody*	mRigidBodies;
	
	GameObject* mBoundingBox;
	Mesh*		mSphere;
	Mesh*		mCube;
	Material*	mSphereMaterial;
	Material*	mCubeMaterial;

	mat4x4*		mTransforms;

	SPHCell* mSPHGrid;
	unordered_map<int, vector<int>> mSPHCellIndexMap;

	SPHFluidSimulation();
	~SPHFluidSimulation();

	// Game Overrides
	void init();
	void updateScene(double secondsElapsed);
	void renderScene(double secondsElapsed);
	void handleEvents(GLFWwindow* window);

	// Render Methods
	void initGeometry();
	void updateTransforms();

	// Simulation Methods
	void initParticleGrid();
	void updateParticleGrid();
	void updateParticlesForces();
	void updateParticlesPressureDensity();
	void applyBoundingVolumeForce();
	void stepSimulation(double secondsElapsed);
	void integrateCellParticles(double deltaTime);

	// Convenience
	void  NormalizedGridPosition(vec3 worldPosition, int* indices);
	
	// Smoothing Kernel Functions
	float SmoothKernelPoly6(float r2, float h, float h2);
	float SmoothKernelPoly6Laplacian(float r2, float h, float h2);
	vec3  SmoothKernelPoly6Gradient(vec3 rDiff, float r2, float h, float h2);
	vec3  SmoothKernelSpikyGradient(vec3 rDiff, float r, float h);
	float SmoothKernelViscosityLaplacian(float r, float h);
};

