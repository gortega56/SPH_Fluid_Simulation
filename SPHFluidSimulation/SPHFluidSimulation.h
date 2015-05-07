#pragma once
#include "Game.h"
#include "GameObject.h"
#include "Collider.h"
#include "Camera.h"
#include <vector>
#include <unordered_map>

#define PARTICLE_COUNT			2
#define MESH_COUNT				1
#define GEOMETRY_COUNT			1

#define SPH_GRID_X				3
#define SPH_GRID_Y				3
#define SPH_GRID_Z				3

#define SPH_CONTAINER_X			5.0f
#define SPH_CONTAINER_Y			5.0f
#define SPH_CONTAINER_Z			5.0f

#define SPH_CORE_RADIUS			SPH_CONTAINER_X / SPH_GRID_X
#define SPH_CORE_RADIUS2		SPH_CORE_RADIUS * SPH_CORE_RADIUS

#define WATER_REST_DENSITY		998.29f
#define WATER_VAPOR_CONSTANT	3.0f
#define WATER_VISCOSITY			3.5f

#define SURFACE_TENSION			0.0728f
#define COLOR_FIELD_THRESHOLD   7.065f

#define GRAVITATIONAL_ACCELERATION	-9.80665f

struct SPHParticle
{
	Transform	mTransform;
	RigidBody	mRigidBody;
	vec3		mAcceleration;
	float		mDensity;
	float		mPressure;

	SPHParticle() : mTransform(Transform(vec3(0.0f), vec3(0.0f), vec3(0.5f))), mRigidBody(RigidBody()), mAcceleration(vec3(0.0f)), mDensity(0.0f), mPressure(0.0f) {};
	SPHParticle(const SPHParticle& other) : mTransform(other.mTransform), mRigidBody(other.mRigidBody), mAcceleration(other.mAcceleration), mDensity(other.mDensity), mPressure(other.mPressure) {};
};

struct SPHCell
{
	vector<SPHParticle> particles;
};

class SPHFluidSimulation :
	public Game
{
public:
	GLuint		mTransformBufferID;
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

	Camera*		mCamera;

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
	void applyBoundingVolumeForce(SPHParticle& particle);
	void stepSimulation(double secondsElapsed);
	void integrateCellParticles(double deltaTime);

	// Convenience
	void  NormalizedGridPosition(vec3 worldPosition, int* indices);
	int	  SPHGridCellIndex(int x, int y, int z);
	// Smoothing Kernel Functions
	float SmoothKernelPoly6(float r2, float h, float h2);
	float SmoothKernelPoly6Laplacian(float r2, float h, float h2);
	vec3  SmoothKernelPoly6Gradient(vec3 rDiff, float r2, float h, float h2);
	vec3  SmoothKernelSpikyGradient(vec3 rDiff, float r, float h);
	float SmoothKernelViscosityLaplacian(float r, float h);
};

