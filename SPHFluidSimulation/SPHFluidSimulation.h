#pragma once
#include "Game.h"
#include "GameObject.h"
#include "Collider.h"
#include "Camera.h"
#include <vector>
#include <unordered_map>

#define MESH_COUNT							1
#define GEOMETRY_COUNT						1

#define SPH_CORE_RADIUS						1.25f
#define SPH_CORE_RADIUS2					SPH_CORE_RADIUS * SPH_CORE_RADIUS

#define SPH_CONTAINER_X						30.0f
#define SPH_CONTAINER_Y						15.0f
#define SPH_CONTAINER_Z						3.0f

#define SPH_GRID_X							(int)std::ceil(SPH_CONTAINER_X / SPH_CORE_RADIUS)
#define SPH_GRID_Y							(int)std::ceil(SPH_CONTAINER_Y / SPH_CORE_RADIUS)
#define SPH_GRID_Z							(int)std::ceil(SPH_CONTAINER_Z / SPH_CORE_RADIUS)
#define SPH_GRID_COUNT						SPH_GRID_X * SPH_GRID_Y * SPH_GRID_Z

#define PARTICLE_VOLUME						1.0f			//  WATER_MASS / WATER_REST_DENSITY 
#define PARTICLE_SCALE						1.24f			//  PARTICLE_RADIUS * 2
#define PARTICLE_RADIUS						0.62f			//  r^3 = (3/4) * (1/pi) * PARTICLE_VOLUME

#define SPH_CELL_PARTICLES_X				 std::floor(SPH_CONTAINER_X / PARTICLE_SCALE) * 0.25f
#define SPH_CELL_PARTICLES_Y				 std::floor(SPH_CONTAINER_Y / PARTICLE_SCALE)
#define SPH_CELL_PARTICLES_Z				 std::floor(SPH_CONTAINER_Z / PARTICLE_SCALE)

#define PARTICLE_COUNT						SPH_CELL_PARTICLES_X * SPH_CELL_PARTICLES_Y *SPH_CELL_PARTICLES_Z

// All Values for Water at 20 degrees Celsius
#define WATER_ABSOLUTE_PRESSURE				238.4906f		// kg/m^2
#define IDEAL_WATER_VAPOR_CONSTANT			461.5f			// J/kg K
#define WATER_MASS							1000.0f			// 1 g/cm^3 = 1000 kg/m^3
#define WATER_REST_DENSITY					998.2f			// kg/m^3 
#define WATER_DYNAMIC_VISCOSITY				0.001002f		// s/m^2 x 10^-3
#define WATER_SURFACE_TENSION				0.0728f         // N/m
#define COLOR_FIELD_THRESHOLD				0.065f

#define GRAVITATIONAL_ACCELERATION			-9.80665f		// m/s^2

#define SPH_CONTAINER_SPRING_CONSTANT		200.0f
#define SPH_CONTAINER_DAMPING				-0.9f

struct SPHParticle
{
	Transform	mTransform;
	RigidBody	mRigidBody;
	vec3		mAcceleration;
	float		mDensity;
	float		mPressure;

	SPHParticle() : mTransform(Transform(vec3(0.0f), vec3(0.0f), vec3(PARTICLE_SCALE))), mRigidBody(RigidBody(WATER_MASS)), mAcceleration(vec3(0.0f)), mDensity(0.0f), mPressure(0.0f) {};
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

	SPHParticle* mSPHParticles;

	bool Play;

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
	void UpdateParticleGrid();
	void updateParticlesForces();
	void updateParticlesPressureDensity();
	void applyBoundingVolumeForce(SPHParticle& particle);
	void stepSimulation(double secondsElapsed);
	void EulerIntegrationStep(double deltaTime);
	void VerletIntegrationStep(double deltaTime);

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

