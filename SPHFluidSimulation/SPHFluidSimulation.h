#pragma once
#include "Game.h"
#include "GameObject.h"
#include "Collider.h"
#include <vector>


struct SPHParticle
{
	Transform mTransform;
	RigidBody mRigidBody;
};

struct SPHCell
{
	vector<SPHParticle> particles;
};

class SPHFluidSimulation :
	public Game
{
public:
	int			mSPHGridX, mSPHGridY, mSPHGridZ;
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



	SPHFluidSimulation();
	~SPHFluidSimulation();

	void init();
	void updateScene(double secondsElapsed);
	void renderScene(double secondsElapsed);
	void handleEvents(GLFWwindow* window);

	void updateParticleGrid();
	void updateParticles();
	void stepSimulation(double secondsElapsed);

	float SmoothKernelPoly6(float r2, float h, float h2);
	float SmoothKernelPoly6Laplacian(float r2, float h, float h2);
	void  SmoothKernelPoly6Gradient(vec3 rDiff, float r2, float h, float h2, vec3* gradient);
	void  SmoothKernelSpikyGradient(vec3 rDiff, float r, float h, vec3* gradient);
	float SmoothKernelViscosityLaplacian(float r, float h);
};

