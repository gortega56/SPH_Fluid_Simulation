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
};

