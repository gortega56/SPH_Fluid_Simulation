#pragma once
#include "Game.h"
#include "GameObject.h"
#include "Collider.h"
#include <vector>

struct SPHCell
{
	vector<GameObject*> mGameObjects;	// Particles
	vector<Mesh*>		mMesh;
	vector<Collider*>   mColliders;
	vector<Geometry*>	mGeometry;
	vector<Transform*>	mTransforms;
	vector<RigidBody*>	mRigidBodies;
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

