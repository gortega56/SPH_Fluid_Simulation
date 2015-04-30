#include "SPHFluidSimulation.h"

#define PARTICLE_COUNT	1
#define MESH_COUNT		1
#define GEOMETRY_COUNT	1

#define SPH_GRID_X		10
#define SPH_GRID_Y		10
#define SPH_GRID_Z		10

SPHFluidSimulation::SPHFluidSimulation() : mGameObjectCount(PARTICLE_COUNT), mColliderCount(PARTICLE_COUNT), mTransformCount(PARTICLE_COUNT), mMeshCount(MESH_COUNT), mRigidBodyCount(PARTICLE_COUNT), mGeometryCount(GEOMETRY_COUNT)
{
	// Missing GameObjects for now
	mColliders = new Collider[mColliderCount];
	mTransform = new Transform[mTransformCount];
	mMesh = new Mesh[mMeshCount];
	mRigidBodies = new RigidBody[mRigidBodyCount];
	mGeometry = new Geometry[mGeometryCount];
	mSPHGrid = new SPHCell[SPH_GRID_X * SPH_GRID_Y * SPH_GRID_Z];
}


SPHFluidSimulation::~SPHFluidSimulation()
{
	delete[] mColliders;
	delete[] mTransform;
	delete[] mMesh;
	delete[] mRigidBodies;
	delete[] mGeometry;
}

void SPHFluidSimulation::init()
{
	Game::init();


}

void SPHFluidSimulation::updateScene(double secondsElapsed)
{

}

void SPHFluidSimulation::renderScene(double secondsElapsed)
{

}


void SPHFluidSimulation::handleEvents(GLFWwindow* window)
{

}