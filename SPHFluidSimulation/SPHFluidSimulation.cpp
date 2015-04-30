#include "SPHFluidSimulation.h"
#include <math.h>

#define PARTICLE_COUNT		1
#define MESH_COUNT			1
#define GEOMETRY_COUNT		1

#define SPH_GRID_X			10
#define SPH_GRID_Y			10
#define SPH_GRID_Z			10

#define SPH_CONTAINER_X		0.5f
#define SPH_CONTAINER_Y		0.5f
#define SPH_CONTAINER_Z		0.5f

#define SPH_PARTICLE_RADIUS 0.5f

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
	float halfContainerX = (SPH_CONTAINER_X * 0.5f);
	float halfContainerY = (SPH_CONTAINER_Y * 0.5f);
	float halfContainerZ = (SPH_CONTAINER_Z * 0.5f);

	for (int x = 0; x < mSPHGridX; x++) {
		for (int y = 0; y < mSPHGridX; y++) {
			for (int z = 0; z < mSPHGridX; z++) {
				SPHCell& cell = mSPHGrid[x + (y * mSPHGridX) + (z * mSPHGridX * mSPHGridY)];
				
				vector<int> indicesToRemove;
				for (int p = 0; p < cell.particles.size(); p++) {
					// This calculation depends on where my world space container is.
					// SPH Cells start at 0,0,0 and are built to +n, +n, +n
					int nextCellX = std::floor((cell.particles[p].mTransform.Position.x + halfContainerX) / SPH_PARTICLE_RADIUS);
					int nextCellY = std::floor((cell.particles[p].mTransform.Position.y + halfContainerY) / SPH_PARTICLE_RADIUS);
					int nextCellZ = std::floor((cell.particles[p].mTransform.Position.z + halfContainerZ) / SPH_PARTICLE_RADIUS);

					// TO DO: Will particles leave the grid...

					// Check if particle has moved to another grid
					if (x != nextCellX || y != nextCellY || z != nextCellZ) {
						mSPHGrid[nextCellX + (nextCellY * mSPHGridX) + (nextCellZ * mSPHGridX * mSPHGridY)].particles.push_back(cell.particles[p]);
						indicesToRemove.push_back(p);
					}
				}

				for (int index = indicesToRemove.size() - 1; index <= 0; index--) {
					cell.particles.erase(std::begin(cell.particles) + index);
				}
			}
		}
	}
}

void SPHFluidSimulation::renderScene(double secondsElapsed)
{

}


void SPHFluidSimulation::handleEvents(GLFWwindow* window)
{

}