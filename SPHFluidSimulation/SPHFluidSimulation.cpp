#include "SPHFluidSimulation.h"
#include <glm/gtx/norm.hpp>

#define _USE_MATH_DEFINES 
#include <math.h>

#define PARTICLE_COUNT			1
#define MESH_COUNT				1
#define GEOMETRY_COUNT			1

#define SPH_GRID_X				3
#define SPH_GRID_Y				3
#define SPH_GRID_Z				3

#define SPH_CONTAINER_X			0.5f
#define SPH_CONTAINER_Y			0.5f
#define SPH_CONTAINER_Z			0.5f

#define SPH_PARTICLE_RADIUS		0.5f
#define SPH_PARTICLE_RADIUS2	SPH_PARTICLE_RADIUS * SPH_PARTICLE_RADIUS

#define WATER_REST_DENSITY		998.29
#define WATER_VAPOR_CONSTANT	3.0

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
	initParticleGrid();
	
}

void SPHFluidSimulation::updateScene(double secondsElapsed)
{
	updateParticleGrid();

}

void SPHFluidSimulation::renderScene(double secondsElapsed)
{

}


void SPHFluidSimulation::handleEvents(GLFWwindow* window)
{

}

void SPHFluidSimulation::initParticleGrid()
{
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {
				int index = x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y);
				SPHCell& cell = mSPHGrid[index];
				vector<int> nIndices;

				for (int offsetX = -1; offsetX <= 1; offsetX++) {
					if (x + offsetX < 0) continue;
					if (x + offsetX >= SPH_GRID_X) break;

					for (int offsetY = -1; offsetY <= 1; offsetY++) {
						if (y + offsetY < 0) continue;
						if (y + offsetY >= SPH_GRID_Y) break;

						for (int offsetZ = -1; offsetZ <= 1; offsetZ++) {
							if (z + offsetZ < 0) continue;
							if (z + offsetZ >= SPH_GRID_Z) break;

							nIndices.push_back((x + offsetX) + ((y + offsetY) * SPH_GRID_X) + ((z + offsetZ) * SPH_GRID_X * SPH_GRID_Y));
						}
					}
				}

				mSPHCellIndexMap.insert({ index, nIndices });
			}
		}
	}
}

void SPHFluidSimulation::updateParticleGrid()
{
	float halfContainerX = (SPH_CONTAINER_X * 0.5f);
	float halfContainerY = (SPH_CONTAINER_Y * 0.5f);
	float halfContainerZ = (SPH_CONTAINER_Z * 0.5f);

	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {
				SPHCell& cell = mSPHGrid[x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y)];

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
						mSPHGrid[nextCellX + (nextCellY * SPH_GRID_X) + (nextCellZ * SPH_GRID_X * SPH_GRID_Y)].particles.push_back(cell.particles[p]);
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

void SPHFluidSimulation::updateParticles()
{
	// For each SPHCell
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {
			
				int index = x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y);
				SPHCell& cell = mSPHGrid[index];
				vector<int>& neighborIndices = mSPHCellIndexMap.at(index);

				vec3 fPressure = vec3(0.0f);

				// Update each particle
				for (int p1 = 0; p1 < cell.particles.size(); p1++) {
					// Against all neighboring cell particles
					for (int nIndex : neighborIndices) {
						SPHCell& nCell = mSPHGrid[nIndex];
						for (int p2 = 0; p2 < nCell.particles.size(); p2++) {
							vec3 rDiff = cell.particles[p1].mTransform.Position - nCell.particles[p2].mTransform.Position;
							float radius = length(rDiff);
							float radius2 = radius * radius;
							if (radius2 <= SPH_PARTICLE_RADIUS2) {
								// Density
								cell.particles[p1].mDensity += cell.particles[p1].mRigidBody.mass * SmoothKernelPoly6(radius2, SPH_PARTICLE_RADIUS, SPH_PARTICLE_RADIUS2);
								cell.particles[p1].mPressure = WATER_VAPOR_CONSTANT * (cell.particles[p1].mDensity - WATER_REST_DENSITY);

								// PressureForce 
								float symmetricPressure = (cell.particles[p1].mPressure + nCell.particles[p2].mPressure) / (2 * nCell.particles[p2].mDensity);
								vec3 pGradient = SmoothKernelSpikyGradient(rDiff, radius, SPH_PARTICLE_RADIUS);
								fPressure += cell.particles[p1].mRigidBody.mass * symmetricPressure * pGradient;

							}
						}
					}
					
					// Remove comparison between identical particle
					cell.particles[p1].mDensity -= SmoothKernelPoly6(0, SPH_PARTICLE_RADIUS, SPH_PARTICLE_RADIUS2);


				}

			}
		}
	}
}

void SPHFluidSimulation::stepSimulation(double secondsElapsed)
{

}

float SPHFluidSimulation::SmoothKernelPoly6(float r2, float h, float h2)
{
	float coefficient = 315.f / (64.0f * M_PI * powf(h, 9));
	return coefficient * powf(h2 - r2, 3);
}

float SPHFluidSimulation::SmoothKernelPoly6Laplacian(float r2, float h, float h2)
{
	float coefficient = -945.f / (32.0f * M_PI * powf(h, 9));
	return coefficient * (h2 - r2) * ((3.f * h2) - (7.f * r2));
}

vec3 SPHFluidSimulation::SmoothKernelPoly6Gradient(vec3 rDiff, float r2, float h, float h2)
{
	float coefficient = -945.f / (32.0f * M_PI * powf(h, 9));
	return (coefficient * powf(h2 - r2, 2)) * rDiff;
}

vec3 SPHFluidSimulation::SmoothKernelSpikyGradient(vec3 rDiff, float r, float h)
{
	float coefficient = -45.f / (M_PI * powf(h, 6));
	return (coefficient * powf(h - r, 2) * rDiff) / r;
}

float SPHFluidSimulation::SmoothKernelViscosityLaplacian(float r, float h)
{
	float coefficient = 45.f / (M_PI * powf(h, 6));
	return coefficient * (h - r);
}
