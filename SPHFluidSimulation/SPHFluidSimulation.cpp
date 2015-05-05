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
	updateParticlesPressureDensity();
	updateParticlesForces();
	stepSimulation(secondsElapsed);
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
					int nextCell[3];
					NormalizedGridPosition(cell.particles[p].mTransform.Position, nextCell);

					// TO DO: Will particles leave the grid...

					// Check if particle has moved to another grid
					if (x != nextCell[0] || y != nextCell[1] || z != nextCell[2]) {
						mSPHGrid[nextCell[0] + (nextCell[1] * SPH_GRID_X) + (nextCell[2] * SPH_GRID_X * SPH_GRID_Y)].particles.push_back(cell.particles[p]);
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

void SPHFluidSimulation::updateParticlesForces()
{
	// For each SPHCell
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {
			
				int index = x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y);
				SPHCell& cell = mSPHGrid[index];
				vector<int>& neighborIndices = mSPHCellIndexMap.at(index);

				vec3 fPressure, fViscous, fSurface, fGravity;

				// Update each particle
				for (int p1 = 0; p1 < cell.particles.size(); p1++) {

					vec3 colorFieldNormal = vec3(0.0f);
					float colorFieldLaplacian = 0.0f;

					// Against all neighboring cell particles
					for (int nIndex : neighborIndices) {
						SPHCell& nCell = mSPHGrid[nIndex];
						for (int p2 = 0; p2 < nCell.particles.size(); p2++) {
							vec3 rDiff = cell.particles[p1].mTransform.Position - nCell.particles[p2].mTransform.Position;
							float rDistance = length(rDiff);
							float rDistance2 = rDistance * rDistance;
							if (rDistance2 <= SPH_CORE_RADIUS2) {
								// Pressure Force 
								float symmetricPressure = (cell.particles[p1].mPressure + nCell.particles[p2].mPressure) / (2 * nCell.particles[p2].mDensity);
								vec3 pGradient = SmoothKernelSpikyGradient(rDiff, rDistance, SPH_CORE_RADIUS);
								fPressure += cell.particles[p1].mRigidBody.mass * symmetricPressure * pGradient;

								// Viscous Force
								vec3 symmetricVelocity = (nCell.particles[p2].mRigidBody.velocity - cell.particles[p1].mRigidBody.velocity) / nCell.particles[p2].mDensity;
								float vGradient = SmoothKernelViscosityLaplacian(rDistance, SPH_CORE_RADIUS);
								fViscous += nCell.particles[p2].mRigidBody.mass * symmetricVelocity * vGradient;

								// Surface Force
								float coefficient = (nCell.particles[p2].mRigidBody.mass / nCell.particles[p2].mDensity);
								colorFieldNormal += coefficient * SmoothKernelPoly6Gradient(rDiff, rDistance2, SPH_CORE_RADIUS, SPH_CORE_RADIUS2);
								colorFieldLaplacian += coefficient * SmoothKernelPoly6Laplacian(rDistance2, SPH_CORE_RADIUS, SPH_CORE_RADIUS2);
							}
						}
					}	

					fPressure *= -1;
					fViscous *= WATER_VISCOSITY;
					fSurface = vec3(0.0f);
					fGravity = cell.particles[p1].mDensity * vec3(0.0f, GRAVITATIONAL_ACCELERATION, 0.0f);

					// Finish up force calculations
					float CFNLength = length(colorFieldNormal);
					if (CFNLength > COLOR_FIELD_THRESHOLD) {
						fSurface = -SURFACE_TENSION * colorFieldLaplacian * (colorFieldNormal / CFNLength);
					}

					cell.particles[p1].mAcceleration = (fPressure + fViscous + fSurface + fGravity) / cell.particles[p1].mDensity;
				}

			}
		}
	}

	applyBoundingVolumeForce();
}

void SPHFluidSimulation::updateParticlesPressureDensity()
{
	// For each SPHCell
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {

				int index = x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y);
				SPHCell& cell = mSPHGrid[index];
				vector<int>& neighborIndices = mSPHCellIndexMap.at(index);

				// Update each particle
				for (int p1 = 0; p1 < cell.particles.size(); p1++) {

					// Start density as value of particle compared with self
					float pDensity = -SmoothKernelPoly6(0, SPH_CORE_RADIUS, SPH_CORE_RADIUS2);

					// Against all neighboring cell particles
					for (int nIndex : neighborIndices) {
						SPHCell& nCell = mSPHGrid[nIndex];
						for (int p2 = 0; p2 < nCell.particles.size(); p2++) {
							vec3 rDiff = cell.particles[p1].mTransform.Position - nCell.particles[p2].mTransform.Position;
							float rDistance = length(rDiff);
							float rDistance2 = rDistance * rDistance;
							if (rDistance2 <= SPH_CORE_RADIUS2) {
								// Accumulate density from neighboring particles
								pDensity += cell.particles[p1].mRigidBody.mass * SmoothKernelPoly6(rDistance2, SPH_CORE_RADIUS, SPH_CORE_RADIUS2);
							}
						}
					}

					// Update particle with density and pressure
					cell.particles[p1].mPressure = WATER_VAPOR_CONSTANT * (pDensity - WATER_REST_DENSITY);
					cell.particles[p1].mDensity = pDensity;
				}
			}
		}
	}
}

void SPHFluidSimulation::applyBoundingVolumeForce()
{

}

void SPHFluidSimulation::stepSimulation(double secondsElapsed)
{
	double t = 0.0;
	double dt = 0.01;

	double accumulator = 0.0;

	double newTime = secondsElapsed;

	double frameTime = newTime - currentTime;
	if (frameTime > 0.25)
	{
		frameTime = 0.25;
	}

	currentTime = newTime;
	accumulator += frameTime;

	while (accumulator >= dt)
	{
		integrateCellParticles(dt);
		t += dt;
		accumulator -= dt;
	}

	const double alpha = accumulator / dt;
	// interpolate between previous phyics state and current physics state
}

void SPHFluidSimulation::integrateCellParticles(double deltaTime)
{
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {
				
				SPHCell& cell = mSPHGrid[x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y)];
				for (SPHParticle particle : cell.particles) {
					particle.mRigidBody.velocity += particle.mAcceleration * (float)deltaTime;
					particle.mTransform.Position += particle.mRigidBody.velocity * (float)deltaTime;
				}
			}
		}
	}
}

void SPHFluidSimulation::NormalizedGridPosition(vec3 worldPosition, int* indices)
{
	indices[0] = std::floor((worldPosition.x / SPH_CONTAINER_X) * SPH_GRID_X);
	indices[1] = std::floor((worldPosition.y / SPH_CONTAINER_Y) * SPH_GRID_Y);
	indices[2] = std::floor((-worldPosition.z / SPH_CONTAINER_Z) * SPH_GRID_Z);
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
