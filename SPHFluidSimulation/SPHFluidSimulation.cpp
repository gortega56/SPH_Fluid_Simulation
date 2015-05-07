#include "SPHFluidSimulation.h"
#include <glm/gtx/norm.hpp>

#define _USE_MATH_DEFINES 
#include <math.h>

SPHFluidSimulation::SPHFluidSimulation() : mGameObjectCount(PARTICLE_COUNT), mColliderCount(PARTICLE_COUNT), mTransformCount(PARTICLE_COUNT), mMeshCount(MESH_COUNT), mRigidBodyCount(PARTICLE_COUNT), mGeometryCount(GEOMETRY_COUNT)
{
	// Missing GameObjects for now
	mColliders = new Collider[mColliderCount];
	mMesh = new Mesh[mMeshCount];
	mRigidBodies = new RigidBody[mRigidBodyCount];
	mGeometry = new Geometry[mGeometryCount];
	
	mSPHGrid = new SPHCell[SPH_GRID_X * SPH_GRID_Y * SPH_GRID_Z];
	mBoundingBox = new GameObject();

	mCamera = new Camera();
}


SPHFluidSimulation::~SPHFluidSimulation()
{
	delete[] mColliders;
	delete[] mMesh;
	delete[] mRigidBodies;
	delete[] mGeometry;
}

void SPHFluidSimulation::init()
{
	Game::init();
	initParticleGrid();
	initGeometry();
	
	SPHCell& cell = mSPHGrid[0];
	for (int i = 0; i < PARTICLE_COUNT; i++) {
		SPHParticle particle = SPHParticle();
		particle.mTransform.Position = vec3(SPH_CONTAINER_X - i, SPH_CONTAINER_Y, -SPH_CONTAINER_Z);
		cell.particles.push_back(particle);
	}
}

void SPHFluidSimulation::updateScene(double secondsElapsed)
{
	//updateParticleGrid();
	//updateParticlesPressureDensity();
	//updateParticlesForces();
	//stepSimulation(secondsElapsed);
	updateTransforms();
}

void SPHFluidSimulation::renderScene(double secondsElapsed)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glUseProgram(mBoundingBox->material->ShaderID);

	glm::mat4 projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.f);
	glm::mat4 view = mCamera->GetView();

	glm::mat4 PVM = projection * view * mBoundingBox->transform->GetWorldTransform();
	glPolygonMode(GL_FRONT_AND_BACK, mBoundingBox->mesh->PolygonMode);
	glBindVertexArray(mBoundingBox->material->VertexArrayID);
	glUniformMatrix4fv(mBoundingBox->material->GetUniformAttribute("clip"), 1, GL_FALSE, &PVM[0][0]);
	glDrawArrays(mBoundingBox->mesh->RenderMode, 0, mBoundingBox->mesh->GetVertexCount());

	glUseProgram(mSphereMaterial->ShaderID);
	glPolygonMode(GL_FRONT_AND_BACK, mSphere->PolygonMode);
	glBindVertexArray(mSphereMaterial->VertexArrayID);
	glUniformMatrix4fv(mSphereMaterial->GetUniformAttribute("view"), 1, GL_FALSE, &view[0][0]);
	glUniformMatrix4fv(mSphereMaterial->GetUniformAttribute("projection"), 1, GL_FALSE, &projection[0][0]);
	glDrawArraysInstanced(GL_TRIANGLES, 0, mSphere->GetVertexCount(), PARTICLE_COUNT);
}


void SPHFluidSimulation::handleEvents(GLFWwindow* window)
{

}

void SPHFluidSimulation::initGeometry()
{
	// Bounding Volume
	Geometry *cube = Geometry::Cube();
	mCube = new Mesh(*cube);
	mCube->RenderMode = GL_TRIANGLES;
	mCube->PolygonMode = GL_LINE;

	mCubeMaterial = new Material();
	mCubeMaterial->SetShaderStage("shaders/TextureVertexShader.glsl", GL_VERTEX_SHADER);
	mCubeMaterial->SetShaderStage("shaders/TextureFragmentShader.glsl", GL_FRAGMENT_SHADER);
	mCubeMaterial->SetShader();

	mCubeMaterial->BindMeshAttributes(*mCube, "vertexPosition", NULL, NULL, "texCoord", NULL);
	mCubeMaterial->BindUniformAttribute("clip");

	mBoundingBox->geometry = cube;
	mBoundingBox->mesh = mCube;
	mBoundingBox->material = mCubeMaterial;
	mBoundingBox->transform = new Transform();
	mBoundingBox->transform->Scale = { SPH_CONTAINER_X, SPH_CONTAINER_Y, SPH_CONTAINER_Z };
	mBoundingBox->transform->Position = { SPH_CONTAINER_X / 2, SPH_CONTAINER_Y / 2, -SPH_CONTAINER_Z / 2 };

	// Particle
	Geometry *sphere = Geometry::Sphere(3, 3);
	mSphere = new Mesh(*sphere);
	mSphere->RenderMode = GL_TRIANGLES;
	mSphere->PolygonMode = GL_FILL;

	mSphereMaterial = new Material();
	mSphereMaterial->SetShaderStage("shaders/InstanceVertexShader.glsl", GL_VERTEX_SHADER);
	mSphereMaterial->SetShaderStage("shaders/FragmentShader.glsl", GL_FRAGMENT_SHADER);
	mSphereMaterial->SetShader();

	mSphereMaterial->BindMeshAttributes(*mSphere, "vertexPosition", NULL, NULL, NULL, NULL);
	mSphereMaterial->BindUniformAttribute("projection");
	mSphereMaterial->BindUniformAttribute("view");

	GLuint modelAttribID = glGetAttribLocation(mSphereMaterial->ShaderID, "model");
	glGenBuffers(1, &mTransformBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, mTransformBufferID);
	glBufferData(GL_ARRAY_BUFFER, PARTICLE_COUNT * sizeof(mat4), NULL, GL_DYNAMIC_DRAW);
	for (int i = 0; i < 4; i++) {
		glVertexAttribPointer(modelAttribID + i, 4, GL_FLOAT, GL_FALSE, sizeof(mat4), (void*)(sizeof(vec4) * i));
		glEnableVertexAttribArray(modelAttribID + i);
		glVertexAttribDivisor(modelAttribID + i, 1);
	}	

	mCamera->transform->Position.x = SPH_CONTAINER_X / 2;
	mCamera->transform->Position.y = SPH_CONTAINER_Y / 2;
}

void SPHFluidSimulation::updateTransforms()
{
	glBindBuffer(GL_ARRAY_BUFFER, mTransformBufferID);
	mat4* matrices = (mat4*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

	int gridsCount = SPH_GRID_X * SPH_GRID_Y * SPH_GRID_Z;
	int particleCount = 0;
	for (int i = 0; i < gridsCount; i++) {
		SPHCell& cell = mSPHGrid[i];
		for (int p = 0; p < cell.particles.size(); p++) {
			cell.particles[p].mTransform.Position.y -= 0.001f * p;
			matrices[particleCount++] = cell.particles[p].mTransform.GetWorldTransform();
			printf("POS: %f, %f, %f \n", cell.particles[p].mTransform.Position.x, cell.particles[p].mTransform.Position.y, cell.particles[p].mTransform.Position.z);
		}
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
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
					nextCell[0] = clamp(nextCell[0], 0, SPH_GRID_X - 1);
					nextCell[1] = clamp(nextCell[1], 0, SPH_GRID_Y - 1);
					nextCell[2] = clamp(nextCell[2], 0, SPH_GRID_Z - 1);
					// Check if particle has moved to another grid
					if (x != nextCell[0] || y != nextCell[1] || z != nextCell[2]) {
						mSPHGrid[nextCell[0] + (nextCell[1] * SPH_GRID_X) + (nextCell[2] * SPH_GRID_X * SPH_GRID_Y)].particles.push_back(cell.particles[p]);
						indicesToRemove.push_back(p);
					}
				}

				for (int index = indicesToRemove.size() - 1; index >= 0; index--) {
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
							if (0 < rDistance2 && rDistance2 <= SPH_CORE_RADIUS2) {
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
					cell.particles[p1].mAcceleration = vec3(0, GRAVITATIONAL_ACCELERATION, 0);

				//	cell.particles[p1].mAcceleration = (fPressure + fViscous + fSurface + fGravity) / cell.particles[p1].mDensity;
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
					printf("ACCL: %f %f %f\n", particle.mAcceleration.x, particle.mAcceleration.y, particle.mAcceleration.z);
					//printf("POS: %f %f %f\n", particle.mTransform.Position.x, particle.mTransform.Position.y, particle.mTransform.Position.z);
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
