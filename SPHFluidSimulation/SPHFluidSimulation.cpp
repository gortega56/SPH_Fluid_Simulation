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
	Play = false;
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
		particle.mTransform.Position = vec3(SPH_CONTAINER_X - (i * 0.5f), SPH_CONTAINER_Y, -SPH_CONTAINER_Z);
		cell.particles.push_back(particle);
	}
}

void SPHFluidSimulation::updateScene(double secondsElapsed)
{
	updateParticleGrid();
	updateParticlesPressureDensity();
	updateParticlesForces();
	stepSimulation(secondsElapsed);
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
	static bool keyDown;
	if (glfwGetKey(window, GLFW_KEY_W)) {
		mCamera->transform->MoveForward();
	}
	else if (glfwGetKey(window, GLFW_KEY_A)) {
		mCamera->transform->MoveLeft();
	}
	else if (glfwGetKey(window, GLFW_KEY_S)) {
		mCamera->transform->MoveBackward();
	}
	else if (glfwGetKey(window, GLFW_KEY_D)) {
		mCamera->transform->MoveRight();
	}
	else if (glfwGetKey(window, GLFW_KEY_P)) {
		if (!keyDown) {
			Play = !Play;
		}
	}
	else {
		keyDown = false;
	}

	static double pMousePosX, pMousePosY;
	double mousePosX, mousePosY;
	glfwGetCursorPos(window, &mousePosX, &mousePosY);

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
		mCamera->transform->Rotation.x += radians(pMousePosY - mousePosY);
		mCamera->transform->Rotation.y += radians(pMousePosX - mousePosX);
	}

	pMousePosX = mousePosX;
	pMousePosY = mousePosY;
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

void SPHFluidSimulation::initParticleGrid()
{
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {
				int index = SPHGridCellIndex(x, y, z);
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

							nIndices.push_back(SPHGridCellIndex(x + offsetX, y + offsetY, z + offsetZ));
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
				SPHCell& cell = mSPHGrid[SPHGridCellIndex(x, y, z)];

				vector<int> indicesToRemove;
				for (int p = 0; p < cell.particles.size(); p++) {
					int nextCell[3];
					printf("C%i P%i pPOS: %i %i %i\n",SPHGridCellIndex(x, y, z) , p, x, y, z);
					NormalizedGridPosition(cell.particles[p].mTransform.Position, nextCell);
					printf("C%i P%i nPOS: %i %i %i\n", SPHGridCellIndex(x, y, z), p, nextCell[0], nextCell[1], nextCell[2]);
					// TO DO: Will particles leave the grid...
					nextCell[0] = clamp(nextCell[0], 0, SPH_GRID_X - 1);
					nextCell[1] = clamp(nextCell[1], 0, SPH_GRID_Y - 1);
					nextCell[2] = clamp(nextCell[2], 0, SPH_GRID_Z - 1);
					// Check if particle has moved to another grid
					if (x != nextCell[0] || y != nextCell[1] || z != nextCell[2]) {
						mSPHGrid[SPHGridCellIndex(nextCell[0], nextCell[1], nextCell[2])].particles.push_back(cell.particles[p]);
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

void SPHFluidSimulation::updateParticlesPressureDensity()
{
	// For each SPHCell
	for (int x = 0; x < SPH_GRID_X; x++) {
		for (int y = 0; y < SPH_GRID_Y; y++) {
			for (int z = 0; z < SPH_GRID_Z; z++) {

				int index = SPHGridCellIndex(x, y, z);
				SPHCell& cell = mSPHGrid[index];
				vector<int>& neighborIndices = mSPHCellIndexMap.at(index);

				// Update each particle
				for (int p1 = 0; p1 < cell.particles.size(); p1++) {

					// Start density as value of particle compared with self
					float pDensity = 0.0f;

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

					printf("C%i P%i D: %f\n", index, p1, cell.particles[p1].mDensity);
					printf("C%i P%i P: %f\n", index, p1, cell.particles[p1].mPressure);
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
			
				int index = SPHGridCellIndex(x, y, z);
				SPHCell& cell = mSPHGrid[index];
				vector<int>& neighborIndices = mSPHCellIndexMap.at(index);

				

				// Update each particle
				for (int p1 = 0; p1 < cell.particles.size(); p1++) {
					
					vec3 fPressure, fViscous, fSurface, fGravity;
					vec3 colorFieldNormal = vec3(0.0f);
					float colorFieldLaplacian = 0.0f;

					// Against all neighboring cell particles
					for (int nIndex : neighborIndices) {
						SPHCell& nCell = mSPHGrid[nIndex];
						for (int p2 = 0; p2 < nCell.particles.size(); p2++) {
							
							vec3 rDiff = cell.particles[p1].mTransform.Position - nCell.particles[p2].mTransform.Position;
							float rDistance = length(rDiff);
							float rDistance2 = rDistance * rDistance;
							bool skip = (index == nIndex && p1 == p2);
							
							if (!skip && rDistance2 <= SPH_CORE_RADIUS2) {
								// Pressure Force 
								float symmetricPressure = (cell.particles[p1].mPressure + nCell.particles[p2].mPressure) / (2 * nCell.particles[p2].mDensity);
								vec3 pGradient = SmoothKernelSpikyGradient(rDiff, rDistance, SPH_CORE_RADIUS);
								fPressure += cell.particles[p1].mRigidBody.mass * symmetricPressure * pGradient;

								// Viscous Force
								vec3 symmetricVelocity = (nCell.particles[p2].mRigidBody.velocity - cell.particles[p1].mRigidBody.velocity) / nCell.particles[p2].mDensity;
								printf("C%i P%i Vel: %f %f %f\n", index, p1, cell.particles[p1].mRigidBody.velocity.x, cell.particles[p1].mRigidBody.velocity.y, cell.particles[p1].mRigidBody.velocity.z);
								printf("C%i P%i Vel: %f %f %f\n", nIndex, p2, nCell.particles[p2].mRigidBody.velocity.x, nCell.particles[p2].mRigidBody.velocity.y, nCell.particles[p2].mRigidBody.velocity.z);

								
								float vGradient = SmoothKernelViscosityLaplacian(rDistance, SPH_CORE_RADIUS);
								fViscous += nCell.particles[p2].mRigidBody.mass * symmetricVelocity * vGradient;
							}
							
							// Surface Force
							float coefficient = (nCell.particles[p2].mRigidBody.mass / nCell.particles[p2].mDensity);
							colorFieldNormal += coefficient * SmoothKernelPoly6Gradient(rDiff, rDistance2, SPH_CORE_RADIUS, SPH_CORE_RADIUS2);
							printf("CFN: %f\n", length(colorFieldNormal));
							colorFieldLaplacian += coefficient * SmoothKernelPoly6Laplacian(rDistance2, SPH_CORE_RADIUS, SPH_CORE_RADIUS2);
							
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
				//	printf("C%i P%i fP: %f %f %f\n", index, p1, fPressure.x, fPressure.y, fPressure.z);
					//printf("C%i P%i fV: %f %f %f\n", index, p1, fViscous.x, fViscous.y, fViscous.z);

					if (length(fSurface) > 0.0) {
						printf("C%i P%i fS: %f %f %f\n", index, p1, fSurface.x, fSurface.y, fSurface.z);

					}
			//		printf("C%i P%i fG: %f %f %f\n", index, p1, fGravity.x, fGravity.y, fGravity.z);

					cell.particles[p1].mAcceleration = (fPressure + fViscous + fSurface + fGravity) / cell.particles[p1].mDensity;
					applyBoundingVolumeForce(cell.particles[p1]);
				}
			}
		}
	}


}

void SPHFluidSimulation::applyBoundingVolumeForce(SPHParticle& particle)
{
	// Check if the object is outside of the box AND still continuing in the same direction
	vec3 boundingBoxMin = vec3(0.0f, 0.0f, -SPH_CONTAINER_Z);
	vec3 boundingBoxMax = vec3(SPH_CONTAINER_X, SPH_CONTAINER_Y, 0.0f);

	if (particle.mTransform.Position.x < boundingBoxMin.x && particle.mRigidBody.velocity.x < 0) {
		particle.mRigidBody.velocity.x *= -SPH_CONTAINER_DAMPING;
	}
	else if (particle.mTransform.Position.x > boundingBoxMax.x && particle.mRigidBody.velocity.x > 0) {
		particle.mRigidBody.velocity.x *= -SPH_CONTAINER_DAMPING;
	}

	if (particle.mTransform.Position.y < boundingBoxMin.y && particle.mRigidBody.velocity.y < 0) {
		particle.mRigidBody.velocity.y *= -SPH_CONTAINER_DAMPING;
	}
	else if (particle.mTransform.Position.y > boundingBoxMax.y && particle.mRigidBody.velocity.y > 0) {
		particle.mRigidBody.velocity.y *= -SPH_CONTAINER_DAMPING;
	}

	if (particle.mTransform.Position.z < boundingBoxMin.z && particle.mRigidBody.velocity.z < 0) {
		particle.mRigidBody.velocity.z *= -SPH_CONTAINER_DAMPING;
	}
	else if (particle.mTransform.Position.z > boundingBoxMax.z && particle.mRigidBody.velocity.z > 0) {
		particle.mRigidBody.velocity.z *= -SPH_CONTAINER_DAMPING;
	}
}

void SPHFluidSimulation::stepSimulation(double secondsElapsed)
{
	if (!Play) {
		return;
	}

	double t = 0.0;
	double dt = 0.01;

	double accumulator = 0.0;

	double newTime = secondsElapsed;

	double frameTime = newTime - currentTime;
	if (frameTime > 0.01)
	{
		frameTime = 0.01;
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
	int SPHGridCellCount = SPH_GRID_X * SPH_GRID_Y * SPH_GRID_Z;
	for (int i = 0; i < SPHGridCellCount; i++) {	
		SPHCell& cell = mSPHGrid[i];
		for (int p = 0; p < cell.particles.size(); p++) {
					
			cell.particles[p].mRigidBody.velocity += cell.particles[p].mAcceleration * (float)deltaTime; // *(0.00001f);
			cell.particles[p].mTransform.Position += cell.particles[p].mRigidBody.velocity * (float)deltaTime;
			printf("C%i P%i A: %f %f %f\n", i, p, cell.particles[p].mAcceleration.x, cell.particles[p].mAcceleration.y, cell.particles[p].mAcceleration.z);
			printf("C%i P%i POS: %f %f %f\n", i, p, cell.particles[p].mTransform.Position.x, cell.particles[p].mTransform.Position.y, cell.particles[p].mTransform.Position.z);
		}
	}
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
			matrices[particleCount++] = cell.particles[p].mTransform.GetWorldTransform();
		}
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
}

void SPHFluidSimulation::NormalizedGridPosition(vec3 worldPosition, int* indices)
{
	indices[0] = std::floor((worldPosition.x / SPH_CONTAINER_X) * (SPH_GRID_X - 1));
	indices[1] = std::floor((worldPosition.y / SPH_CONTAINER_Y) * (SPH_GRID_Y - 1));
	indices[2] = std::floor((-worldPosition.z / SPH_CONTAINER_Z) * (SPH_GRID_Z - 1));
}

int	SPHFluidSimulation::SPHGridCellIndex(int x, int y, int z)
{
	return x + (y * SPH_GRID_X) + (z * SPH_GRID_X * SPH_GRID_Y);
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
