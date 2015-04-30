#include <GL\glew.h>
#include <GLFW\glfw3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>
#include "SPHFluidSimulation.h"

int main()
{
	SPHFluidSimulation fluidSim = SPHFluidSimulation();
	Game::Launch(fluidSim);
	return 0;
}
