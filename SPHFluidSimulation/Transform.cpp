#include "Transform.h"


Transform::Transform(vec3 position, vec3 rotation, vec3 scale) : 
	Position(position),
	Rotation(rotation),
	Scale(scale),
	Forward({ 0.0f, 0.0f, 1.0f }),
	Right({ 1.0f, 0.0f, 0.0f }),
	Up({ 0.0f, 1.0f, 0.0f }) {}

Transform::Transform() : Transform(vec3(0.0f), vec3(0.0f), vec3(1.0f)) {}

Transform::Transform(const Transform& other): 
	Position(other.Position),
	Rotation(other.Rotation),
	Scale(other.Scale),
	Forward(other.Forward),
	Right(other.Right),
	Up(other.Up) {}

Transform::~Transform() {}

void Transform::MoveForward()
{
	vec3 worldForward = (mat3)GetRotationTransform() * Forward;
	Position += worldForward;
}

void Transform::MoveBackward()
{
	vec3 worldForward = (mat3)GetRotationTransform() * Forward;
	Position -= worldForward;
}
void Transform::MoveRight()
{
	mat3 worldRotation = (mat3)GetRotationTransform();
	vec3 worldForward = worldRotation * Forward;
	vec3 worldUp = worldRotation * Up;
	Position += cross(worldForward, worldUp);
}

void Transform::MoveLeft()
{
	mat3 worldRotation = (mat3)GetRotationTransform();
	vec3 worldForward = worldRotation * Forward;
	vec3 worldUp = worldRotation * Up;
	Position -= cross(worldForward, worldUp);
}

void Transform::MoveUp()
{
	vec3 worldUp = (mat3)GetRotationTransform() * Up;
	Position += worldUp;
}

void Transform::MoveDown()
{
	vec3 worldUp = (mat3)GetRotationTransform() * Up;
	Position -= worldUp;
}

mat4 Transform::GetWorldTransform()
{
	mat4 identity = mat4(1.0f);
	return translate(identity, Position) *
			GetRotationTransform() *
			glm::scale(identity, Scale);
}

mat4 Transform::GetRotationTransform()
{
	mat4 identity = mat4(1.0f);
	return rotate(identity, Rotation.z, vec3(0, 0, 1)) *
			rotate(identity, Rotation.y, vec3(0, 1, 0)) *
			rotate(identity, Rotation.x, vec3(1, 0, 0));
}