
#pragma once

// glm
#include <glm/glm.hpp>

// project
#include "scene.hpp"


class Boid {
private:

	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;
	glm::vec3 m_colour;
	float m_radius;

public:
	Boid(glm::vec3 pos, glm::vec3 dir, glm::vec3 boidC = {0,1,0}) : m_position(pos), m_velocity(dir), m_colour(boidC) { }

	glm::vec3 position() const { return m_position; }
	glm::vec3 velocity() const { return m_velocity; }
	glm::vec3 acceleration() const { return m_acceleration; }

	glm::vec3 color() const;
	glm::vec3 colour();

	float getRadius();
	void setRadius(float r);

    void updateColour(glm::vec3 colour);
	void calculateForces(Scene *scene);
	void update(float timestep, Scene *scene);
};