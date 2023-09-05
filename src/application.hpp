
#pragma once

// std
#include <chrono>

// glm
#include <glm/glm.hpp>

// project
#include "boid.hpp"
#include "opengl.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"

// main application class
class Application {
private:
	// window
	glm::vec2 m_windowsize;
	GLFWwindow *m_window;

	// oribital camera
	float m_pitch = 0;
	float m_yaw = 0;
	float m_distance = 50;
	const float m_max_distance = 200;

	// input
	glm::vec2 m_viewport_size;
	bool m_leftMouseDown = false;
	glm::vec2 m_mousePosition;

	// scene
	Scene m_scene;

	// time keeping
	float m_timescale = 1.0;
	bool m_pause = false;
	std::chrono::time_point<std::chrono::steady_clock> m_current_time;

public:
	// setup
	Application(GLFWwindow *);

	// disable copy constructors (for safety)
	Application(const Application&) = delete;
	Application& operator=(const Application&) = delete;

	// rendering callbacks (every frame)
	void render();
	void renderGUI();

	// input callbacks
	void cursorPosCallback(double xpos, double ypos);
	void mouseButtonCallback(int button, int action, int mods);
	void scrollCallback(double xoffset, double yoffset);
	void keyCallback(int key, int scancode, int action, int mods);
	void charCallback(unsigned int c);
};