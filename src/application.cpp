
// std
#include <iostream>
#include <string>

// glm
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// project
#include "application.hpp"
#include "cgra/cgra_gui.hpp"
#include "cgra/cgra_shader.hpp"


using namespace std;
using namespace cgra;


Application::Application(GLFWwindow *window) : m_window(window) {
	// set time
	m_current_time = chrono::steady_clock::now();
}


void Application::render() {
	
	// Update

	chrono::time_point<chrono::steady_clock> now = chrono::steady_clock::now();
	double time_delta = (now - m_current_time) / 1.0s; // in seconds
	m_current_time = now;
	if (!m_pause)
		m_scene.update(float(time_delta) * m_timescale);

	// Draw

	// retrieve the window hieght
	int width, height;
	glfwGetFramebufferSize(m_window, &width, &height);

	// record current draw size
	m_viewport_size = glm::vec2(width, height);

	// ensure we draw to the entire 
	glViewport(0, 0, width, height);

	// clear the back-buffer to a solid grey/blueish background
	glClearColor(0.3f, 0.3f, 0.4f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// enable flags for normal/forward rendering
	glEnable(GL_DEPTH_TEST); 
	glDepthFunc(GL_LESS);

	// calculate the projection and view matrix
	glm::mat4 proj = glm::perspective(1.0f, float(width) / height, 1.0f, 2000.0f);
	glm::mat4 view(1);
	view = glm::translate(view, glm::vec3(0, 0, -m_distance));
	view = glm::rotate(view, m_pitch, glm::vec3(1, 0, 0));
	view = glm::rotate(view, m_yaw, glm::vec3(0, 1, 0));

	m_scene.draw(proj, view);
}


void Application::renderGUI() {

	// setup window
	ImGui::SetNextWindowPos(ImVec2(5, 5), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ImVec2(350, 500), ImGuiSetCond_Once);
	ImGui::Begin("Options", 0);

	// display camera parameters
	if (ImGui::TreeNode("Camera")) {
		ImGui::Text("Application %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::SliderFloat("Pitch", &m_pitch, -glm::pi<float>() / 2, glm::pi<float>() / 2);
		ImGui::SliderFloat("Yaw", &m_yaw, -glm::pi<float>(), glm::pi<float>());
		ImGui::SliderFloat("Distance", &m_distance, 0, m_max_distance, "%.2f", 2.0f);

		ImGui::TreePop();
	}

	// simulation parameters
	if (ImGui::TreeNode("Simulation")) {
		ImGui::SliderFloat("Timescale", &m_timescale, 0.0, 100.0, "%.2f", 3.0f);
		if (ImGui::Button("Reset", ImVec2(100, 0))) m_timescale = 1.0;
		ImGui::SameLine();
		if (m_pause) {
			if (ImGui::Button("Start", ImVec2(100, 0)))
				m_pause = false;
		} 
		else {
			if (ImGui::Button("Pause", ImVec2(100, 0)))
				m_pause = true;
		}

		ImGui::TreePop();
	}

	// internal scene parameters
	if (ImGui::TreeNode("Scene")) {
		m_scene.renderGUI();
		ImGui::TreePop();
	}
	if(ImGui::TreeNode("Boid")){
	    ImGui::SliderFloat("Local", &m_scene.local,0,30);
	    ImGui::SliderFloat("Cohesion", &m_scene.coh,0,3);
	    ImGui::SliderFloat("Allignment", &m_scene.allign,0,3);
	    ImGui::SliderFloat("Avoidance", &m_scene.avoid,0,3);
	    ImGui::SliderFloat("min Speed", &m_scene.minS,0,10);
	    ImGui::SliderFloat("max Speed", &m_scene.maxS,1,20);
	    ImGui::SliderFloat("max Acceleration", &m_scene.maxA,1,20);
	    ImGui::SliderFloat("Predator Avoidance", &m_scene.boidPredatorAvoidance,1,100);
	    ImGui::SliderFloat("Boid Vision", &m_scene.boidVision2P,1,10);
	    ImGui::SliderFloat("Sphere Avoidance", &m_scene.boidSphereAvoidance,1,100);
	    ImGui::SliderFloat("Sphere Vision", &m_scene.boid2SphereVision,1,100);
	    ImGui::TreePop();
	}
	if(ImGui::TreeNode("Predator")){
	    ImGui::SliderFloat("Vision", &m_scene.predatorVision,0,20);
	    ImGui::SliderFloat("Chase", &m_scene.predatorChase,0,10);
	    ImGui::SliderFloat("max Speed", &m_scene.predatorMaxS,1,20);
	    ImGui::SliderFloat("min Speed", &m_scene.predatorMinS,0,20);
	    ImGui::SliderFloat("max Acceleration", &m_scene.predatorMaxA,1,20);
	    ImGui::TreePop();
	}

	
	// finish creating window
	ImGui::End();
}


void Application::cursorPosCallback(double xpos, double ypos) {
	if (m_leftMouseDown) {
		glm::vec2 viewport_hsize = m_viewport_size / 2.0f;

		// clamp the pitch to [-pi/2, pi/2]
		m_pitch += glm::acos(glm::clamp((m_mousePosition.y - viewport_hsize.y) / viewport_hsize.y, -1.0f, 1.0f))
			- glm::acos(glm::clamp((float(ypos) - viewport_hsize.y) / viewport_hsize.y, -1.0f, 1.0f));
		m_pitch = glm::clamp(m_pitch, -glm::pi<float>() / 2, glm::pi<float>() / 2);

		// wrap the yaw to [-pi, pi]
		m_yaw += glm::acos(glm::clamp((m_mousePosition.x - viewport_hsize.x) / viewport_hsize.x, -1.0f, 1.0f))
			- glm::acos(glm::clamp((float(xpos) - viewport_hsize.x) / viewport_hsize.x, -1.0f, 1.0f));
		if (m_yaw > glm::pi<float>()) m_yaw -= 2* glm::pi<float>();
		else if (m_yaw < -glm::pi<float>()) m_yaw += 2* glm::pi<float>();
	}
	m_mousePosition = glm::vec2(xpos, ypos);
}


void Application::mouseButtonCallback(int button, int action, int mods) {
	(void)mods;

	if (button == GLFW_MOUSE_BUTTON_LEFT)
		m_leftMouseDown = (action == GLFW_PRESS); // only other option is GLFW_RELEASE
}


void Application::scrollCallback(double xoffset, double yoffset) {
	(void)xoffset;

	// logrithmic zoom
	float d = m_max_distance  * pow(pow(m_distance / m_max_distance, 0.5f) - (float(yoffset) / 50.0f), 2.0f);
	if (d == d && !isinf(d))
		m_distance = glm::clamp(d, 0.0f, m_max_distance);
}


void Application::keyCallback(int key, int scancode, int action, int mods) {
	(void)key, scancode, action, mods;
}


void Application::charCallback(unsigned int c) {
	(void)c;
}