
// std
#include <random>

// stb
#include <stb_image.h>

// imgui
#include <imgui.h>

// glm
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

// project
#include "scene.hpp"
#include "boid.hpp"
#include "cgra/cgra_geometry.hpp"
#include "cgra/cgra_image.hpp"
#include "cgra/cgra_wavefront.hpp"


using namespace glm;
using namespace std;


Scene::Scene() {

	// load meshes
	cgra::mesh_builder simple_boid_md = cgra::load_wavefront_data(CGRA_WORKDIR + string("res/models/boid.obj"));
	m_simple_boid_mesh = simple_boid_md.build();

	cgra::mesh_builder boid_md = cgra::load_wavefront_data(CGRA_WORKDIR + string("res/models/spaceship_boid.obj"));
	m_boid_mesh = boid_md.build();

	cgra::mesh_builder predator_md = cgra::load_wavefront_data(CGRA_WORKDIR + string("res/models/predator_boid.obj"));
	m_predator_mesh = predator_md.build();

	cgra::mesh_builder sphere_md = cgra::load_wavefront_data(CGRA_WORKDIR + string("res/models/sphere.obj"));
	m_sphere_mesh = sphere_md.build();

	// load color shader
	cgra::shader_builder color_sp;
	color_sp.set_shader(GL_VERTEX_SHADER, CGRA_WORKDIR + string("res/shaders/simple_color.glsl"));
	color_sp.set_shader(GL_FRAGMENT_SHADER, CGRA_WORKDIR + string("res/shaders/simple_color.glsl"));
	m_color_shader = color_sp.build();

	// load aabb shader
	cgra::shader_builder aabb_sp;
	aabb_sp.set_shader(GL_VERTEX_SHADER, CGRA_WORKDIR + string("res/shaders/aabb.glsl"));
	aabb_sp.set_shader(GL_GEOMETRY_SHADER, CGRA_WORKDIR + string("res/shaders/aabb.glsl"));
	aabb_sp.set_shader(GL_FRAGMENT_SHADER, CGRA_WORKDIR + string("res/shaders/aabb.glsl"));
	m_aabb_shader = aabb_sp.build();

	// load skymap shader
	cgra::shader_builder skymap_sp;
	skymap_sp.set_shader(GL_VERTEX_SHADER, CGRA_WORKDIR + string("res/shaders/skymap.glsl"));
	skymap_sp.set_shader(GL_GEOMETRY_SHADER, CGRA_WORKDIR + string("res/shaders/skymap.glsl"));
	skymap_sp.set_shader(GL_FRAGMENT_SHADER, CGRA_WORKDIR + string("res/shaders/skymap.glsl"));
	m_skymap_shader = skymap_sp.build();
}


void Scene::loadCore() {
	//-------------------------------------------------------------
	// [Assignment 3] (Core) :
	// Initialize the scene with 100-300 boids in random locations
	// inside the current bound size.
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

	// this creates a boid with a random location in [-1, 1]^3 and random velocity (magnitude = 1)

	m_boids.clear();
	for(int i = 0; i < 100; i++){
	    m_boids.push_back(Boid(linearRand(vec3(-m_bound_hsize), vec3(m_bound_hsize)), sphericalRand(1.0)));
	}
}

void Scene::loadCompletion() {
	//-------------------------------------------------------------
	// [Assignment 3] (Completion) :
	// Initialize the scene with 2 different flocks of boids,
	// 75-150 in each flock, in random locations inside the current
	// bound size. Additionally include at least one Predator.
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...
	m_boids.clear();
	for(int i = 0; i < 75; i++){
	    m_boids.push_back(Boid(linearRand(vec3(-m_bound_hsize), vec3(m_bound_hsize)), sphericalRand(1.0)));
	}
	for(int i = 0; i < 150; i++){
	    m_boids.push_back(Boid(linearRand(vec3(-m_bound_hsize), vec3(m_bound_hsize)), sphericalRand(1.0)));
	    m_boids.at(i).updateColour({0,0,1});
	}

	for(int i = 0; i <2; i++){
	    m_boids.push_back(Boid(linearRand(vec3(-m_bound_hsize), vec3(m_bound_hsize)), sphericalRand(1.0)));
	    m_boids.at(i).updateColour({1,0,0});
	}

}


void Scene::loadChallenge() {
	//-------------------------------------------------------------
	// [Assignment 3] (Challenge) :
	// Initalize the scene with 100-300 boids in random locations
	// inside the current bound size. Additionally add at least
	// three spheres with a large radius inside the bounds.
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...
	m_boids.clear();
	for(int i = 0; i < 200; i++){
	    m_boids.push_back(Boid(linearRand(vec3(-m_bound_hsize), vec3(m_bound_hsize)), sphericalRand(1.0)));
	}
	for(int i = 0; i <3; i++){
	    m_boids.push_back(Boid(linearRand(vec3(-m_bound_hsize * 0.8f) , vec3(m_bound_hsize * 0.8f)), sphericalRand(1.0)));
	    m_boids.at(i).updateColour({0.5,0.5,0.5});
	    m_boids.at(i).setRadius(5);
	}

}


void Scene::update(float timestep) {
	for (Boid &b : m_boids) {
		b.calculateForces(this);
	}

	for (Boid &b : m_boids) {
		b.update(timestep, this);
	}
}


void Scene::draw(const mat4 &proj, const mat4 &view) {

	// draw skymap (magically)
	//
	if (m_show_skymap) {
		static GLuint tex = 0;
		if (!tex) {
			tex = cgra::rgba_image(CGRA_WORKDIR + string("res/textures/sky.jpg")).uploadTexture();
		}
		glUseProgram(m_skymap_shader);
		glUniformMatrix4fv(glGetUniformLocation(m_skymap_shader, "uProjectionMatrix"), 1, false, value_ptr(proj));
		glUniformMatrix4fv(glGetUniformLocation(m_skymap_shader, "uModelViewMatrix"), 1, false, value_ptr(view));
		glUniform1f(glGetUniformLocation(m_skymap_shader, "uZDistance"), 1000.0f);
		glActiveTexture(GL_TEXTURE0); // Set the location for binding the texture
		glBindTexture(GL_TEXTURE_2D, tex); // Bind the texture
		glUniform1i(glGetUniformLocation(m_skymap_shader, "uSkyMap"), 0);  // Set our sampler (texture0) to use GL_TEXTURE0 as the source
		cgra::draw_dummy(12);
	}

	// draw axis (magically)
	//
	if (m_show_axis) {
		cgra::drawAxis(view, proj);
	}

	// draw the aabb (magically)
	//
	if (m_show_aabb) {
		glUseProgram(m_aabb_shader);
		glUniformMatrix4fv(glGetUniformLocation(m_aabb_shader, "uProjectionMatrix"), 1, false, value_ptr(proj));
		glUniformMatrix4fv(glGetUniformLocation(m_aabb_shader, "uModelViewMatrix"), 1, false, value_ptr(view));
		glUniform3fv(glGetUniformLocation(m_aabb_shader, "uColor"), 1, value_ptr(vec3(0.8, 0.8, 0.8)));
		glUniform3fv(glGetUniformLocation(m_aabb_shader, "uMax"), 1, value_ptr(m_bound_hsize));
		glUniform3fv(glGetUniformLocation(m_aabb_shader, "uMin"), 1, value_ptr(-m_bound_hsize));
		cgra::draw_dummy(12);
	}

	// draw boids
	//
	for (const Boid &b : m_boids) {

		// get the boid direction (default to z if no velocity)
		vec3 dir = normalize(b.velocity());
		if (dir.x != dir.x) dir = vec3(0, 0, 1);

		// calculate the model matrix
		mat4 model(1);
        //scale
        if(b.color() == glm::vec3{0.5,0.5,0.5}){
            model = scale(mat4(1), vec3(3,3,3)) * model;
        }

		// rotate the model to point it in the direction of its velocity

		// pitch rotation
		if (dir.y != 0) {
			float angle = -asin(dir.y);
			model = rotate(mat4(1), angle, vec3(1, 0, 0)) * model;
		}

		// yaw rotation
		if (dir.x != 0 || dir.z != 0) {
			float angle = atan2(dir.x, dir.z);
			model = rotate(mat4(1), angle, vec3(0, 1, 0)) * model;
		}

		// translate the model to its worldspace position

		// translate by m_position
		model = translate(mat4(1), b.position()) * model;

		// calculate the modelview matrix
		mat4 modelview = view * model;

		// load shader and variables
		glUseProgram(m_color_shader);
		glUniformMatrix4fv(glGetUniformLocation(m_color_shader, "uProjectionMatrix"), 1, false, value_ptr(proj));
		glUniformMatrix4fv(glGetUniformLocation(m_color_shader, "uModelViewMatrix"), 1, false, value_ptr(modelview));
		glUniform3fv(glGetUniformLocation(m_color_shader, "uColor"), 1, value_ptr(b.color()));

		// draw
		if(b.color() == glm::vec3{1,0,0}){
		    m_predator_mesh.draw();
		}
		else if(b.color() == glm::vec3{0.5,0.5,0.5}){
		    m_sphere_mesh.draw();
		}
		else{
		    m_boid_mesh.draw();
		}


	}
}


void Scene::renderGUI() {

	if (ImGui::Button("Core", ImVec2(80, 0))) { loadCore(); }
	ImGui::SameLine();
	if (ImGui::Button("Completion", ImVec2(80, 0))) { loadCompletion(); }
	ImGui::SameLine();
	if (ImGui::Button("Challenge", ImVec2(80, 0))) { loadChallenge(); }

	ImGui::Checkbox("Draw Bound", &m_show_aabb);
	ImGui::Checkbox("Draw Axis", &m_show_axis);
	ImGui::Checkbox("Draw Skybox", &m_show_skymap);

	//-------------------------------------------------------------
	// [Assignment 3] :
	// Add ImGui sliders for controlling boid parameters :
	// Core :
	// - boid min speed
	// - boid max speed
	// - boid max acceleration
	// - boid neighbourhood size
	// - boid cohesion weight
	// - boid alignment weight
	// - boid avoidance weight
	// - bounding mode (optional)
	// Completion :
	// - boid anti-predator weight
	// - predator min speed
	// - predator max speed
	// - predator max acceleration
	// - predator neighbourhood size
	// - predator avoidance weight
	// - predator chase weight
	//-------------------------------------------------------------
	
	ImGui::SliderFloat3("Bound hsize", value_ptr(m_bound_hsize), 0, 100.0, "%.0f");

	// YOUR CODE GOES HERE
	// ...

}