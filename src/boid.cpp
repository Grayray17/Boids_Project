
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"

#include <glm/gtc/matrix_transform.hpp>


using namespace glm;
using namespace std;


vec3 Boid::color() const{
    return m_colour;
}
void Boid::updateColour(vec3 upCol){
    m_colour = upCol;
}
vec3 Boid::colour() {
    return m_colour;
}
float Boid::getRadius(){
    return m_radius;
}
void Boid::setRadius(float r){
    m_radius = r;
}

void Boid::calculateForces(Scene *scene) {
    if(m_colour != glm::vec3{0.5,0.5,0.5}){
        if(m_colour != glm::vec3 {1,0,0}){
            m_acceleration = {0,0,0};
            //-------------------------------------------------------------
            // [Assignment 3] :
            // Calculate the forces affecting the boid and update the
            // acceleration (assuming mass = 1).
            // Do NOT update velocity or position in this function.
            // Core :
            std::vector<Boid> boids = scene->boids();

            glm::vec3 average_position = {0,0,0};
            glm::vec3 average_velocity = {0,0,0};

            glm::vec3 cohesion = {0,0,0};
            glm::vec3 alignment = {0,0,0};
            glm::vec3 avoidance = {0,0,0};
            glm::vec3 predAvoidance = {0,0,0};
            glm::vec3 sphereAvoidance = {0,0,0};

            int cohesionNeighbours = 0;
            int alignmentNeighbours = 0;

            float closestSphere = -1;


            for(int i = 0; i < boids.size();i++){
                //  - Cohesion

                if(boids.at(i).position() != m_position && glm::length(m_position - boids.at(i).position()) < scene->local && m_colour == boids.at(i).colour() && boids.at(i).colour() != glm::vec3 {0.5,0.5,0.5}){
                    average_position += boids.at(i).position();
                    cohesionNeighbours += 1;
                }

                //  - Alignment
                if(boids.at(i).position() != m_position && glm::length(m_position - boids.at(i).position()) < scene->local && m_colour == boids.at(i).colour() && boids.at(i).colour() != glm::vec3 {0.5,0.5,0.5}){
                    average_velocity += boids.at(i).velocity();
                    alignmentNeighbours += 1;
                }

                //  - Avoidance
                if(boids.at(i).position() != m_position && glm::length(m_position - boids.at(i).position()) < scene->local && boids.at(i).colour() != glm::vec3 {0.5,0.5,0.5}){
                    avoidance += (m_position - boids.at(i).position()) / (glm::length(m_position - boids.at(i).position()) * glm::length(m_position - boids.at(i).position()));
                }

                // - Predator Avoidance
                if(boids.at(i).colour() == glm::vec3 {1,0,0} && glm::length(m_position - boids.at(i).position()) < scene->boidVision2P){
                    predAvoidance += (m_position - boids.at(i).position()) / (glm::length(m_position - boids.at(i).position()));
                }
                // - Sphere Avoidance
                if(boids.at(i).colour() == glm::vec3 {0.5,0.5,0.5} && glm::length(m_position - boids.at(i).position()) <= scene->boid2SphereVision){
                    float r = boids.at(i).getRadius();
                    float a = glm::dot(m_velocity,m_velocity);
                    float b = glm::dot(2.0f * (m_position - boids.at(i).position()),m_velocity);
                    float c = glm::dot(m_position - boids.at(i).position(),m_position - boids.at(i).position()) - (r*r);
                    float discriminant = glm::pow(b,2) - (4 * a * c);
                    if(discriminant >= 0){
                        float t1 = (-b + glm::sqrt(discriminant))/(2*a);
                        float t2 = (-b - glm::sqrt(discriminant))/(2*a);
                        if(t1 > 0 && t2 > 0){
                            glm::vec3 pt1 = m_position + (t1 * m_velocity);
                            glm::vec3 pt2 = m_position + (t2 * m_velocity);
                            glm::vec3 e = {pt1.x + pt2.x, pt1.y + pt2.y, pt1.z + pt2.z};
                            e /= 2.0f;
                            glm::vec3 f = (e - boids.at(i).position() )/ glm::length(e - boids.at(i).position());

                            //checking for center point of the sphere
                            if(e == boids.at(i).position()){
                                glm::mat4 rotationMatrix = glm::rotate( mat4(1.0f), 3.14f/2, glm::vec3(0, 1, 0));
                                glm::vec4 temp = {f.x,f.y,f.z,0};
                                temp = temp * rotationMatrix;
                                 f = {temp.x,temp.y,temp.z};
                            }
                            if(closestSphere < 0){
                                closestSphere = glm::length(m_position - boids.at(i).position());
                                sphereAvoidance += f;
                            }
                            else if(closestSphere > glm::length(m_position - boids.at(i).position())){
                                sphereAvoidance += f;
                            }
                        }
                    }
                }
            }
            if(cohesionNeighbours > 0){
                average_position /= cohesionNeighbours;
            }
            cohesion = (average_position - m_position) * scene->coh;

            if(alignmentNeighbours > 0){
                average_velocity /= alignmentNeighbours;
            }
            alignment = (average_velocity - m_velocity) * scene->allign;

            avoidance *= scene->avoid;
            predAvoidance *= scene->boidPredatorAvoidance;
            sphereAvoidance *= scene->boidSphereAvoidance;

            m_acceleration = cohesion + alignment + avoidance + predAvoidance + sphereAvoidance;
        }
        else{
            std::vector<Boid> boids = scene->boids();
            glm::vec3 chase = vec3(0);
            int indexTemp = -1;
            float closest = 100000.0f;
            for(int i = 0; i < boids.size();i++){
                if(boids.at(i).position() != m_position && glm::length(m_position - boids.at(i).position()) < scene->predatorVision && boids.at(i).colour() != m_colour){
                    if(glm::length(m_position - boids.at(i).position()) < closest){
                        indexTemp = i;
                        closest = glm::length(m_position - boids.at(i).position())
                                ;
                    }
                }
            }
            if(indexTemp > 0){
                chase = (boids.at(indexTemp).position() - m_position);
            }
            chase *= scene->predatorChase;
            m_acceleration = chase;
        }
    }

}


void Boid::update(float timestep, Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Integrate the velocity of the boid using the timestep.
	// Update the position of the boid using the new velocity.
	// Take into account the bounds of the scene which may
	// require you to change the velocity (if bouncing) or
	// change the position (if wrapping).
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

	//vector<Boid> m_b = scene->boids();

	glm::vec3 m_bb = scene->bound();
	    if(m_position.y < -m_bb.y){
	        m_velocity.y = -m_velocity.y;
	        m_position.y = -m_bb.y;
	    }
	    if(m_position.y > m_bb.y){
	        m_velocity.y = -m_velocity.y;
	        m_position.y = m_bb.y;
	    }
	    if(m_position.x > m_bb.x){
	        m_velocity.x = -m_velocity.x;
	        m_position.x = m_bb.x;
	    }
	    if(m_position.x < -m_bb.x){
	        m_velocity.x = -m_velocity.x;
	        m_position.x = -m_bb.x;
	    }
	    if(m_position.z > m_bb.z){
	        m_velocity.z = -m_velocity.z;
	        m_position.z = m_bb.z;
	    }
	    if(m_position.z < -m_bb.z){
	        m_velocity.z = -m_velocity.z;
	        m_position.z = -m_bb.z;
	    }


	    if(m_colour != glm::vec3 {1,0,0} && length(m_velocity) > scene->maxS ){
           m_velocity = normalize(m_velocity);
           m_velocity = m_velocity * scene->maxS;
        }
	    else if(m_colour != glm::vec3 {1,0,0} && length(m_velocity) < scene->minS ){
	        m_velocity = normalize(m_velocity);
	        m_velocity = m_velocity * scene->minS;
	    }

	    if(m_colour == glm::vec3 {1,0,0} && length(m_velocity) > scene->predatorMaxS ){
	        m_velocity = normalize(m_velocity);
	        m_velocity = m_velocity * scene->predatorMaxS;
	    }
	    else if(m_colour == glm::vec3 {1,0,0} && length(m_velocity) < scene->predatorMinS ){
	        m_velocity = normalize(m_velocity);
	        m_velocity = m_velocity * scene->predatorMinS;
	    }

	    if(m_colour != glm::vec3{0.5,0.5,0.5}){
	        m_position += m_velocity * timestep;
	    }



	    if(m_colour != glm::vec3 {1,0,0} && length(m_acceleration) > scene->maxA){
	        m_acceleration = normalize(m_acceleration);
	        m_acceleration = m_acceleration * scene->maxA;
	    }
//	    else if(length(m_acceleration) > scene->predatorMaxA){
//	        m_acceleration = normalize(m_acceleration);
//	        m_acceleration = m_acceleration * scene->predatorMaxA;
//	    }

	    m_velocity += m_acceleration * timestep;

}