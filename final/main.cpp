#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>
#include "glm/glm.hpp"


#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265
using namespace std;

//****************************************************
// Some Classes
//****************************************************

class Particle {
  public:
    Particle(){};
    glm::vec3 position; 
    glm::vec3 velocity;
    glm::vec3 pressure;
    float density;
    float cfLap;
    glm::vec3 cfGrad;
    glm::vec3 viscosity;
    bool hitsBoundingBox();
    ~Particle(){};
    void draw();
    vector<Particle > neighbors;
    Particle(glm::vec3 pos, glm::vec3 vel);
    bool willHitBoundingBox(float t);
};

class Viewport {
  public:
    int w, h; // width and height
};

//****************************************************
// Bounding Box 
//****************************************************

class BoundingBox {
  public: 
    glm::vec3 nll;
    glm::vec3 ntl;
    glm::vec3 nlr;
    glm::vec3 ntr;
    glm::vec3 fll;
    glm::vec3 ftl;
    glm::vec3 flr;
    glm::vec3 ftr;
    float left_wall;
    float right_wall;
    float rear_wall;
    float front_wall;
    float bottom_wall;
    float sideLength;
    BoundingBox(){};
    BoundingBox(glm::vec3 start, float length);
    bool hitsBoundary(glm::vec3 position, glm::vec3 radius);
};




//****************************************************
// Bounding Box constructor
// Takes in a start vertex, and a length, and produces a cube.
//****************************************************

BoundingBox::BoundingBox(glm::vec3 start, float length) {
    sideLength = length;
    nll = start;
    ntl = start;
    nlr = start;
    ntr = start;
    fll = start;
    ftl = start;
    flr = start;
    ftr = start;
    ntl.y += length;
    nlr.x += length;
    ntr.x += length;
    ntr.y += length;
    fll.z -= length;
    ftl.z -= length;
    ftl.y += length;
    flr.x += length;
    flr.z -= length;
    ftr.x += length;
    ftr.y += length;
    ftr.z -= length;
    left_wall = nll.x;
    right_wall = nlr.x;
    rear_wall = fll.z;
    front_wall = nll.z;
    bottom_wall = nll.y;
}


bool BoundingBox::hitsBoundary(glm::vec3 position, glm::vec3 radius) {
  return false;
}

//****************************************************
// Grid class and methods
//****************************************************


class Grid {
  //vector < vector < vector < int > > >  grid_locs;
  vector < vector <int > > grid_locs;
  float min_val;
  float max_val;
  float cube_length;
  int total_cubes;

  public: 
    Grid(){};
    Grid(float min_val, float max_val, float cube_length);
    void clearGrid();

    int insertCube(int particle_index, glm::vec3 position, float cube_length);
    void updateGrid(vector<Particle> particles);
    void validateCubeIndex(int k);
    vector<int > generateNeighbors(int box_index, int cube_length);
};


//****************************************************
// Bounding Box collision global variables
//****************************************************
float DAMP = -0.2f;
static glm::vec3 dampVec;
float PARTICLE_RADIUS = 0.05f;
Grid grid;
BoundingBox box;
static float timeToHit = 0.0f;
static float timeRemainder = 0.0f;
float TIME_STEP = 0.0025f;

//****************************************************
// Checks bounding box collision. Return bool and sets damping vector appropriately.
// Should probably use willHitBoundingBox instead for better particle deflection
//****************************************************
bool Particle::hitsBoundingBox() {
	if (position[0] + PARTICLE_RADIUS > box.right_wall || position[0] - PARTICLE_RADIUS < box.left_wall) {
		dampVec[0] = DAMP;
		dampVec[1] = 1.0f;
		dampVec[2] = 1.0f;
		return true;
	}
	else if (position[2] + PARTICLE_RADIUS > box.front_wall ||	position[2] - PARTICLE_RADIUS < box.rear_wall) {
		dampVec[0] = 1.0f;
		dampVec[1] = 1.0f;
		dampVec[2] = DAMP;
		return true;	
	}
	else if(position[1] - PARTICLE_RADIUS < box.bottom_wall) {
		dampVec[0] = 1.0f;
		dampVec[1] = DAMP;
		dampVec[2] = 1.0f;
		return true;
	}
	return false;
}

//****************************************************
// Checks if a particle WILL hit the bounding box, and set the time it will take to hit.
// Sets time to the minimum for the first wall to be hit
// Also sets the remaining time to calculate position after deflection
//****************************************************
bool Particle::willHitBoundingBox(float time) {
	bool hit = false;
	timeToHit = time;
	timeRemainder = time;
	if (velocity[0] != 0.0f && position[0] + (velocity[0] * TIME_STEP) + PARTICLE_RADIUS >= box.right_wall) {
		float t = (box.right_wall - position[0] - PARTICLE_RADIUS) / velocity[0];
		hit = true;
		if (t < timeToHit) { //cout<<"damp set\n";
			dampVec[0] = DAMP;
			dampVec[1] = 1.0f;
			dampVec[2] = 1.0f;
			timeToHit = t;
			timeRemainder = time - t;
		} //cout<<"Time to hit right wall: "<<t<<"\n";
	}
	if (velocity[0] != 0.0f && position[0] + (velocity[0] * TIME_STEP) - PARTICLE_RADIUS <= box.left_wall) {
		float t = (box.left_wall - position[0] + PARTICLE_RADIUS) / velocity[0];
		hit = true;
		if (t < timeToHit) {	//		cout<<"damp set\n";
			dampVec[0] = DAMP;
			dampVec[1] = 1.0f;
			dampVec[2] = 1.0f;
			timeToHit = t;
			timeRemainder = time- t;
		} //cout<<"Time to hit left wall: "<<t<<"\n";
	} 
	
	if (velocity[2] != 0.0f && position[2] + (velocity[2] * TIME_STEP) + PARTICLE_RADIUS >= box.front_wall) {
		float t = (box.front_wall - position[2] - PARTICLE_RADIUS) / velocity[2];
		hit = true;
		if (t < timeToHit) {	//		cout<<"damp set\n";
			dampVec[0] = 1.0f;
			dampVec[1] = 1.0f;
			dampVec[2] = DAMP;
			timeToHit = t;
			timeRemainder = time - t;
		} //cout<<"Time to hit front wall: "<<t<<"\n";
	}
	if (velocity[2] != 0.0f && position[2] + (velocity[2] * TIME_STEP) - PARTICLE_RADIUS <= box.rear_wall) {
		float t = (box.rear_wall - position[2] + PARTICLE_RADIUS) / velocity[2];
		hit = true;
		if (t < timeToHit) {	//		cout<<"damp set\n";
			dampVec[0] = 1.0f;
			dampVec[1] = 1.0f;
			dampVec[2] = DAMP;
			timeToHit = t;
			timeRemainder = time - t;
		} //cout<<"Time to hit rear wall: "<<t<<"\n";
	} 
	if (velocity[1] != 0.0f && position[1] + (velocity[1] * TIME_STEP) - PARTICLE_RADIUS <= box.bottom_wall) {
		float t = abs(box.bottom_wall - position[1] + PARTICLE_RADIUS) / velocity[1];
		hit = true;
		if (t < timeToHit) {	//		cout<<"damp set\n";
			dampVec[0] = 1.0f;
			dampVec[1] = DAMP;
			dampVec[2] = 1.0f;
			timeToHit = t;
			timeRemainder = time - t;
		} //cout<<"Time to hit bottom: "<<t<<"\n";
	} 
	return hit;
}

Particle::Particle(glm::vec3 pos, glm::vec3 vel) {
	position = pos;
	velocity = vel;
}

//****************************************************
// More Global Variables
//****************************************************

#define VIEWPORT_HEIGHT 1200;
#define VIEWPORT_WIDTH  1200;
Viewport    viewport;
float CURRENT_TIME = 0.0f;
float PARTICLE_MASS = 1.05f;

float SMOOTHING_LENGTH = 2.0f;

float K = 5.0f;
float REST_DENSITY = 20.0f;
glm::vec3 GRAVITY = glm::vec3(0.0f, -9.8f, 0.0f);
vector<Particle> particles;
int numParts = 100;
GLint stacks = 10;
int RENDER_BACKGROUND = 1;

//****************************************************
// Declare functions for later use
//****************************************************

void generateParticles(int);
float w_poly6(glm::vec3, float, float);
glm::vec3 w_poly6_gradient(glm::vec3, float, float);
float w_poly6_laplacian(glm::vec3, float, float);
glm::vec3 w_pressure_gradient(glm::vec3, float, float);
float vec3dist(glm::vec3, glm::vec3);
float w_viscosity_laplacian(glm::vec3, float r, float h) ;


//****************************************************
// Distance between two vectors
//****************************************************
float vec3dist(glm::vec3 v1, glm::vec3 v2) {
   	float dx = abs(v2.x - v1.x);
   	float dy = abs(v2.y - v1.y);
   	float dz = abs(v2.z - v1.z);
   	float dist = sqrt(dx*dx+dy*dy+dz*dz);
   	//cout<<"Dist (printing from vec3dist): "<<dist<<"\n";
   	return dist;
}

//****************************************************
// Length of a vector (magnitude)
//****************************************************
float vec3length(glm::vec3 v1) {
   	return sqrt(v1.x*v1.x+v1.y*v1.y+v1.z*v1.z);
}


//****************************************************
// Drawing a particle
//****************************************************
void Particle::draw() {
   	glutSolidSphere(PARTICLE_RADIUS, stacks, stacks);
}

//****************************************************
// Initial setup for particles
//****************************************************

void setupParticles() {
  for (int i = 0; i < (signed)particles.size(); i++) {
    //Replace this with initialDensity
    particles[i].density = 1.0f;
    //particles[i].pressure = 0.0f;
    particles[i].pressure = glm::vec3(0.0f, 0.0f, 0.0f);
    particles[i].cfLap = 1.0f;
    particles[i].cfGrad = glm::vec3(0.0f, 0.0f, 0.0f);
  }
}


//****************************************************
// Calculates particle densities based on position
//****************************************************
void calculateParticleDensities() {
  float h = SMOOTHING_LENGTH;
  for (int i = 0; i < (signed)particles.size(); i++) {
    for (int j = 0; j < (signed)particles.size(); j++) {
      if (i == j) {
        continue;
      }
      float R = vec3dist(particles[i].position, particles[j].position);      
      glm::vec3 r = particles[i].position - particles[j].position;
      //cout<<"WPOLY6: "<<w_poly6(r, R, h)<<"\n";
      if (R <= h) {
      	particles[i].density = particles[i].density + PARTICLE_MASS * w_poly6(r, R, h);
      }
    }
  }
}


//BS value. Might want to actually replace with proper value, if possible.
float GAS_CONSTANT = 1000;
// viscosity coefficient
float eta = 1000.0f;

//****************************************************
// Calculates other particle forces
//****************************************************

void calculateParticleForces() {

	float h = SMOOTHING_LENGTH;
	for (int i = 0; i < (signed)particles.size(); i++) {
		//cout<<particles[i].density<<"\n";
		for (int j = 0; j < (signed)particles.size(); j++) {
			if (i == j) {
				continue;
			}
			glm::vec3 r = particles[i].position - particles[j].position;
      		float R = vec3dist(particles[i].position, particles[j].position);


			if (R <= h) {
				float density_p = particles[i].density;
				float density_n = particles[j].density;
				float pressure_p = GAS_CONSTANT * (density_p - REST_DENSITY);
				float pressure_n = GAS_CONSTANT * (density_n - REST_DENSITY);
				

				particles[i].pressure = PARTICLE_MASS*(pressure_p + pressure_n)/(2*density_n)*w_pressure_gradient(r, R, h);				
				//cout<<"Pressure: "<<particles[i].pressure[0]<<" "<<particles[i].pressure[1]<<" "<<particles[i].pressure[2]<<"\n";
				particles[i].viscosity = eta * PARTICLE_MASS * ((particles[j].velocity - particles[i].velocity) / density_n) * w_viscosity_laplacian(r, R, h);
				particles[i].cfLap = particles[i].cfLap + PARTICLE_MASS / density_n * w_poly6_laplacian(r, R, h);
				particles[i].cfGrad = particles[i].cfGrad + PARTICLE_MASS / density_n * w_poly6_gradient(r, R, h);
        /*
        particles[i].cfLap = particles[i].cfLap + PARTICLE_MASS / density_n * w_poly6(r, R,h);
        particles[i].cfGrad = particles[i].cfGrad + PARTICLE_MASS / density_n * w_poly6(r, R,h);
        */

				
			}
		}
	}
}

//****************************************************
// Smoothing Kernel Functions
//****************************************************
float w_poly6(glm::vec3 r, float R, float h) {
  if (0 <= R && R <= h) {
    float h_squared = h * h;
    float r_squared = R * R;
    float h_r_diff = h_squared - r_squared;
    h_r_diff = pow(h_r_diff, 3);

    float h_ninth = pow(h, 9);

    float weight = (315 / (64 * PI * h_ninth)) * (h_r_diff);
    return weight;
  }
  return 0.0f;  
}

glm::vec3 w_poly6_gradient(glm::vec3 r, float R, float h) {
 float h_squared = h * h; 
 float r_squared = R * R;
 float hrsquared = pow((h_squared - r_squared), 2);
 float h_ninth = pow(h, 9);
 glm::vec3 neg_r = -1.0f * r;
 float weight = (945 / (32 * PI * h_ninth)) * hrsquared;
 return neg_r * weight;

}

float w_poly6_laplacian(glm::vec3 r, float R, float h) {
 float h_squared = h * h; 
 float r_squared = R * R;
 float hrsquared = pow((h_squared - r_squared), 2);
 float h_ninth = pow(h, 9);

 float lead_coeff = (945 / (8 * PI * h_ninth));
 return lead_coeff * (hrsquared) * (r_squared - 0.75 * hrsquared);

}

glm::vec3 w_pressure_gradient(glm::vec3 r, float R, float h) {
  float h_sixth = pow(h, 6);

  float h_r = h - R;
  float h_r_squared = pow(h_r, 2);

  glm::vec3 neg_r = r * -1.0f;
  //cout<<"R: "<<r[0]<<" "<<r[1]<<" "<<r[2]<<"\n";
  //cout<<"Neg_r: "<<neg_r[0]<<" "<<neg_r[1]<<" "<<neg_r[2]<<"\n";
  float weight = (45 / (PI * h_sixth * R)) * h_r_squared;
  //cout<<"Ret: "<<neg_r[0]*weight<<" "<<neg_r[1]*weight<<" "<<neg_r[2]*weight<<"\n";
  return neg_r * weight;
}

float w_viscosity_laplacian(glm::vec3 r, float R, float h)  {
  float h_fifth = pow(h, 5);
  float weight = (45 / (PI * h_fifth)) * (1 - (R / h));
  return weight;
}

//****************************************************
// Finds the minimum positions of particles
//****************************************************
void findMinParticlePositions() {
  float minx = 0.0f;
  float miny = 0.0f;
  float minz = 0.0f;

  for (int i = 0; i < particles.size(); i++) {
    glm::vec3 position = particles[i].position;
    minx = min(minx, position.x);
    miny = min(miny, position.y);
    minz = min(minz, position.z);
  }
  cout << "Min Values: " << endl;
  cout << "Minx: " << minx << endl;
  cout << "Miny: " << miny << endl;
  cout << "Minz: " << minz << endl;

}


float SIGMA = 250.0f;
float threshold = 0.01;

//****************************************************
// Loops through particles and updates their positions
//****************************************************

void updateParticlePositions() {
	for (int i = 0; i < (signed)particles.size(); i++) {

		glm::vec3 surface_tension, particle_pressure, viscosity; 
		
		float grad_length = vec3length(particles[i].cfGrad);
		if (grad_length >= threshold){
			surface_tension = -1.0f*SIGMA*particles[i].cfLap * particles[i].cfGrad/grad_length;
		}
		else{
			surface_tension = glm::vec3(0.0f, 0.0f, 0.0f); 
		}
		
		//cout<<"Surface Tension: "<<surface_tension[0]<<" "<<surface_tension[1]<<" "<<surface_tension[2]<<"\n";
		
		particle_pressure = particles[i].pressure;
		viscosity = particles[i].viscosity;
		
		glm::vec3 total_force = surface_tension + particle_pressure + viscosity;
		glm::vec3 acceleration = (total_force / particles[i].density) * TIME_STEP + GRAVITY;
		particles[i].velocity = particles[i].velocity + acceleration * TIME_STEP;
		// if collision imminent, update position until it hits wall, then reverse proper 
		// velocity component with damping and update position with remaining time
		if(particles[i].willHitBoundingBox(TIME_STEP)){
			float t = TIME_STEP;
			while (particles[i].willHitBoundingBox(t)) {
				//cout<<"DAMP: "<<dampVec[0]<<" "<<dampVec[1]<<" "<<dampVec[2]<<"\n";
				particles[i].position = particles[i].position + (particles[i].velocity * timeToHit);
				particles[i].velocity = particles[i].velocity * dampVec;
				t = timeRemainder;
				//cout<<"Updating position. Will Hit\n";
			}
			particles[i].position = particles[i].position + (particles[i].velocity * timeRemainder);
		}
		//otherwise, just update position
		else{
			particles[i].position = particles[i].position + (particles[i].velocity * TIME_STEP);
			//cout<<"Updating position\n";
		}
		/*
		
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) {
				continue;
			}
			float d = vec3dist(particles[i].position, particles[j].position);
			float minDist = 2*PARTICLE_RADIUS;
			if (d < minDist) {
				glm::vec3 tVec = particles[j].position - particles[i].position;
				float scalar = sqrt((minDist*minDist)/(tVec[0]*tVec[0] + tVec[1]*tVec[1] + tVec[2]*tVec[2]));
				particles[i].velocity = particles[i].velocity * 0.5f;
				particles[j].velocity = particles[j].velocity + particles[i].velocity * 0.6f;
				
				particles[j].position = particles[j].position + (tVec*scalar);			
			}
		}
		*/
		
		
		
	} 
	//cout<<"TIME: " << CURRENT_TIME<<"\n:;
	//cout<<"POS:"<<' '<<particles[1].position[0]<<' '<<particles[1].position[1]<<' '<<particles[1].position[2]<<'\n';
	//cout<<"VEL:"<<' '<<particles[1].velocity[0]<<' '<<particles[1].velocity[1]<<' '<<particles[1].velocity[2]<<'\n';
}


//****************************************************
// reshape viewport if the window is resized
//****************************************************

void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport(0,0,viewport.w,viewport.h);// sets the rectangle that will be the window
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();                // loading the identity matrix for the screen

  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system
  // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  // glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center
  glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch
  //------------------------------------------------------------
}

//****************************************************
// sets the window up
//****************************************************

void initScene(){

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent
  myReshape(viewport.w,viewport.h);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);

  GLfloat lightAmbient[] = {0.1,0.1,0.1,1.0};
  GLfloat lightDiffuse[] = {1,1,1,1};
  GLfloat lightSpecular[] = {1,1,1,1};
  GLfloat lightPosition[] = {10, 10,-10,0};
  glEnable(GL_LIGHTING); 
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
  GLfloat materialAmbient [] = {0.4, .5, .9, 1.0};
  GLfloat materialDiffuse[] = {0.2, 0.3, 0.9, 1.0};
  GLfloat materialSpecular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat materialShininess[] = {40, 0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, materialAmbient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, materialDiffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular);
  glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS, materialShininess);

  
  generateParticles(numParts);

}


//****************************************************
// function for handling keypresses
//****************************************************
void getKeys(unsigned char key, int x, int y) {
    switch(key) {
      /*
      case ' ': {
			exit(0);
			break;
		  }
      case 'n': {
			Particle particle;
			particle.position = glm::vec3(-0.70f, 0.5f, -3.5f);
			particle.velocity = glm::vec3(10, 0, 1);
			particles.push_back(particle);
			cout << "Particle added! Total particles: " << particles.size() << endl;
			break;
		  }
      case 'm': {
			Particle particle;
			particle.position = glm::vec3(0.0f, 1.0f, -3.5f);
			particle.velocity = glm::vec3(3, 0, 2);
			particles.push_back(particle);
			cout << "Particle added! Total particles: " << particles.size() << endl;
			break;
		  }*/
      default:{
      		Particle particle;
			particle.position = glm::vec3(-0.70f, 0.5f, -3.5f);
			int k = 4;
			if (key %2 == 0) {
				k = -4;
			}
			particle.velocity = glm::vec3((key-96), 0, (int)(key-96)/k);
			particles.push_back(particle);
			cout << "Particle added! Total particles: " << particles.size() << endl;
			break;
		}
      }
  }

//****************************************************
// function for handling special keypresses (SHIFT and ARROWS)
//****************************************************

void getSpecialKeys(int key, int x, int y) {
  int modifier = glutGetModifiers();
  switch (modifier) {
    case GLUT_ACTIVE_SHIFT:
      switch(key) {
            case GLUT_KEY_RIGHT:
              break;
            default:
              break;
      }   
      break;
    default:
      switch(key) {
            case GLUT_KEY_RIGHT:
              break;
            default:
              break;
      }
  }
}

//****************************************************
// Draw background / bounds
//****************************************************

void drawBackground() {
  
	GLfloat bgSpecular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat bgShininess = 10;
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glColor4f(0.9,0.9,0.9, 1.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, bgSpecular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, bgShininess);

	glPushMatrix();
	glBegin(GL_QUADS);
    glVertex3f(box.fll.x, box.fll.y, box.fll.z);
    glVertex3f(box.flr.x, box.flr.y, box.flr.z);
    glVertex3f(box.ftr.x, box.ftr.y, box.ftr.z);
    glVertex3f(box.ftl.x, box.ftl.y, box.ftl.z);
    
    
    glVertex3f(box.fll.x, box.fll.y, box.fll.z);
    glVertex3f(box.flr.x, box.flr.y, box.flr.z);
    glVertex3f(box.nlr.x, box.nlr.y, box.nlr.z);
    glVertex3f(box.nll.x, box.nll.y, box.nll.z);
    
    
    glVertex3f(box.nll.x, box.nll.y, box.nll.z);
    glVertex3f(box.fll.x, box.fll.y, box.fll.z);
    glVertex3f(box.ftl.x, box.ftl.y, box.ftl.z);
    glVertex3f(box.ntl.x, box.ntl.y, box.ntl.z);

    
    glVertex3f(box.nlr.x, box.nlr.y, box.nlr.z);
    glVertex3f(box.flr.x, box.flr.y, box.flr.z);
    glVertex3f(box.ftr.x, box.ftr.y, box.ftr.z);
    glVertex3f(box.ntr.x, box.ntr.y, box.ntr.z);
    
    glEnd();
    glDisable(GL_COLOR_MATERIAL);
    glPopMatrix();
    
    glDisable(GL_COLOR_MATERIAL);
}






//****************************************************
// Draw bounding box  
//****************************************************

void drawBoundingBox() {
  glPushMatrix();
  glLineWidth(.5);
  glColor4f(0.5, 0.5, 0.5, 1.0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(box.nll.x, box.nll.y, box.nll.z);
  glVertex3f(box.nlr.x, box.nlr.y, box.nlr.z);
  glVertex3f(box.ntr.x, box.ntr.y, box.ntr.z);
  glVertex3f(box.ntl.x, box.ntl.y, box.ntl.z);
  glVertex3f(box.nll.x, box.nll.y, box.nll.z);
  glEnd();
  
  glLineWidth(.5);
  glColor4f(0.5, 0.5, 0.5, 1.0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(box.fll.x, box.fll.y, box.fll.z);
  glVertex3f(box.flr.x, box.flr.y, box.flr.z);
  glVertex3f(box.ftr.x, box.ftr.y, box.ftr.z);
  glVertex3f(box.ftl.x, box.ftl.y, box.ftl.z);
  glVertex3f(box.fll.x, box.fll.y, box.fll.z);
  glEnd();

  glLineWidth(.5);
  glColor4f(0.5, 0.5, 0.5, 1.0);
  glBegin(GL_LINES);
  glVertex3f(box.nll.x, box.nll.y, box.nll.z);
  glVertex3f(box.fll.x, box.fll.y, box.fll.z);
  glEnd();

  glLineWidth(.5);
  glColor4f(0.5, 0.5, 0.5, 1.0);
  glBegin(GL_LINES);
  glVertex3f(box.ntl.x, box.ntl.y, box.ntl.z);
  glVertex3f(box.ftl.x, box.ftl.y, box.ftl.z);
  glEnd();

  glLineWidth(.5);
  glColor4f(0.5, 0.5, 0.5, 1.0);
  glBegin(GL_LINES);
  glVertex3f(box.ntr.x, box.ntr.y, box.ntr.z);
  glVertex3f(box.ftr.x, box.ftr.y, box.ftr.z);
  glEnd();

  glLineWidth(.5);
  glColor4f(0.5, 0.5, 0.5, 1.0);
  glBegin(GL_LINES);
  glVertex3f(box.nlr.x, box.nlr.y, box.nlr.z);
  glVertex3f(box.flr.x, box.flr.y, box.flr.z);
  glEnd();

  glPopMatrix();
}


//****************************************************
// Generate particles  
//****************************************************
void generateParticles(int number) {
	GLdouble distance = 3*PARTICLE_RADIUS;
	number--;
	float xMult = 0;
	float yMult = 0;
	float zMult = 0;
	float x = box.left_wall + PARTICLE_RADIUS;
	float y = box.ntl.y - PARTICLE_RADIUS;
	float z = box.front_wall - PARTICLE_RADIUS;
	while (number > 0) {
		Particle particle;
		particle.position = glm::vec3(x+xMult*distance, y+yMult*distance, z+zMult*distance);
		//particle.velocity = glm::vec3((float)(number%5), -1*((float)(number%7)), (float)(number%11));
		particle.density = 1.0f;
		particle.pressure = glm::vec3(0.0f, 0.0f, 0.0f);
		particle.cfLap = 1.0f;
    	particle.cfGrad = glm::vec3(0.0f, 0.0f, 0.0f);
		particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		particles.push_back(particle);
		zMult--;
		if (z+zMult*distance <= box.ftl.z + PARTICLE_RADIUS) {
			zMult = 0;
			xMult++;
		}
		if (x+xMult*distance >= box.ntr.x - PARTICLE_RADIUS) {
			xMult = 0;
			yMult--;
		}
		number--;
	}
}


//****************************************************
// Draw particles  
//****************************************************
void drawParticles() {
  for (int i = 0; i < particles.size(); i++) {
    glm::vec3 position = particles[i].position;
  
	GLfloat bgSpecular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat bgShininess = 40;
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
	glColor4f(0.3, 0.4, 0.9, 1.0);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glColor4f(0.2, 0.3, 0.9, 1.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, bgSpecular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, bgShininess);
	
    glPushMatrix();
    glTranslatef(position.x, position.y, position.z);
    particles[i].draw();
    glPopMatrix();
    glDisable(GL_COLOR_MATERIAL);
  }
}

//****************************************************
// load the scene and draw the particles
//****************************************************

void myDisplay() {
  CURRENT_TIME += TIME_STEP;
  //Scene Setup
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the color buffer (sets everything to black)
  glViewport(0, 0, viewport.w, viewport.h);
  //Camera Setup:
  glColor3f(0.0, 1.0, 0);
  glMatrixMode(GL_PROJECTION);                  
  glLoadIdentity();                            
  gluPerspective(40.0f, viewport.w/viewport.h, 0.1f, 100.0f);
  /*
  gluPerspective(fov * zoom, aspect_ratio, z_near, z_far);
  */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); // make sure transformation is "zero'd"
  /* Main Rendering loop: uses naive neighbor calculation ==> O(n^2) */
  setupParticles();
  calculateParticleDensities();
  calculateParticleForces();
  updateParticlePositions();
  /* Drawing sphere loop */
  drawBoundingBox();
  if (RENDER_BACKGROUND) {
  	drawBackground();
  }
  drawParticles();
  //Scene cleanup
/*
  findMinParticlePositions();

  //Testing Neighbors algorithm
  grid.clearGrid();
  grid.updateGrid(particles);
*/
  glFlush();
  glutSwapBuffers();// swap buffers (we earlier set double buffer)
}


//****************************************************
// called by glut when there are no messages to handle
//****************************************************

void myFrameMove() {
	  //nothing here for now
	#ifdef _WIN32
	  Sleep(10); //give ~10ms back to OS (so as not to waste the CPU)
	#endif
	  glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}



void parseFile(string file) {
  std::ifstream inpfile(file.c_str());

  if(!inpfile.is_open()) {
    std::cout << "Unable to open file" << std::endl;
  } else {
    std::string line;
    while(inpfile.good()) {
      std::vector<std::string> splitline;
      std::string buf;
      std::getline(inpfile,line);
      std::stringstream ss(line);
      while (ss >> buf) {
        splitline.push_back(buf);
      }
      //Ignore blank lines
      if(splitline.size() == 0) {
        continue;
      }
      else if(splitline.size() == 1) {
        continue;
      }
      else if(splitline[0][0] == '#') {
        continue;
      }
      else if(!splitline[0].compare("size")) {
      }
      else {
        std::cerr << "Unknown command: " << splitline[0] << std::endl;
      }
    }
    inpfile.close();
  }
}



//****************************************************
// the usual stuff, nothing exciting here
//****************************************************

int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);
  //This tells glut to use a double-bufferd window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
  // Initalize theviewport size
  box = BoundingBox(glm::vec3(-.75,-.75,-3), 1.5);
  viewport.w = VIEWPORT_WIDTH;
  viewport.h = VIEWPORT_HEIGHT;
  //Parse input file: where OBJ files are parsed
  /*
  string filename = argv[1];
  parseFile(filename);
  */
  
  for (int i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-np") == 0){
	    numParts = atoi(argv[i+1]);
	    i++;
    }
	else if (strcmp(argv[i], "-rad") == 0){
		PARTICLE_RADIUS = atof(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-damp") == 0){
		DAMP = -1*atof(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-s") == 0){
		stacks = atoi(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-rd") == 0){
		REST_DENSITY = atof(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-surf") == 0){
		SIGMA = atof(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-smooth") == 0){
		SMOOTHING_LENGTH = atof(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-back") == 0){
		RENDER_BACKGROUND = atoi(argv[i+1]);
		i++;
	}
	
  }
  
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Fluid Simulator");
  initScene();                                 // quick function to set up scene
  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  glutKeyboardFunc(getKeys);
  glutSpecialFunc(getSpecialKeys);
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  return 0;

}