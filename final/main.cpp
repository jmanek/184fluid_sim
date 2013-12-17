#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <list>

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
#include "glm/gtx/rotate_vector.hpp"


#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265
using namespace std;
 
   
   
//****************************************************
// Class declarations
//****************************************************
class Particle;
class Grid;
class BoundingBox;
class Viewport;
//****************************************************
// Struct for storing collisions
//****************************************************
struct Collision{
	float time; // time to collision
	int id1; // id of the travelling particle
	int id2; // id of the particle that will be collided with
	Collision(float t, int i1, int i2) : time(t), id1(i1), id2(i2) {}
	bool operator < (const Collision& other) const {
		return (time < other.time);
	}
	// overriding the < operator allows the use of the standard sort
	// std::sort(someVec.begin(), someVec.end()) 
	// to sort a vector of Collisions.
};
//****************************************************
// Global variables
//****************************************************
float K = 5.0f; //Gas constant 1000
float eta = 2.0f; //Viscosity coefficient
glm::vec3 GRAVITY = glm::vec3(0.0f, -9.8f, 0.0f);
float TIME_STEP = 0.0025f;
float DAMP = -0.2f;
GLint stacks = 6;
vector<Particle> particles;
vector<glm::vec3> pressures;
vector<float> cfLap;
vector<glm::vec3> cfGrad;
vector<glm::vec3> viscosities;
vector<float> densities;
static glm::vec3 dampVec;
float PARTICLE_RADIUS = 0.05f;
static float timeToHit = 0.0f;
static float timeRemainder = 0.0f;
#define VIEWPORT_HEIGHT 1200;
#define VIEWPORT_WIDTH  1200;
float CURRENT_TIME = 0.0f;
float PARTICLE_MASS = 1.05f;
float SMOOTHING_LENGTH = 0.05f;
float REST_DENSITY = 100.0f;
bool grav = false;
int numParts = 100;
bool RENDER_BACKGROUND = true;
float SIGMA = 4.9f; //surface tension coefficient
float threshold = 0.55f;
vector<Collision> collisions;
bool PAUSED = false;
bool PRESSURE_MAPPED = false;
//****************************************************
// Declare functions for later use
//****************************************************
void generateParticles(int);
float vec3dist(glm::vec3, glm::vec3);
float w_viscosity_laplacian(glm::vec3, float, float) ;
void generateParticlesRandom(int);
float vec3dist(glm::vec3, glm::vec3);
void vec3print(glm::vec3);
float vec3length(glm::vec3);
float w_poly6(glm::vec3, float, float);
glm::vec3 gradient_w_poly6(glm::vec3, float, float);
float laplacian_w_poly6(glm::vec3, float, float);
glm::vec3 gradient_w_spiky(glm::vec3, float, float);
float laplacian_w_viscosity(glm::vec3, float, float);
//****************************************************
// Bounding Box class
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
// Bounding Box class - constructor
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
BoundingBox box;
//****************************************************
// Particle Class
//****************************************************
class Particle {
  public:
    Particle(){};
    int gridIndex;
    glm::vec3 position; 
    glm::vec3 velocity;
    ~Particle(){};
    void draw();
    vector<Particle> neighbors;
    Particle(glm::vec3, glm::vec3);
    bool willHitBoundingBox(float);
    int findCollision();
};
//****************************************************
// Grid Class
//****************************************************
class Grid {
	//vector < vector < vector < int > > >  grid_locs;
	vector < vector <int > > grid_locs;
	float min_val;
	float max_val;
	float cube_length;
	int total_cubes;
	vector <int > occupied_locs;
	vector <vector <int > > neighbor_cube_list; 

	public: 
		Grid(){};
		Grid(float min_val, float max_val, float cube_length);
		void initialize();
		void clearGrid();

		int insertCube(int particle_index, glm::vec3 position, float cube_length, vector<Particle> particles);
		void updateGrid(vector<Particle> particles);
		void validateCubeIndex(int k);
		vector<int > generateNeighbors(int box_index, int cube_length);
		vector<int > consolidateNeighbors(int neighbor_index);
};
Grid grid;
//****************************************************
// Particle Class - Drawing a particle
//****************************************************
void Particle::draw() {
   	glutSolidSphere(PARTICLE_RADIUS, stacks, stacks);
}
//****************************************************
// Creates Collision if collision occurs in time frame.
// Added to global vector to be processed each iteration.
// Not used. Too expensive.
//****************************************************
void findCollisions(int id1, float time) {
	Particle copy1 = Particle(particles[id1]);
	Particle copy2;
	glm::vec3 oldVel;
	vector<Collision> localCollisions;
   	//for (int i = 0; i < particles[id1].neighbors.size(); i++){
   	for (int i = 0; i < particles.size(); i++) {
   		if (i == id1) {
   			continue;
   		}
   		oldVel = copy1.velocity;
   		copy2 = Particle(particles[i]);
   		copy1.velocity = copy1.velocity - copy2.velocity;
   		copy2.velocity = copy2.velocity - copy2.velocity;
   		glm::vec3 startPos = copy1.position + glm::normalize(copy1.velocity)*PARTICLE_RADIUS;
   		float a = glm::dot(copy1.velocity, copy1.velocity);
   		float b = 2 * glm::dot(copy1.velocity, startPos - copy2.position);
   		float c = glm::dot(startPos - copy2.position, startPos - copy2.position) - (PARTICLE_RADIUS * PARTICLE_RADIUS);
   		float discriminant = b*b - 4*a*c;
   		//cout<<"Discriminant: "<<discriminant<<"\n";
   		if (discriminant < 0){ // imaginary roots indicate no hit
   			continue;
   		}
   		float d = sqrtf(discriminant);
   		float q;
   		if (b<0) {
   			q = (-b - d)/2.0f;
   		}
   		else{
   			q = (-b + d)/2.0f;
   		}
   		float time0 = q/a;
   		float time1 = c/q;
   		
   		if (time0 > time1) {
   			float temp = time0;
   			time0 = time1;
   			time1 = temp;
   		}
   		if (time1 < 0) { // negative time indicates other particle is behind current particle, thus, no hit
   			continue;
   		}
   		if (time0 < 0 && time1 < time) {
   			localCollisions.push_back(Collision(time1, id1, i)); 
   		}
   		else if (time0 < time){
   			localCollisions.push_back(Collision(time0, id1, i));
   		}
   		copy1.velocity = oldVel;
   	}
   	if (!localCollisions.empty()) {
   		//cout<<"Adding collision to list\n";
   		std::sort(localCollisions.begin(), localCollisions.end()); // sort by time
   		collisions.push_back(localCollisions[0]); // add first collision to global list
   	}
   	
}

//****************************************************
// Returns index of first colliding neighbor, or -1 if no collision occurs.
//****************************************************
int Particle::findCollision() {
	int retVal = -1;
	float minDist = 1000;
	vector<int> neighbors = grid.consolidateNeighbors(gridIndex);
	for (int j = 0; j< neighbors.size(); j++) {
		float d = vec3dist(position, particles[j].position);
		if (d < minDist) {
			minDist = d;
			retVal = j;
		}
	}
	return retVal;
}
//****************************************************
// Particle Class - projects particle, calculates time to hit boundingBox
//****************************************************
bool Particle::willHitBoundingBox(float time) {
	bool hit = false;
	timeToHit = time;
	timeRemainder = time;
	if (velocity[0] != 0.0f && position[0] + (velocity[0] * time) + PARTICLE_RADIUS >= box.right_wall) {
		float t = (box.right_wall - position[0] - PARTICLE_RADIUS) / velocity[0];
		hit = true;
		if (t < timeToHit) {
			dampVec[0] = DAMP;
			dampVec[1] = 1.0f;
			dampVec[2] = 1.0f;
			timeToHit = t;
			timeRemainder = time - t;
		}
	}
	if (velocity[0] != 0.0f && position[0] + (velocity[0] * time) - PARTICLE_RADIUS <= box.left_wall) {
		float t = (box.left_wall - position[0] + PARTICLE_RADIUS) / velocity[0];
		hit = true;
		if (t < timeToHit) {
			dampVec[0] = DAMP;
			dampVec[1] = 1.0f;
			dampVec[2] = 1.0f;
			timeToHit = t;
			timeRemainder = time- t;
		}
	} 
	
	if (velocity[2] != 0.0f && position[2] + (velocity[2] * time) + PARTICLE_RADIUS >= box.front_wall) {
		float t = (box.front_wall - position[2] - PARTICLE_RADIUS) / velocity[2];
		hit = true;
		if (t < timeToHit) {
			dampVec[0] = 1.0f;
			dampVec[1] = 1.0f;
			dampVec[2] = DAMP;
			timeToHit = t;
			timeRemainder = time - t;
		}
	}
	if (velocity[2] != 0.0f && position[2] + (velocity[2] * time) - PARTICLE_RADIUS <= box.rear_wall) {
		float t = (box.rear_wall - position[2] + PARTICLE_RADIUS) / velocity[2];
		hit = true;
		if (t < timeToHit) {
			dampVec[0] = 1.0f;
			dampVec[1] = 1.0f;
			dampVec[2] = DAMP;
			timeToHit = t;
			timeRemainder = time - t;
		}
	} 
	if (velocity[1] != 0.0f && position[1] + (velocity[1] * time) - PARTICLE_RADIUS <= box.bottom_wall) {
		float t = abs(box.bottom_wall - position[1] + PARTICLE_RADIUS) / velocity[1];
		hit = true;
		if (t < timeToHit) {
			dampVec[0] = 1.0f;
			dampVec[1] = DAMP;
			dampVec[2] = 1.0f;
			timeToHit = t;
			timeRemainder = time - t;
		}
	} 
	return hit;
};
//****************************************************
// Particle Class - constructor
//****************************************************
Particle::Particle(glm::vec3 pos, glm::vec3 vel) {
	position = pos;
	velocity = vel;
}
//****************************************************
// Viewport Class
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};  
Viewport viewport;
//****************************************************
// Grid class - Grid constructor
//****************************************************
Grid::Grid(float minx, float maxx, float side_length) {
  min_val = minx;
  max_val = maxx;
  cube_length = side_length;
  total_cubes = int (max_val - min_val) / cube_length;
}
//****************************************************
// Grid class - Grid initializer
//****************************************************
void Grid::initialize() {
  int max_cube_index = total_cubes * total_cubes * total_cubes; 
  for (int i = 0; i < max_cube_index; i++) {
    vector<int> neighbor_indices;
    grid_locs.push_back(neighbor_indices);
  }

  cout << "cube_length: "  << cube_length << endl;
  cout << "total_cubes: " << total_cubes << endl;
  int count = 0;
  for (int j = 0; j < max_cube_index; j++) {
    vector<int > neighbor_cubes = generateNeighbors(j, total_cubes);
    neighbor_cube_list.push_back(neighbor_cubes);
    if (j == 0) {
      cout << "neighbor_cubes.size for 0: " << neighbor_cubes.size() << endl;
    }
  }
}
//****************************************************
// Grid class - Grid clearer
//****************************************************
void Grid::clearGrid() {
cout<<"clearing grid\n";
  int max_cube_index = total_cubes * total_cubes * total_cubes; 
  grid_locs.clear();
  for (int i = 0; i < occupied_locs.size(); i++) {
    vector<int> neighbor_indices;
    grid_locs[occupied_locs[i]] = neighbor_indices;
  }
  occupied_locs.clear();
}
//****************************************************
// Grid class - Insert a cube into the grid
//****************************************************
int Grid::insertCube(int particle_index, glm::vec3 position, float cube_length, vector<Particle > particles) {
	//cout << "hits in insertCube at " << particle_index << endl;
	int column_offset = floor(position.x / cube_length);
	int row_offset = floor(position.y / cube_length);
	int depth_offset = floor(position.z / cube_length);
	int total_cubes_squared = total_cubes * total_cubes;
	int total_cubes_cubed = total_cubes_squared * total_cubes;
	int grid_locs_index = column_offset + row_offset * (total_cubes)  + depth_offset * (total_cubes_squared);
	//cout << "grid_locs_index: " << grid_locs_index << endl;
	occupied_locs.push_back(grid_locs_index);
	//cout << "grid_locs_index " << grid_locs_index << endl;
	if (grid_locs_index > (total_cubes_cubed) - 1) {
		cout << "Broken index at:  " << grid_locs_index << endl;
		    	cout<<"-1000\n";
		return -1000;
	}
	else {
		//Temporary Hack: set any negative value to the 0th box to avoid seg faults: fixed when bounding box moved to lowest point at origin
		if (grid_locs_index < 0) {
			grid_locs[0].push_back(particle_index);
		}
		else {
			grid_locs[grid_locs_index].push_back(particle_index);
			particles[particle_index].gridIndex = grid_locs_index;
		}
	}
	return grid_locs_index;
}
//****************************************************
// Grid class - Update the grid
//****************************************************
void Grid::updateGrid(vector<Particle> particles){
cout<<"Starting update grid\n";
  clearGrid();
  for (int i = 0; i < particles.size(); i++) {
      	cout<<"iteration "<<i<<"\n";
    glm::vec3 particle_position = particles[i].position;
    int grid_index = insertCube(i, particle_position, cube_length, particles);
    //cout << "particles[i].gridIndex: " << particles[i].gridIndex << endl;
    //cout << "Inserted at grid number: " << grid_index << endl;
    if (grid_index == -1000) {
      cout << "Broke at particle position : " << particle_position.x << ", " << particle_position.y << ", " << particle_position.z << endl;
      break;
    }
  }
  cout<<"Finished updating grid\n";
}
//****************************************************
// Grid class - Cube index validation
//****************************************************
void Grid::validateCubeIndex(int k) {
  if (k < 0 || k > total_cubes) {
    cout << "Invalid index at " << k << endl;
  }
}
//****************************************************
// Get the nearest neighbors in a cube
// Note: code/cube indices based off of pg. 29 of 
// Auer's neighbor search algorithm. An explanation
// is provided in the paper as to how to compute the potential 27 values 
// found in the cases.  
//****************************************************
vector<int > Grid::generateNeighbors(int box_index, int cube_length) {
  vector<int > neighbors;
  int cube_length_squared = cube_length * cube_length;
  int total_cube_size = cube_length_squared * cube_length;

  // Back Face 
  if (box_index < cube_length_squared) {
    //Back face left column
    if (box_index % cube_length == 0) {
      //Back face - left column - lower left corner
      if (box_index == 0) {
        neighbors.push_back(box_index+1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
      }
      //Back face - left column - upper left corner
      else if (box_index == (cube_length_squared - cube_length)) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length + 1);

      }
      // Back face - left column - not a corner
      else {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
      }
    }
    // Back face - right column
    else if (box_index % cube_length == (cube_length - 1)) {
      //Back face - right column - lower right corner
      if (box_index == (cube_length - 1)) {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
      }
      //Back face - right column - upper right corner
      else if (box_index == (cube_length_squared - 1)) {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
      }
      //Backface - right column - not a corner
      else {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
      }
    }
    else {
      //Back face - bottom row - not corner
      if (box_index % cube_length_squared < cube_length) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
      }
      //Back face - top row - not corner
      else if (box_index % cube_length_squared > (cube_length_squared - cube_length)) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
      }
      //Back face - not row/column cubes
      else {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
      }
    }
  }

  //Front face
  else if (box_index >= (total_cube_size - cube_length_squared)) {
    //Front face - left column
    if (box_index % cube_length == 0) {
      //Front face - left column - lower left corner
      if (box_index == (total_cube_size - cube_length_squared)) {
        neighbors.push_back(box_index+1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
      }
      //Front face - left column - upper left corner 
      else if (box_index == (total_cube_size - cube_length)) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length + 1);
      }
      //Front face - left column - not a corner
      else {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length + 1);
      }
    }
    //Front face - right column

    else if (box_index % cube_length == (cube_length - 1)) {

      //Front face - right column - lower right corner
      if (box_index == (total_cube_size - cube_length_squared + (cube_length - 1))) {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
      }
      //Front face - right column - upper right corner
      else if (box_index == total_cube_size - 1 ) {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
      }

      //Front face - right column - not a corner
      else {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
      }
    }
    //Front face - not a column 
    else {
      //Front face - bottom row - not corner
      if (box_index % cube_length_squared < (total_cube_size - cube_length_squared + cube_length)) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
      }
      //Front face - top row - not corner
      else if (box_index % cube_length_squared > (total_cube_size - cube_length)) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
      }
      //Front face - not row/column cubes
      else {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
      }
    }
  }

  else if (box_index % cube_length == 0) {
    //Left face, top row
    if (box_index % cube_length_squared == (cube_length_squared - cube_length)) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
    }
    //Left face, bottom row
    else if (box_index % cube_length_squared < cube_length) {
        neighbors.push_back(box_index + 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
    }
    //Left face, every other value
    else {
        neighbors.push_back(box_index + 1);

        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length + 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length + 1);

        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared + 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length + 1);

        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared + 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
    }
  }

  //Right face
  else if (box_index % cube_length == (cube_length - 1)) {
    //Right face, top row
    if (box_index % cube_length_squared == (cube_length_squared - 1)) {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
    }
    //Right face, bottom row
    else if (box_index % cube_length_squared < cube_length) {
        neighbors.push_back(box_index - 1);
        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
    }
    //Right face, any other value
    else {
        neighbors.push_back(box_index - 1);

        neighbors.push_back(box_index + cube_length);
        neighbors.push_back(box_index + cube_length - 1);
        neighbors.push_back(box_index - cube_length);
        neighbors.push_back(box_index - cube_length - 1);

        neighbors.push_back(box_index - cube_length_squared);
        neighbors.push_back(box_index - cube_length_squared - 1);
        neighbors.push_back(box_index - cube_length_squared + cube_length);
        neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
        neighbors.push_back(box_index - cube_length_squared - cube_length);
        neighbors.push_back(box_index - cube_length_squared - cube_length - 1);

        neighbors.push_back(box_index + cube_length_squared);
        neighbors.push_back(box_index + cube_length_squared - 1);
        neighbors.push_back(box_index + cube_length_squared + cube_length);
        neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
        neighbors.push_back(box_index + cube_length_squared - cube_length);
        neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
    }
  }

  //Top face 
  else if (box_index % cube_length_squared >= (cube_length_squared - cube_length)){
    //Should just be one case: remaining squares inside
    neighbors.push_back(box_index - 1);
    neighbors.push_back(box_index + 1);

    neighbors.push_back(box_index - cube_length);
    neighbors.push_back(box_index - cube_length - 1);
    neighbors.push_back(box_index - cube_length + 1);

    neighbors.push_back(box_index - cube_length_squared);
    neighbors.push_back(box_index - cube_length_squared - 1);
    neighbors.push_back(box_index - cube_length_squared + 1);
    neighbors.push_back(box_index - cube_length_squared - cube_length);
    neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
    neighbors.push_back(box_index - cube_length_squared - cube_length + 1);

    neighbors.push_back(box_index + cube_length_squared);
    neighbors.push_back(box_index + cube_length_squared - 1);
    neighbors.push_back(box_index + cube_length_squared + 1);

    neighbors.push_back(box_index + cube_length_squared - cube_length);
    neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
    neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
  }
  //Bottom face
  else if (box_index % cube_length_squared < cube_length) {
    //Should just be one case: remaining squares inside
    neighbors.push_back(box_index - 1);
    neighbors.push_back(box_index + 1);

    neighbors.push_back(box_index + cube_length);
    neighbors.push_back(box_index + cube_length - 1);
    neighbors.push_back(box_index + cube_length + 1);

    neighbors.push_back(box_index - cube_length_squared);
    neighbors.push_back(box_index - cube_length_squared - 1);
    neighbors.push_back(box_index - cube_length_squared + 1);
    neighbors.push_back(box_index - cube_length_squared + cube_length);
    neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
    neighbors.push_back(box_index - cube_length_squared + cube_length + 1);

    neighbors.push_back(box_index + cube_length_squared);
    neighbors.push_back(box_index + cube_length_squared - 1);
    neighbors.push_back(box_index + cube_length_squared + 1);
    
    neighbors.push_back(box_index + cube_length_squared + cube_length);
    neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
    neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
  }
  //The rest of the cubes: everything inside of the cube
  else {
    neighbors.push_back(box_index - 1);
    neighbors.push_back(box_index + 1);

    neighbors.push_back(box_index + cube_length);
    neighbors.push_back(box_index + cube_length - 1);
    neighbors.push_back(box_index + cube_length + 1);
    neighbors.push_back(box_index - cube_length);
    neighbors.push_back(box_index - cube_length - 1);
    neighbors.push_back(box_index - cube_length + 1);

    neighbors.push_back(box_index - cube_length_squared);
    neighbors.push_back(box_index - cube_length_squared - 1);
    neighbors.push_back(box_index - cube_length_squared + 1);
    neighbors.push_back(box_index - cube_length_squared + cube_length);
    neighbors.push_back(box_index - cube_length_squared + cube_length - 1);
    neighbors.push_back(box_index - cube_length_squared + cube_length + 1);
    neighbors.push_back(box_index - cube_length_squared - cube_length);
    neighbors.push_back(box_index - cube_length_squared - cube_length - 1);
    neighbors.push_back(box_index - cube_length_squared - cube_length + 1);

    neighbors.push_back(box_index + cube_length_squared);
    neighbors.push_back(box_index + cube_length_squared - 1);
    neighbors.push_back(box_index + cube_length_squared + 1);
    
    neighbors.push_back(box_index + cube_length_squared + cube_length);
    neighbors.push_back(box_index + cube_length_squared + cube_length - 1);
    neighbors.push_back(box_index + cube_length_squared + cube_length + 1);
    neighbors.push_back(box_index + cube_length_squared - cube_length);
    neighbors.push_back(box_index + cube_length_squared - cube_length - 1);
    neighbors.push_back(box_index + cube_length_squared - cube_length + 1);
  } 
  neighbors.push_back(box_index);

  return neighbors;
}
//****************************************************
// Grid class - consolidates neighbors in a cube
//****************************************************
vector<int > Grid::consolidateNeighbors(int box_index) {
  vector<int > neighbors_to_current = neighbor_cube_list[box_index];
  vector<int > return_vec;
  for (int i = 0; i < neighbors_to_current.size(); i++) {
    vector<int > neighbor_particles = grid_locs[i];
    for (int j = 0; j < neighbor_particles.size(); j++) {
      return_vec.push_back(neighbor_particles[j]);
    }
  }
  return return_vec;
} 
//****************************************************
// Distance between two vectors
//****************************************************
float vec3dist(glm::vec3 v1, glm::vec3 v2) {
   	float dx = abs(v2.x - v1.x);
   	float dy = abs(v2.y - v1.y);
   	float dz = abs(v2.z - v1.z);
   	float dist = sqrt(dx*dx+dy*dy+dz*dz);
   	return dist;
}
//****************************************************
// Prints out a vec3
//****************************************************
void vec3Print(glm::vec3 v1) {
   	cout<<"Vec3: "<<v1.x<<" "<<v1.y<<" "<<v1.z<<"\n";
}
//****************************************************
// Length of a vector (magnitude)
//****************************************************
float vec3length(glm::vec3 v1) {
   	return sqrt(v1.x*v1.x+v1.y*v1.y+v1.z*v1.z);
}
//****************************************************
// Initializes density of all particles
//****************************************************
void initDensity() {
	for (int i = 0; i < particles.size(); i++) {
		densities[i] = 0.0f;
	}
}
//****************************************************
// Clears pressure of all particles
//****************************************************
void clearPressures() {
	for (int i = 0; i < pressures.size(); i++) {
		pressures[i] = glm::vec3(0.0f);
	}
}
//****************************************************
// Clears viscosity of all particles
//****************************************************
void clearViscosities() {
	for (int i = 0; i < viscosities.size(); i++) {
		viscosities[i] = glm::vec3(0.0f);
	}
}
//****************************************************
// Clears color-field gradient of all particles
//****************************************************
void clearCFGrad() {
	for (int i = 0; i < cfGrad.size(); i++) {
		cfGrad[i] = glm::vec3(0.0f);
	}
}
//****************************************************
// Clears color-field laplacian of all particles
//****************************************************
void clearCFLap() {
	for (int i = 0; i < cfLap.size(); i++) {
		cfLap[i] = 0.0f;
	}
}
//****************************************************
// Calculates particle densities based on position
//****************************************************
void calculateDensities() { 
	float h = SMOOTHING_LENGTH;
    for (int i = 0; i< particles.size(); i++) {
    	vector<int> neighbors = grid.consolidateNeighbors(particles[i].gridIndex);
    	for (int j = 0; j<neighbors.size(); j++) {
    		if (particles[i].position.x == particles[j].position.x && particles[i].position.y == particles[j].position.y && particles[i].position.z == particles[j].position.z){
        		continue;
      		}
      		else{
				glm::vec3 r = (particles[i].position) - (particles[j].position);
				float rDist = vec3length(r);
				if (rDist <= h) {
					densities[i] = densities[i] + PARTICLE_MASS * w_poly6(r,rDist,h);
				}
			}
    	}
    }
}
//****************************************************
// Calculates other particle forces
//****************************************************  
void calculateForces() {  
	float h = SMOOTHING_LENGTH;
    for (int i = 0; i <particles.size(); i++) {
    	vector<int> neighbors = grid.consolidateNeighbors(particles[i].gridIndex);
    	for (int j = 0; j<neighbors.size(); j++) {
			if (particles[i].position.x == particles[j].position.x && particles[i].position.y == particles[j].position.y && particles[i].position.z == particles[j].position.z){
				continue;
			}
			else{
				glm::vec3 r = (particles[i].position) - (particles[j].position);
				float rDist = vec3length(r);
				if (rDist <= h) {
					float density_p = densities[i];
					float density_n = densities[j];
					float pressure_p = K * density_p - REST_DENSITY;
					float pressure_n = K * density_n - REST_DENSITY;
				
					pressures[i] = pressures[i] + PARTICLE_MASS * (pressure_p + pressure_n) / (2 * density_n) * gradient_w_spiky(r,rDist,h);
					viscosities[i] = viscosities[i] + eta * PARTICLE_MASS * (particles[j].velocity - particles[i].velocity) / density_n * laplacian_w_viscosity(r,rDist,h);
					cfGrad[i] = cfGrad[i] + PARTICLE_MASS / density_n * gradient_w_poly6(r,rDist,h);
					cfLap[i] = cfLap[i] + PARTICLE_MASS / density_n * laplacian_w_poly6(r,rDist,h);
				}
    		}
    	}
    }
    //cout<<"Press: "<<pressures[0][0]<<" "<<pressures[0][1]<<" "<<pressures[0][2]<<endl;
}
//****************************************************
// Loops through particles and updates their positions
//****************************************************
void calculatePositions() {
	float h = SMOOTHING_LENGTH;
    for (int i = 0; i < particles.size(); i++) {
    	float gradient_length = vec3length(cfGrad[i]);
    	glm::vec3 surface_tension;
    	if (gradient_length >= threshold) {
    		surface_tension = -1.0f * SIGMA * cfLap[i] * cfGrad[i] / gradient_length;
    	}
    	else{
    		surface_tension = glm::vec3(0.0f, 0.0f, 0.0f);
    	}
    	//cout<<"ST: "<<surface_tension[0]<<" "<<surface_tension[1]<<" "<<surface_tension[2]<<endl;
    	
    	glm::vec3 total_force = surface_tension + pressures[i] + viscosities[i];
    	glm::vec3 acceleration = total_force / densities[i] * TIME_STEP + GRAVITY;
    	
    	particles[i].velocity = particles[i].velocity + acceleration * TIME_STEP;
    }

    for (int i = 0; i< particles.size(); i++) {
    	//cout<<"Naively updating particles\n";
    	if(particles[i].willHitBoundingBox(TIME_STEP)){
			float t = TIME_STEP;
			while (particles[i].willHitBoundingBox(t)) {
				particles[i].position = particles[i].position + (particles[i].velocity * timeToHit);
				particles[i].velocity = particles[i].velocity * dampVec;
				t = timeRemainder;
			}
			particles[i].position = particles[i].position + (particles[i].velocity * timeRemainder);
		}
		else{
			particles[i].position = particles[i].position + (particles[i].velocity * TIME_STEP);
		}
    }
    for (int i = 0; i < particles.size(); i++) {
    	int id = particles[i].findCollision();
    	if (id != -1){
    		glm::vec3 collisionNormal = glm::normalize(particles[i].position - particles[id].position);
    		particles[i].velocity = glm::rotate(particles[i].velocity, 179.0f, collisionNormal);
    		particles[id].velocity = glm::rotate(particles[id].velocity, 181.0f, collisionNormal);
    	}
    }
}
//****************************************************
// Smoothing Kernel Functions
//****************************************************
float w_poly6(glm::vec3 r, float R, float h) {
  if (0 <= R && R <= h) {
    return (315 / (64 * PI * h*h*h*h*h*h*h*h*h*h)) * ((h*h - R*R)*(h*h - R*R)*(h*h - R*R));
  }
  return 0.0f;  
}

glm::vec3 gradient_w_poly6(glm::vec3 r, float R, float h) {
	glm::vec3 neg_r = r * -1.0f;
	float weight = ((945 / (32 * PI * h*h*h*h*h*h*h*h*h)) * (h*h - R*R)*(h*h - R*R));
 	return neg_r * weight;
}

float laplacian_w_poly6(glm::vec3 r, float R, float h) {
 return (945 / (8 * PI * h*h*h*h*h*h*h*h*h)) * ((h * h - R * R) * (h * h - R * R)) * (R * R - 0.75 * (h * h - R * R) * (h * h - R * R));

}

glm::vec3 gradient_w_spiky(glm::vec3 r, float R, float h) {
	glm::vec3 neg_r = r * -1.0f;
	float weight = ((45 / (PI * h*h*h*h*h*h * R)) * (h-R)*(h-R));
  	return neg_r * weight;
}

float laplacian_w_viscosity(glm::vec3 r, float R, float h)  {
  return (45 / (PI * h*h*h*h*h)) * (1 - (R / h));
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
	if (grav) {
		GRAVITY = glm::vec3(0.0f, 0.0f, 0.0f);
	}
	generateParticles(numParts);
}
//****************************************************
// function for handling keypresses
//****************************************************
/*
void getKeys(unsigned char key, int x, int y) {
	switch(key) {
		default:{
			Particle particle;
			particle.position = glm::vec3(0.0f, 0.5f, -3.0f);
			int k = 4;
			if (key %2 == 0) {
				k = -4;
			}
			//particle.velocity = glm::vec3((key-96), 0, (int)(key-96)/k);
			particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
			densities.push_back(1.0f);
			pressures.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
			cfLap.push_back(1.0f);
    		cfGrad.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
    		viscosities.push_back(glm::vec3(0.0f,0.0f,0.0f));
			particles.push_back(particle);
			cout << "Particle added! Total particles: " << particles.size() << endl;
			break;
		}
	}
}
*/
//****************************************************
// function for handling keypresses
//****************************************************
void getKeys(unsigned char key, int x, int y) {
    switch(key) {
      case ' ': {
  			exit(0);
  			break;
		  }
      case 'p': {
        for (int i = 0; i < particles.size(); i++) {
          if (particles[i].velocity.y < 0.0f) {
            particles[i].velocity.y = 5.0f;
          }
          else {
            particles[i].velocity.y *= -1.0f;
          }
        }
        break;
      }
      case 'c': {
        PRESSURE_MAPPED = ! PRESSURE_MAPPED;
        break;
      }
      case 'x': {
        PAUSED = ! PAUSED;
        break;
      }
      case 'b': {
        RENDER_BACKGROUND = ! RENDER_BACKGROUND;
        break;
      }
      case 'g': {
        grav = ! grav;
        break;
      }
      default:{
      	Particle particle;
			  particle.position = glm::vec3(-0.70f, 0.5f, -3.5f);
			  int k = 4;
        //NOTE: B AND A CAUSE A BUS ERROR
		    if (key %2 == 0) {
				  k = -4;
			  }
  			particle.velocity = glm::vec3((key-94), 0, (int)(key-94)/k);
        particle.gridIndex = 0;
  			particles.push_back(particle);
  			cout << "Particle added! Total particles: " << particles.size() << endl;
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
	glLineWidth(1);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(box.nll.x, box.nll.y, box.nll.z);
	glVertex3f(box.nlr.x, box.nlr.y, box.nlr.z);
	glVertex3f(box.ntr.x, box.ntr.y, box.ntr.z);
	glVertex3f(box.ntl.x, box.ntl.y, box.ntl.z);
	glVertex3f(box.nll.x, box.nll.y, box.nll.z);
	glEnd();

	glLineWidth(1);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(box.fll.x, box.fll.y, box.fll.z);
	glVertex3f(box.flr.x, box.flr.y, box.flr.z);
	glVertex3f(box.ftr.x, box.ftr.y, box.ftr.z);
	glVertex3f(box.ftl.x, box.ftl.y, box.ftl.z);
	glVertex3f(box.fll.x, box.fll.y, box.fll.z);
	glEnd();

	glLineWidth(1);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINES);
	glVertex3f(box.nll.x, box.nll.y, box.nll.z);
	glVertex3f(box.fll.x, box.fll.y, box.fll.z);
	glEnd();

	glLineWidth(1);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINES);
	glVertex3f(box.ntl.x, box.ntl.y, box.ntl.z);
	glVertex3f(box.ftl.x, box.ftl.y, box.ftl.z);
	glEnd();

	glLineWidth(1);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINES);
	glVertex3f(box.ntr.x, box.ntr.y, box.ntr.z);
	glVertex3f(box.ftr.x, box.ftr.y, box.ftr.z);
	glEnd();

	glLineWidth(1);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINES);
	glVertex3f(box.nlr.x, box.nlr.y, box.nlr.z);
	glVertex3f(box.flr.x, box.flr.y, box.flr.z);
	glEnd();

	glPopMatrix();
}
//****************************************************
// Generate particles in rows
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
		particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		//particle.velocity = glm::vec3((float)(number%5), -1*((float)(number%7)), (float)(number%11));
    	particle.gridIndex = 0;
		densities.push_back(1.0f);
		pressures.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
		cfLap.push_back(1.0f);
    	cfGrad.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
    	viscosities.push_back(glm::vec3(0.0f,0.0f,0.0f));
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
		float total_pressure = 1000.0f;
		if (PRESSURE_MAPPED && (abs(pressures[i].x) + abs(pressures[i].y) + abs(pressures[i].z)) > total_pressure) {
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
			glColor4f(1.0, 1.0, 1.0, 1.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

			glColor4f(1.0, 1.0, 1.0, 1.0);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, bgSpecular);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, bgShininess);
		}
		else {
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
			glColor4f(0.3, 0.4, 0.9, 1.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
		}

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
	if (! PAUSED) {
		CURRENT_TIME += TIME_STEP;
		//Scene Setup
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the color buffer (sets everything to black)
		glViewport(0, 0, viewport.w, viewport.h);
		//Camera Setup:
		glColor3f(0.0, 1.0, 0);
		glMatrixMode(GL_PROJECTION);                  
		glLoadIdentity();                            
		gluPerspective(45.0f, viewport.w/viewport.h, 0.1, 100.0f);
		/*
		gluPerspective(fov * zoom, aspect_ratio, z_near, z_far);
		*/
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity(); // make sure transformation is "zero'd"
    	gluLookAt(1.0, 1.75, 5.0f, 1.0, 0.5f, 0, 0, 1, 0);
		/* Main Rendering loop: uses naive neighbor calculation ==> O(n^2) */
		initDensity();
		clearPressures();
		clearViscosities();
		clearCFGrad();
		clearCFLap();
		calculateDensities();
		calculateForces();
		calculatePositions();
		drawParticles();
	}
	drawBoundingBox();
	if (RENDER_BACKGROUND) {
		drawBackground();
	}

	//Testing Neighbors algorithm
	grid.updateGrid(particles);
	for (int j = 0; j < particles.size(); j++) {
		if (particles[j].gridIndex > 0) {
			cout << "hits" << endl;
		}
	}
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
//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);
  //This tells glut to use a double-bufferd window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
  // Initalize theviewport size
  //box = BoundingBox(glm::vec3(-.75,-.75,-3), 1.5);
  box = BoundingBox(glm::vec3(0, 0, 2), 2.0);
  grid = Grid(0.0f, 2.0f, SMOOTHING_LENGTH);
  //grid.initialize();
  //grid.updateGrid(particles);
  viewport.w = VIEWPORT_WIDTH;
  viewport.h = VIEWPORT_HEIGHT;
  
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
	else if (strcmp(argv[i], "-visc") == 0){
		eta = atof(argv[i+1]);
		i++;
	}
	else if (strcmp(argv[i], "-g") == 0){
		grav = true;
		i++;
	}
  }
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Fluid Simulator");
  initScene();     // quick function to set up scene
  grid.initialize();
  grid.updateGrid(particles);
  cout<<"My display called\n";
  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  cout<<"My display finished\n";
  glutKeyboardFunc(getKeys);
  
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  return 0;
}