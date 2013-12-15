#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <SOIL.h>
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
    float pressure;
    glm::vec3 velocity;
    float density;
    bool hitsBoundingBox();
    ~Particle(){};
    void draw();
    vector<Particle > neighbors;
    Particle(glm::vec3 pos, glm::vec3 vel);
};
class Viewport {
  public:
    int w, h; // width and height
};

/*
 * Bounding box specifies the container into which the particles will fall.
 */

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
    float sideLength;
    BoundingBox(){};
    BoundingBox(glm::vec3 start, float length);
    bool hitsBoundary(glm::vec3 position, glm::vec3 radius);
};

class Grid {
  vector < vector <int > > grid_locs;
  vector < vector <int > > grid_neighbors;
  float min_val;
  float max_val;
  float cube_length;
  int total_cubes;

  public: 
    Grid(){};
    Grid(float min_val, float max_val, float cube_length);
    void clearGrid();
    void initializeNeighbors();

    int insertCube(int particle_index, glm::vec3 position, float cube_length);
    void updateGrid(vector<Particle> particles);
    void validateCubeIndex(int k);
    //vector<int > generateNeighbors(int box_index, int cube_length);
    void generateNeighbors(int box_index, int cube_length);
};


Grid::Grid(float minx, float maxx, float side_length) {
  min_val = minx;
  max_val = maxx;
  cube_length = side_length;
  total_cubes = int (max_val - min_val) / cube_length;
}

void Grid::initializeNeighbors() {
  int max_cube_index = total_cubes * total_cubes * total_cubes; 
  grid_neighbors.clear(); 
  for (int i = 0; i < max_cube_index; i++) {
    vector<int> neighbor_cell_indices;
    grid_neighbors.push_back(neighbor_cell_indices);
  }
}

void Grid::clearGrid() {
  int max_cube_index = total_cubes * total_cubes * total_cubes; 
  /*
  cout << "Total cubes: " << total_cubes << endl;
  cout << "Max number of cubes = " << max_cube_index << endl;
  */

  grid_locs.clear();
  for (int i = 0; i < max_cube_index; i++) {
    vector<int> neighbor_indices;
    grid_locs.push_back(neighbor_indices);
  }
}

int Grid::insertCube(int particle_index, glm::vec3 position, float cube_length) {
  int column_offset = floor(position.x / cube_length);
  int row_offset = floor(position.y / cube_length);
  int depth_offset = floor(position.z / cube_length);

  int total_cubes_squared = total_cubes * total_cubes;
  int total_cubes_cubed = total_cubes_squared * total_cubes;

  int grid_locs_index = column_offset + row_offset * (total_cubes)  + depth_offset * (total_cubes_squared);
  if (grid_locs_index > (total_cubes_cubed) - 1) {
      cout << "Broken index at:  " << grid_locs_index << endl;
      return -1000;
  }
  else {
    //cout << "hits here " << endl;
    //Temporary Hack: set any negative value to the 0th box to avoid seg faults: fix w/ bounding box
    if (grid_locs_index < 0) {
      grid_locs[0].push_back(particle_index);
    }
    else {
      grid_locs[grid_locs_index].push_back(particle_index);
    }
  }
  return grid_locs_index;
}
void Grid::updateGrid(vector<Particle> particles){
  clearGrid();
  for (int i = 0; i < particles.size(); i++) {
    glm::vec3 particle_position = particles[i].position;
    int grid_index = insertCube(i, particle_position, cube_length);
    //cout << "Inserted at grid number: " << grid_index << endl;
    if (grid_index == -1000) {
      cout << "Broke at particle position : " << particle_position.x << ", " << particle_position.y << ", " << particle_position.z << endl;
      break;
    }
  }
  for (int i = 0; i < grid_locs.size(); i++) {
    if (grid_locs[i].size() != 0) {
      //cout << "Size of grid_locs[" << i << "]: " << grid_locs[i].size();
     // cout << endl;
    }
  }
}

void Grid::validateCubeIndex(int k) {
  if (k < 0 || k > total_cubes) {
    cout << "Invalid index at " << k << endl;
  }
}
void Grid::generateNeighbors(int box_index, int cube_length) {
  vector<int > neighborCubes;


  /* The following variables are defined to calculate the edge cases for the cube grid
  *  box_index: index from 0 to total_cubes to identify which box it is
  *  cube_length: the width of a face of the bounding cube
  *  cube_length_squared: the number of cubes in one face of the bounding cube
  *  total_cube_size: the total number of cubes in the bounding cube ( == cube_length ^ 3)
  */

  int cube_length_squared = cube_length * cube_length;
  int total_cube_size = cube_length_squared * cube_length;
  if (box_index > grid_neighbors.size()) {
    cout << "SOMETHING WENT WRONG IN generateNeighbors!!" << endl;
    cout << "boxIndex > grid_neighbors.size()" << endl;
  }

  // Back Face 
  if (box_index < cube_length_squared) {
    //Back face left column
    if (box_index % cube_length == 0) {
      //Back face - left column - lower left corner
      if (box_index == 0) {

      }
      //Back face - left column - upper left corner
      if (box_index == (cube_length_squared - cube_length)) {

      }
      // Back face - left column - not a corner
      else {

      }
    }
    // Back face - right column
    else if (box_index % cube_length == (cube_length - 1)) {
      //Back face - right column - lower right corner
      if (box_index == (cube_length - 1)) {

      }
      //Back face - right column - upper right corner
      else if (box_index == (cube_length_squared - 1)) {

      }
      //Backface - right column - not a corner
      else {

      }

    }
    else {
      //Back face - bottom row - not corner
      if (box_index % cube_length_squared < cube_length) {

      }
      //Back face - top row - not corner
      else if (box_index % cube_length_squared > (cube_length_squared - cube_length)) {

      }
      //Back face - not row/column cubes
      else {

      }
    }
  }

  //Front face
  else if (box_index >= (total_cubes - cube_length_squared)) {
    //Front face - left column
    if (box_index % cube_length == 0) {
      //Front face - left column - lower left corner
      if (box_index == (total_cubes - cube_length_squared)) {

      }
      //Front face - left column - upper left corner 
      else if (box_index == (total_cubes - cube_length)) {

      }
      //Front face - left column - not a corner
      else {

      }
    }
    //Front face - right column

    else if (box_index % cube_length == (cube_length - 1)) {

      //Front face - right column - lower right corner
      if (box_index == (total_cubes - cube_length_squared + (cube_length - 1))) {

      }
      //Front face - right column - upper right corner
      else if (box_index == total_cubes - 1 ) {

      }

      //Front face - right column - not a corner
      else {

      }
    }
    //Front face - not a column 
    else {
      //Front face - bottom row - not corner
      if (box_index % cube_length_squared < (total_cubes - cube_length_squared + cube_length)) {

      }
      //Front face - top row - not corner
      else if (box_index % cube_length_squared > (total_cubes - cube_length)) {

      }
      //Front face - not row/column cubes
      else {

      }
    }
  }
  

  // Left face
  else if (box_index % cube_length == 0) {
    //Left face, back column
    if (box_index < cube_length_squared)  {

    }
    //Left face, near column
    else if (box_index >= (total_cubes - cube_length_squared)) {

    }
    //Left face, top row
    else if (box_index % cube_length_squared == (cube_length_squared - cube_length)) {

    }
    //Left face, bottom row
    else if (box_index % cube_length_squared < cube_length) {

    }
    //Left face, every other value
    else {

    }
  }

  //Right face
  else if (box_index % cube_length == (cube_length - 1)) {
    //Right face, back column
    if (box_index < cube_length_squared)  {

    }
    //Right face, front column
    else if (box_index >= (total_cubes - cube_length_squared)) {

    }
    //Right face, top row
    else if (box_index % cube_length_squared == (cube_length_squared - 1)) {
    }
    //Right face, bottom row
    else if (box_index % cube_length_squared < cube_length) {
    }
    //Right face, any other value
    else {
    }
  }

  //Top face 
  else if (box_index % cube_length_squared >= (cube_length_squared - cube_length)){
    //Should just be one case: remaining squares inside
  }
  //Bottom face
  else if (box_index % cube_length_squared < cube_length) {
    //Should just be one case: remaining squares inside
  }
  //The rest of the cubes: everything inside of the cube
  else {
  }

}





/* BoundingBox takes in a start vertex, which specifies the NEAR LOWER LEFT CORNER of the bounding 
 * box, while sideLength represents the length of each side of the cube. When hitsBoundary gets called,
 * there is a check to see if a particle collides. 
 */

BoundingBox::BoundingBox(glm::vec3 start, float length) {
    sideLength = length;
    nll = start;
    
    ntl = start;
    ntl.y += length;
    
    nlr = start;
    nlr.x += length;
    
    ntr = start;
    ntr.x += length;
    ntr.y += length;
    
    fll = start;
    fll.z -= length;
    ftl = start;
    ftl.z -= length;
    ftl.y += length;
    
    flr = start;
    flr.x += length;
    flr.z -= length;
    
    ftr = start;
    ftr.x += length;
    ftr.y += length;
    ftr.z -= length;
}


bool BoundingBox::hitsBoundary(glm::vec3 position, glm::vec3 radius) {
  return false;
}

bool Particle::hitsBoundingBox() {
  return false;
}

Particle::Particle(glm::vec3 pos, glm::vec3 vel) {
        position = pos;
        velocity = vel;
}

class AABB {
	public:
		glm::vec3 min;
		glm::vec3 max;
		bool intersectsObj(Particle part, float radius);
		int intersectsObjSide(Particle part, float radius);
		AABB(vector<vector<glm::vec3>> verts);
		AABB(){};
};

/*
	AABB defines bounding box for obj
*/
bool AABB::intersectsObj(Particle part, float radius) {
			
	if (part.position.x +radius < min.x ||
		part.position.x -radius > max.x ||
		part.position.y +radius < min.y ||
		part.position.y -radius > max.y ||
		part.position.z +radius < min.z ||
		part.position.z -radius > max.z) {
			return false;}
	return true;
}

//returns side that a particle hits, otherwise 0
int AABB::intersectsObjSide(Particle part, float radius) {
	float r = radius;
	float x = part.position.x;
	float y = part.position.y;
	float z = part.position.z;
	if ( x+r == min.x && y-r <= max.y && y+r >= min.y && z-r <= max.z && z+r >= min.z) {return 1;} //left face
	if ( x-r == max.x && y-r <= max.y && y+r >= min.y && z-r <= max.z && z+r >= min.z) {return 2;} //right face
	if ( z+r == min.z && y-r <= max.y && y+r >= min.y && x-r <= max.x && x+r >= min.x) {return 3;} //back face
	if ( z-r == max.z && y-r <= max.y && y+r >= min.y && x-r <= max.x && x+r >= min.x) {return 4;} //front face
	if ( y+r == min.y && z-r <= max.z && z+r >= min.z && x-r <= max.x && x+r >= min.x) {return 5;} //bottom face
	if ( y-r == max.y && z-r <= max.z && z+r >= min.z && x-r <= max.x && x+r >= min.x) {return 6;} //top face
	return 0;
}

//generates a bounding box for an object based on its vertices
AABB::AABB(vector<vector<glm::vec3>> verts) {
	min = glm::vec3(0,0,0);
	max = glm::vec3(0,0,0);
	for (int i=0; i < verts.size(); i++) {
		for (int j=0; j < verts[i].size(); j++){
			if (verts[i][j].x < min.x) {min.x = verts[i][j].x;}
			if (verts[i][j].x > max.x) {max.x = verts[i][j].x;}
			if (verts[i][j].y < min.y) {min.y = verts[i][j].y;}
			if (verts[i][j].y > max.y) {max.y = verts[i][j].y;}
			if (verts[i][j].z < min.z) {min.z = verts[i][j].z;}
			if (verts[i][j].z > max.z) {max.z = verts[i][j].z;}
		}
	}
}

void translateVertices(vector<vector<glm::vec3>> vert, float x, float y, float z) {
	for(int i=0; vert.size(); i++) {
		for(int j=0; vert[i].size(); j++) {
			vert[i][j].x += x;
			vert[i][j].y += y;
			vert[i][j].z += z;
		}
	}
}



//****************************************************
// Global Variables
//****************************************************

#define VIEWPORT_HEIGHT 1200;
#define VIEWPORT_WIDTH  1200;
Viewport    viewport;
BoundingBox box;
Grid grid;
float CURRENT_TIME = 0.0f;
float TIME_STEP = 0.0025f;

float PARTICLE_RADIUS = 0.05f;
float PARTICLE_MASS = 1.05f;

float SMOOTHING_LENGTH = 0.5;

float K = 5.0f;
float REST_DENSITY = 10.0f;

glm::vec3 GRAVITY = glm::vec3(0.0f, -9.8f, 0.0f);
vector<Particle> particles;
int numParts = 100;
GLint stacks = 10;
float VELOCITY_THRESHOLD = 0.005;
float VELOCITY_POSITION_THRESHOLD = PARTICLE_RADIUS*1.05f;

vector < vector <glm::vec3> > obj_normals;
vector < vector <glm::vec2> > obj_textures;
vector < vector <glm::vec3> > obj_vertices; 

bool obj_file = true;
string obj_type = "GL_TRIANGLE";
AABB obj_BB;
//*********************************

//****************************************************
// Declare functions for later use
//****************************************************

void generateParticles(int);
float w_poly6(float, float);
float w_pressure_gradient(float, float);

//****************************************************
// Other functions
//****************************************************
void Particle::draw() {
           glutSolidSphere(PARTICLE_RADIUS, stacks, stacks);
}

void setupParticles() {
  for (int i = 0; i < (signed)particles.size(); i++) {
    //Replace this with initialDensity
    particles[i].density = 1.0f;
    particles[i].pressure = 0.0f;
  }
}

void calculateParticleDensities() {
  float h = SMOOTHING_LENGTH;
  for (int i = 0; i < (signed)particles.size(); i++) {
    for (int j = 0; j < (signed)particles.size(); j++) {
      if (i == j) {
        continue;
      }
      glm::vec3 position_i = particles[i].position;
      glm::vec3 position_j = particles[j].position;

      float distance_x = position_j.x - position_i.x;
      distance_x = glm::dot(distance_x, distance_x);

      float distance_y = position_j.y - position_i.y;
      distance_y = glm::dot(distance_y, distance_y);

      float distance_z = position_j.z - position_j.z;
      distance_z = glm::dot(distance_z, distance_z);

      float r = sqrt((distance_x + distance_y + distance_z));
      if (r <= h) {
        particles[i].density = particles[i].density + (PARTICLE_MASS * w_poly6(r, h));
      }
    }
  }
}

void calculateParticleForces() {
  float h = SMOOTHING_LENGTH;
  for (int i = 0; i < (signed)particles.size(); i++) {
    for (int j = 0; j < (signed)particles.size(); j++) {
      if (i == j) {
        continue;
      }
      glm::vec3 position_i = particles[i].position;
      glm::vec3 position_j = particles[j].position;

      float distance_x = position_j.x - position_i.x;
      distance_x = glm::dot(distance_x, distance_x);

      float distance_y = position_j.y - position_i.y;
      distance_y = glm::dot(distance_y, distance_y);

      float distance_z = position_j.z - position_j.z;
      distance_z = glm::dot(distance_z, distance_z);

      float r = sqrt((distance_x + distance_y + distance_z));

      float density_p = particles[i].density;
      float density_n = particles[j].density;

      float pressure_p = K * (density_p - REST_DENSITY);
      float pressure_n = K * (density_n - REST_DENSITY);

      particles[i].pressure = w_pressure_gradient(r, h) * (PARTICLE_MASS * (pressure_p + pressure_n)) / (2 * density_n);
    }
  }



}

void updateParticlePositions() {
  for (int i = 0; i < (signed)particles.size(); i++) {
    /* Gradient/color field calculations go here
    * add in surface_tension, pressure, and viscosity later
    */ 

    glm::vec3 surface_tension = glm::vec3(0.0f, 0.0f, 0.0f); 
    glm::vec3 particle_pressure = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 viscosity = glm::vec3(0.0f, 0.0f, 0.0f);

    static int pers = 0;
    glm::vec3 total_force = surface_tension + particle_pressure + viscosity;
    glm::vec3 acceleration = (total_force / particles[i].density) * TIME_STEP+GRAVITY;
    

    bool slow = abs(particles[i].velocity.y) < VELOCITY_THRESHOLD;
    particles[i].velocity = particles[i].velocity + (acceleration * TIME_STEP);
    bool should_stop = particles[i].velocity.y < VELOCITY_THRESHOLD;
    bool close_to_bottom = particles[i].position.y - box.nll.y < VELOCITY_POSITION_THRESHOLD;
    if (slow && should_stop && close_to_bottom) {
            particles[i].velocity.y = 0.0f;
    }
    particles[i].position = particles[i].position + (particles[i].velocity * TIME_STEP);

    if ((particles[i].position.y) < box.nll.y + PARTICLE_RADIUS) {
      particles[i].velocity *= -0.71f;
    }
    
  }  
      // cout<<"POS:"<<' '<<particles[1].position[0]<<' '<<particles[1].position[1]<<' '<<particles[1].position[2]<<'\n';
      // cout<<"VEL:"<<' '<<particles[1].velocity[0]<<' '<<particles[1].velocity[1]<<' '<<particles[1].velocity[2]<<'\n';
}

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
  /*
  cout << "Min Values: " << endl;
  cout << "Minx: " << minx << endl;
  cout << "Miny: " << miny << endl;
  cout << "Minz: " << minz << endl;
  */
}


//****************************************************
// Smoothing Kernel functions
//****************************************************
float w_poly6(float r, float h) {
  if (0 <= r && r <= h) {
    float h_squared = h * h;
    float r_squared = r * r;
    float h_r_diff = h_squared - r_squared;
    h_r_diff = pow(h_r_diff, 3);

    float h_ninth = pow(h, 9);

    float weight = (315 / (64 * PI * h_ninth)) * (h_r_diff);
    return weight;
  }
  return 0.0f;  
}

float w_pressure_gradient(float r, float h) {
  float h_sixth = pow(h, 6);

  float h_r = h - r;
  float h_r_squared = pow(h_r, 2);

  float weight = (-1.0f * r) * (45 / (PI * h_sixth * r)) * h_r_squared;
  return weight;
}

float w_viscosity_laplacian(float r, float h)  {
  float h_fifth = pow(h, 5);
  float weight = (45 / (PI * h_fifth)) * (1 - (r / h));
  return weight;
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

void getKeys(unsigned char key, int x, int y) {
    switch(key) {
      case ' ': {
        exit(0);
      }
        break;
      case 'n': {
        Particle particle;
        particle.position = glm::vec3(0.0f, 1.0f, -2.5f);
        particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
        particles.push_back(particle);
        cout << "Particle added! Total particles: " << particles.size() << endl;
      }
        break;
      case 'h':
        break;
      case '+': 
        break;
      default:
        break;
      }
  }


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
        float x = box.ntl.x + PARTICLE_RADIUS;
        float y = box.ntl.y + PARTICLE_RADIUS;
        float z = box.ntl.z - PARTICLE_RADIUS;
        while (number > 0) {
                Particle particle;
                particle.position = glm::vec3(x+xMult*distance, y+yMult*distance, z+zMult*distance);
                particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
                particles.push_back(particle);
                zMult--;
                if (z+zMult*distance <= box.ftl.z - PARTICLE_RADIUS) {
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
// Draw Object
//****************************************************
void drawObject() {
	if(obj_file) {
	  	glPushMatrix();                               
		glEnable(GL_TEXTURE_2D);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
		
		//glTranslatef(0,.25,-4.5);
		//glScalef(.1,.1,.1);
		for (int i=0; i<obj_vertices.size(); i++) {
			if (obj_type == "GL_QUADS") {
				glBegin(GL_QUADS);
			} else {glBegin(GL_POLYGON);}
			cout <<"drawing obj" << endl;	
			glTexCoord2f(obj_textures[i][0].x, obj_textures[i][0].y);
			glNormal3f(obj_normals[i][0].x, obj_normals[i][0].y, obj_normals[i][0].z);
			glVertex3f(obj_vertices[i][0].x, obj_vertices[i][0].y, obj_vertices[i][0].z);

			glTexCoord2f(obj_textures[i][1].x, obj_textures[i][1].y);
			glNormal3f(obj_normals[i][1].x, obj_normals [i][1].y, obj_normals[i][1].z);
			glVertex3f(obj_vertices[i][1].x, obj_vertices[i][1].y, obj_vertices[i][1].z);

			glTexCoord2f(obj_textures[i][2].x, obj_textures[i][2].y);
			glNormal3f(obj_normals[i][2].x, obj_normals[i][2].y, obj_normals[i][2].z);
			glVertex3f(obj_vertices[i][2].x, obj_vertices[i][2].y, obj_vertices[i][2].z);

			if (obj_type == "GL_QUADS" ) {
			    glTexCoord2f(obj_textures[i][3].x, obj_textures[i][3].y);
				glNormal3f(obj_normals[i][3].x, obj_normals[i][3].y, obj_normals[i][3].z);
				glVertex3f(obj_vertices[i][3].x, obj_vertices[i][3].y, obj_vertices[i][3].z);
			}

			glEnd();
	}
       glPopMatrix();
	   glDisable(GL_TEXTURE_2D);
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
 // gluLookAt(0,0,10,0,0,0,0,1,0);
  /*
  gluPerspective(fov * zoom, aspect_ratio, z_near, z_far);
  */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); // make sure transformation is "zero'd"
  drawObject();
  /* Main Rendering loop: uses naive neighbor calculation ==> O(n^2) */
  setupParticles();
  calculateParticleDensities();
  calculateParticleForces();
  updateParticlePositions();
  /* Drawing sphere loop */
  drawBoundingBox();
  drawBackground();
  drawParticles();
  //Scene cleanup

  findMinParticlePositions();

  //Testing Neighbors algorithm
  grid.clearGrid();
  grid.updateGrid(particles);

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
	vector < glm::vec3 > temp_vertices;
	vector < glm::vec3 > temp_normals;
	vector < glm::vec2 > temp_textures;

	//cout << file << endl;
  std::ifstream inpfile(file.c_str());
  if(!inpfile.is_open()) {
    std::cout << "Unable to open file" << std::endl;
  } else {
    std::string line;
   obj_file = true;
    while(inpfile.good()) {
	//cut << "generating obj" << endl;
      std::vector<std::string> splitline;
      std::string buf;

      std::getline(inpfile,line);
      std::stringstream ss(line);
	  //std::cout << splitline[0] << endl;
      while (ss >> buf) {
        splitline.push_back(buf);
      }
	  //Ignore blank lines
      if(splitline.size() == 0) {
        continue;
      }
	  if (splitline[0] == "GL_QUADS") {
		  obj_type = "GL_QUADS";
	  } 
	  //Applies any translations/scales on the object
	  if(splitline[0] == "v") {
		glm::vec3 vertex = glm::vec3(
					atof(splitline[1].c_str())/13, 
					atof(splitline[2].c_str())/13 + .15,
                    atof(splitline[3].c_str())/13 -4.0);
		temp_vertices.push_back(vertex);
	  }
	  if(splitline[0] == "vn") {
		  glm::vec3 vertex = glm::vec3(
			  		atof(splitline[1].c_str()), 
					atof(splitline[2].c_str()),
                    atof(splitline[3].c_str()));
		temp_normals.push_back(vertex);
      }
	  
	  if(splitline[0] == "vt") {
		glm::vec2 vertex = glm::vec2(
					atof(splitline[1].c_str()), 
					atof(splitline[2].c_str()));
		temp_textures.push_back(vertex);
	  }
	  
	  if(splitline[0] == "f") {
		  //cout << "f" << endl;
		vector <int> vert_indices;
		vector <int> text_indices;
		vector <int> norm_indices;
		

			//cout << splitline[1] << " " << splitline[2] << " " << splitline[3] << endl;

			vert_indices.push_back(atoi(splitline[1].substr(0,splitline[1].find_first_of("/")).c_str()));
			vert_indices.push_back(atoi(splitline[2].substr(0,splitline[2].find_first_of("/")).c_str()));
			vert_indices.push_back(atoi(splitline[3].substr(0,splitline[3].find_first_of("/")).c_str()));

			text_indices.push_back(atoi(splitline[1].substr(splitline[1].find_first_of("/")+1, splitline[1].find_last_of("/")-1 -splitline[1].find_first_of("/")).c_str()));
			text_indices.push_back(atoi(splitline[2].substr(splitline[2].find_first_of("/")+1, splitline[2].find_last_of("/")-1 -splitline[2].find_first_of("/")).c_str()));
			text_indices.push_back(atoi(splitline[3].substr(splitline[3].find_first_of("/")+1, splitline[3].find_last_of("/")-1 -splitline[3].find_first_of("/")).c_str()));

			norm_indices.push_back(atoi(splitline[1].substr(splitline[1].find_last_of("/")+1, splitline[1].length()-1 - splitline[1].find_last_of("/")).c_str()));
			norm_indices.push_back(atoi(splitline[2].substr(splitline[2].find_last_of("/")+1, splitline[2].length()-1 - splitline[2].find_last_of("/")).c_str()));
			norm_indices.push_back(atoi(splitline[3].substr(splitline[3].find_last_of("/")+1, splitline[3].length()-1 - splitline[3].find_last_of("/")).c_str()));

		                  

  			vector <glm::vec3> verts;
			verts.push_back(temp_vertices[vert_indices[0]-1]);
			verts.push_back(temp_vertices[vert_indices[1]-1]);                           
			verts.push_back(temp_vertices[vert_indices[2]-1]);

			//cout << text_indices[0] << " " <<  text_indices[1] << " " << text_indices[2] << endl;
  		
			vector <glm::vec2> texts;
			texts.push_back(temp_textures[text_indices[0]-1]);
			texts.push_back(temp_textures[text_indices[1]-1]);                           
			texts.push_back(temp_textures[text_indices[2]-1]);
			
			vector <glm::vec3> norms;
			norms.push_back(temp_normals[norm_indices[0]-1]);
			norms.push_back(temp_normals[norm_indices[1]-1]);
			norms.push_back(temp_normals[norm_indices[2]-1]);          
			
			if (obj_type == "GL_QUADS") {
				
				vert_indices.push_back(atoi(splitline[4].substr(0,splitline[4].find_first_of("/")).c_str()));
				text_indices.push_back(atoi(splitline[4].substr(splitline[4].find_first_of("/")+1, splitline[4].find_last_of("/")-1 -splitline[4].find_first_of("/")).c_str()));
				norm_indices.push_back(atoi(splitline[4].substr(splitline[4].find_last_of("/")+1, splitline[4].length()-1 - splitline[4].find_last_of("/")).c_str()));
			  	verts.push_back(temp_vertices[vert_indices[3]-1]);
			    texts.push_back(temp_textures[text_indices[3]-1]);
				norms.push_back(temp_normals[norm_indices[3]-1]);

		   }                               
			
			obj_vertices.push_back(verts);
			obj_textures.push_back(texts);
		    obj_normals.push_back(norms);
	  }

      else if(splitline[0][0] == '#') {
        continue;
      }
	}

    inpfile.close();
  
  }
  }


//*************************************************
// Generates an OpenGL texture handle based on 
// an .obj input
//*************************************************
void getTexture() {

	GLuint obj_img = SOIL_load_OGL_texture(
					"soccer.bmp",
					SOIL_LOAD_AUTO,
					SOIL_CREATE_NEW_ID,
					SOIL_FLAG_INVERT_Y
					);
		  if(obj_img == 0) {printf("SOIL LOADING error: '%s'\n", SOIL_last_result()); }
	 cout << "img loaded" << endl;
	 glBindTexture(GL_TEXTURE_2D, obj_img);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
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
  box = BoundingBox(glm::vec3(-.70,-.75,-3), 1.5);
  //box = BoundingBox(glm::vec3(0, 0, 0), 1.5);

  grid = Grid(-0.70, 1.5, SMOOTHING_LENGTH);
  grid.initializeNeighbors();
  viewport.w = VIEWPORT_WIDTH;
  viewport.h = VIEWPORT_HEIGHT;
  //Parse input file: where OBJ files are parsed
  
  string filename = argv[1];
  parseFile(filename);
  obj_BB = AABB(obj_vertices);
  
  
  for (int i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-np") == 0){
            numParts = atoi(argv[i+1]);
            i++;
    }
        else if (strcmp(argv[i], "-rad") == 0){
                PARTICLE_RADIUS = atof(argv[i+1]);
                VELOCITY_POSITION_THRESHOLD = PARTICLE_RADIUS*1.05f;
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
  getTexture();
glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  return 0;

}