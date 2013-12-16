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
    int gridIndex;
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
    float top_wall;

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
    top_wall = ntr.y;
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


//****************************************************
// Grid constructor
//****************************************************

Grid::Grid(float minx, float maxx, float side_length) {
  min_val = minx;
  max_val = maxx;
  cube_length = side_length;
  total_cubes = int (max_val - min_val) / cube_length;
}

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


void Grid::clearGrid() {
  int max_cube_index = total_cubes * total_cubes * total_cubes; 
  grid_locs.clear();
  for (int i = 0; i < occupied_locs.size(); i++) {
    vector<int> neighbor_indices;
    grid_locs[occupied_locs[i]] = neighbor_indices;
  }
  occupied_locs.clear();
}

//****************************************************
// Insert a cube into the grid
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
// Update the grid
//****************************************************
void Grid::updateGrid(vector<Particle> particles){
  clearGrid();
  for (int i = 0; i < particles.size(); i++) {
    glm::vec3 particle_position = particles[i].position;
    int grid_index = insertCube(i, particle_position, cube_length, particles);
    //cout << "particles[i].gridIndex: " << particles[i].gridIndex << endl;
    //cout << "Inserted at grid number: " << grid_index << endl;
    if (grid_index == -1000) {
      cout << "Broke at particle position : " << particle_position.x << ", " << particle_position.y << ", " << particle_position.z << endl;
      break;
    }
  }
}


//****************************************************
// Cube index validation
//****************************************************
void Grid::validateCubeIndex(int k) {
  if (k < 0 || k > total_cubes) {
    cout << "Invalid index at " << k << endl;
  }
}

//****************************************************
// Get the nearest neighbors in a cube
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
// Bounding Box collision global variables
//****************************************************
float DAMP = -0.2f;
static glm::vec3 dampVec;
float PARTICLE_RADIUS = 0.01f;
Grid grid;
BoundingBox box;

static float timeToHit = 0.0f;
static float timeRemainder = 0.0f;
float TIME_STEP = 0.0025f;
float GRADIENT_THRESHOLD = 0.55f;

//Surface tension of water
float SIGMA = 4.9;

bool PAUSED = false;
bool PRESSURE_MAPPED = false;


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

  /*
  if (velocity[1] != 0.0f && position[1] + (velocity[1] * TIME_STEP) + PARTICLE_RADIUS >= box.top_wall) {
    float t = abs(box.top_wall - position[1] - PARTICLE_RADIUS) / velocity[1];
    hit = true;
    if (t < timeToHit) {  //    cout<<"damp set\n";
      dampVec[0] = 1.0f;
      dampVec[1] = DAMP;
      dampVec[2] = 1.0f;
      timeToHit = t;
      timeRemainder = time - t;
    } //cout<<"Time to hit bottom: "<<t<<"\n";
	} 
  */

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

float SMOOTHING_LENGTH = 0.05;

float K = 5.0f;
float REST_DENSITY = 100.0f;


glm::vec3 GRAVITY = glm::vec3(0.0f, -9.8f, 0.0f);
vector<Particle> particles;
int numParts = 100;
GLint stacks = 6;

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
// Drawing a particle
//****************************************************
float vec3dist(glm::vec3 v1, glm::vec3 v2) {
   	float dx = abs(v2.x - v1.x);
   	float dy = abs(v2.y - v1.y);
   	float dz = abs(v2.z - v1.z);
   	return sqrt(dx*dx+dy*dy+dz*dz);
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
    //particles[i].density = 1.0f;
    //particles[i].gridIndex = 0.0f;
    particles[i].density = 2.2f;
    particles[i].viscosity = glm::vec3(0.0f, 0.0f, 0.0f);
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
    //cout << "Grid index: " << particles[i].gridIndex << endl;
    vector<int > neighbors = grid.consolidateNeighbors(particles[i].gridIndex);
    /*
    for (int l = 0; l < neighbors.size(); l++) {
      cout << "Neighbor for index at : " << i << ": " << l << endl;
    }
    */
    //for (int j = 0; j < (signed)particles.size(); j++) {
    for (int j = 0; j < neighbors.size(); j++) {

      /*
      if (i == j) {
        continue;
      }
      */
      if (particles[i].position.x == particles[j].position.x && particles[i].position.y == particles[j].position.y && particles[i].position.z == particles[j].position.z){
        continue;
      }
      float R = vec3dist(particles[i].position, particles[j].position);      
      glm::vec3 r = particles[i].position - particles[j].position;
      if (R <= h) {
      	particles[i].density = particles[i].density + PARTICLE_MASS * w_poly6(r, R, h);
      }
    }
  }
}


//BS value. Might want to actually replace with proper value, if possible.
float GAS_CONSTANT = 8.314462;
float eta = 0.0008;
//float eta = 200;
//****************************************************
// Calculates other particle forces
//****************************************************

void calculateParticleForces() {

	float h = SMOOTHING_LENGTH;
	for (int i = 0; i < (signed)particles.size(); i++) {
    vector<int > neighbors = grid.consolidateNeighbors(particles[i].gridIndex);

    for (int j = 0; j < neighbors.size(); j++) {

      /*
      if (i == j) {
        continue;
      }
      */
      if (particles[i].position.x == particles[j].position.x && particles[i].position.y == particles[j].position.y && particles[i].position.z == particles[j].position.z){
        continue;
      }
    /*
		for (int j = 0; j < (signed)particles.size(); j++) {
			if (i == j) {
				continue;
			}
      */

      glm::vec3 r = particles[i].position - particles[j].position;
      float R = vec3dist(particles[i].position, particles[j].position);

			if (R <= h) {
				float density_p = particles[i].density;
				float density_n = particles[j].density;
        //cout << "Density_p for " << i << ": " << density_p << endl;
        //cout << "Density_n: for " << j << ": " << density_n << endl;
				// float pressure_p = GAS_CONSTANT * (density_p - REST_DENSITY);
				// float pressure_n = GAS_CONSTANT * (density_n - REST_DENSITY);
        float pressure_p = K * (density_p - REST_DENSITY);
        float pressure_n = K * (density_n - REST_DENSITY);

				particles[i].pressure = particles[i].pressure + (w_pressure_gradient(r, R, h) * (PARTICLE_MASS) * (pressure_p + pressure_n)/(2 * density_n));
				particles[i].viscosity = particles[i].viscosity + eta * PARTICLE_MASS * ((particles[j].velocity - particles[i].velocity) / density_n) * w_viscosity_laplacian(r, R, h);
				particles[i].cfLap = particles[i].cfLap + PARTICLE_MASS / density_n * w_poly6_laplacian(r, R, h);
				particles[i].cfGrad = particles[i].cfGrad + PARTICLE_MASS / density_n * w_poly6_gradient(r, R, h);
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
  float weight = (45 / (PI * h_sixth * R)) * h_r_squared;
  //cout << "weight: " << (neg_r * weight).x << ", " << (neg_r * weight).y << ", " << (neg_r * weight).z << endl;
  return neg_r * weight;
}

float w_viscosity_laplacian(glm::vec3 r, float R, float h)  {
  float h_fifth = pow(h, 5);

  float first_part = (45 / PI * h_fifth);
  float second_part = (1 - (R/h));
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

}


//****************************************************
// Loops through particles and updates their positions
//****************************************************

void updateParticlePositions() {
  for (int i = 0; i < (signed)particles.size(); i++) {
    /* Gradient/color field calculations go here
    * add in surface_tension, pressure, and viscosity later
    */ 

    glm::vec3 surface_tension = glm::vec3(0.0f, 0.0f, 0.0f); 
    glm::vec3 grad = particles[i].cfGrad;
    float gradient_length = sqrt(pow(grad.x, 2) + pow(grad.y, 2) + pow(grad.z, 2));

    if (gradient_length >= GRADIENT_THRESHOLD) {
      surface_tension = -1.0f * SIGMA * particles[i].cfLap * (particles[i].cfGrad / gradient_length);
    }

 
    glm::vec3 particle_pressure = particles[i].pressure;
    //surface_tension = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 viscosity = particles[i].viscosity;
    //glm::vec3 viscosity = glm::vec3(0.0f, 0.0f, 0.0f);
    //cout << "viscosity: " << viscosity.x << " " << viscosity.y << " " << viscosity.z << endl;
    glm::vec3 total_force = particle_pressure + viscosity + surface_tension;
    glm::vec3 acceleration = (total_force / particles[i].density) * TIME_STEP + GRAVITY;
    //calculate new velocity
    particles[i].velocity = particles[i].velocity + acceleration * TIME_STEP;

    // if collision imminent, update position until it hits wall, then reverse proper 
    // velocity component with damping and update position with remaining time
    if(particles[i].willHitBoundingBox(TIME_STEP)){
		float t = TIME_STEP;
		while (particles[i].willHitBoundingBox(t)) {
			particles[i].position = particles[i].position + (particles[i].velocity * timeToHit);
			particles[i].velocity = particles[i].velocity * dampVec;
			t = timeRemainder;
		}
		particles[i].position = particles[i].position + (particles[i].velocity * timeRemainder);
    }
    //otherwise, just update position
    else{
    
    particles[i].position = particles[i].position + (particles[i].velocity * TIME_STEP);
    }
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
      case ' ': {
  			exit(0);
  			break;
		  }
      /*
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
      default:{
      	Particle particle;
			  particle.position = glm::vec3(-0.70f, 0.5f, -3.5f);
			  int k = 4;
        //NOTE: B AND A CAUSE A BUS ERROR
		    if (key %2 == 0) {
				  k = -4;
			  }
  			particle.velocity = glm::vec3((key-96), 0, (int)(key-96)/k);
        particle.gridIndex = 0;
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
	float x = box.ntl.x + PARTICLE_RADIUS;
	float y = box.ntl.y - PARTICLE_RADIUS;
	float z = box.ntl.z - PARTICLE_RADIUS;
	while (number > 0) {
		Particle particle;
		particle.position = glm::vec3(x+xMult*distance, y+yMult*distance, z+zMult*distance);
		particle.velocity = glm::vec3((float)(number%5), -1*((float)(number%7)), (float)(number%11));
    particle.gridIndex = 0;
		//particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
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
  float total_pressure = 1000.0f;
  if (PRESSURE_MAPPED && (abs(particles[i].pressure.x) + abs(particles[i].pressure.y) + abs(particles[i].pressure.z)) > total_pressure) {
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
    setupParticles();
    calculateParticleDensities();
    calculateParticleForces();
    updateParticlePositions();
    drawParticles();
  }
    /* Drawing sphere loop */
    drawBoundingBox();
    //drawBackground();
    //Scene cleanup
    //findMinParticlePositions();

    //Testing Neighbors algorithm
    grid.clearGrid();
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
  //box = BoundingBox(glm::vec3(-.75,-.75,-3), 1.5);
  box = BoundingBox(glm::vec3(0, 0, 2), 2.0);
  grid = Grid(0.0f, 2.0f, SMOOTHING_LENGTH);
  grid.initialize();
  grid.updateGrid(particles);
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