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



/* BoundingBox takes in a start vertex, which specifies the NEAR LOWER LEFT CORNER of the bounding 
 * box, while sideLength represents the length of each side of the cube. When hitsBoundary gets called,
 * there is a check to see if a particle collides. 
 */

BoundingBox::BoundingBox(glm::vec3 start, float length) {
    sideLength = length;
    nll = start;
    
    ntl = start;
    //ntl.y += length + length/4;
    ntl.y += length;
    
    nlr = start;
    nlr.x += length;
    
    ntr = start;
    ntr.x += length;
    //ntr.y += length + length/4;
    ntr.y += length;
    
    fll = start;
    fll.z -= length;
    ftl = start;
    ftl.z -= length;
    //ftl.y += length + length/4;
    ftl.y += length;
    
    flr = start;
    flr.x += length;
    flr.z -= length;
    
    ftr = start;
    ftr.x += length;
    //ftr.y += length + length/4;
    ftr.y += length;
    ftr.z -= length;
}

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
    Particle(glm::vec3 pos, glm::vec3 vel);
};

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

//****************************************************
// Global Variables
//****************************************************

#define VIEWPORT_HEIGHT 1200;
#define VIEWPORT_WIDTH  1200;
Viewport    viewport;
BoundingBox box;
float CURRENT_TIME = 0.0f;
float TIME_STEP = 0.0025f;
float PARTICLE_RADIUS = 0.05f;
float PARTICLE_MASS = 1.05f;
float SMOOTHING_LENGTH = 0.1;
float K = 5.0f;
float REST_DENSITY = 10.0f;
glm::vec3 GRAVITY = glm::vec3(0.0f, -9.8f, 0.0f);
vector<Particle> particles;
int numParts = 100;
GLint stacks = 10;
float VELOCITY_THRESHOLD = 0.005;
float VELOCITY_POSITION_THRESHOLD = PARTICLE_RADIUS*1.05f;

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
      cout<<"POS:"<<' '<<particles[1].position[0]<<' '<<particles[1].position[1]<<' '<<particles[1].position[2]<<'\n';
      cout<<"VEL:"<<' '<<particles[1].velocity[0]<<' '<<particles[1].velocity[1]<<' '<<particles[1].velocity[2]<<'\n';
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
  drawBackground();
  drawParticles();
  //Scene cleanup
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
  box = BoundingBox(glm::vec3(-.70,-.75,-3), 1.5);
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
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  return 0;

}
