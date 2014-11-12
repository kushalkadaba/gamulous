////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////
//	These skeleton codes are later altered by Ming Jin,
//	for "CS6533: Interactive Computer Graphics", 
//	taught by Prof. Andy Nealen at NYU
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <list>
#include <iostream>
#include <fstream>
// #if __GNUG__
// #   include <tr1/memory>
// #endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif


#include "arcball.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "rigtform.h"
#include "math.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include "sgutils.h"
#include "keyframe.h"
#include "frametograph.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
// using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;
// ========================================
// TODO: you can add global variables here
// ========================================
static const enum Obj{SKY,CUBE1, CUBE2}; //Viewmodes & objects to cycle through.
enum SkyMode {WORLD_SKY, SKY_SKY};
static Obj g_viewmode = SKY;			// default viewmode.
SkyMode g_activeCameraFrame = WORLD_SKY;
//static Obj g_currObj = SKY;			// default object.
static RigTForm g_defSkyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0),Quat());//holds the E matrix of default view.
bool g_showArcBall = true; //to control the visibility of the arc ball interface
double g_arcBallScreenRadius = 0.25*min(g_windowHeight,g_windowWidth);
double g_arcBallScale = 0.25*min(g_windowHeight,g_windowWidth);
bool g_isArcBallDisable = false, g_isPicking = false;
static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback

/*=================================================
	Key Frame Animation 
===================================================*/
typedef vector<RigTForm> Frame;
list<Frame>	keyFrames;
list<Frame>::iterator currentFrame = keyFrames.end();
std::vector<std::shared_ptr<SgRbtNode>> rbtNodes;
static const string END_FRAME = "end";

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3; // 3 shaders instead of 2
static const char * const g_shaderFiles[g_numShaders][3] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][3] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
  Cvec3f p, n;

  VertexPN() {}
  VertexPN(float x, float y, float z,
           float nx, float ny, float nz)
    : p(x,y,z), n(nx, ny, nz)
  {}

  // Define copy constructor and assignment operator from GenericVertex so we can
  // use make* functions from geometrymaker.h
  VertexPN(const GenericVertex& v) {
    *this = v;
  }

  VertexPN& operator = (const GenericVertex& v) {
    p = v.pos;
    n = v.normal;
    return *this;
  }
};

struct Geometry {
  GlBufferObject vbo, ibo;
  int vboLen, iboLen;

  Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
    this->vboLen = vboLen;
    this->iboLen = iboLen;

    // Now create the VBO and IBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
  }

  void draw(const ShaderState& curSS) {
    // Enable the attributes used by our shader
    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aNormal);

    // bind vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
    safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

    // bind ibo
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // draw!
    glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

    // Disable the attributes used by our shader
    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aNormal);
  }
};

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_arcBall;

// --------- Scene
// ===================================================================
// Declare the scene graph and pointers to suitable nodes in the scene
// graph
// ===================================================================

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode = g_skyNode; // used later when you do picking
static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
//static RigTForm g_skyRbt = g_defSkyRbt;
// ============================================
// TODO: add a second cube's 
// 1. transformation
// 2. color
// ============================================
/*
static RigTForm g_objectRbt[3] = {RigTForm(Cvec3(-0.65,0,0)),RigTForm(Cvec3(0.65,0,0)), RigTForm()};  // currently only 1 obj is defined
*/
static Cvec3f g_objectColors[3] = {Cvec3f(1, 0, 0),Cvec3f(0, 0, 1),Cvec3f(0,1,0)};
//static RigTForm* g_currObjRbt = &g_skyRbt; // set default to the sky camera
static shared_ptr<SgRbtNode> g_currViewRbt;// set default view to sky camera

///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
    VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3};
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initArcBall(){
	int ibLen, vbLen;
	getSphereVbIbLen(24,12,vbLen,ibLen);

	// Temporary storage for sphere geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeSphere(1,24,12,vtx.begin(),idx.begin());
  g_arcBall.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if t has reached the end of the animation
// sequence, or false otherwise. Be careful with the range of t (recall that the
// keyframes are numbered from -1 to n, while only keyframes between 0 and n-1 are
// shown
bool interpolateAndDisplay(float t) {
	if(floor(t) >= keyFrames.size()-2)
		return true;
	else
	{
		list<Frame> :: iterator baseFrame;
		baseFrame  = keyFrames.begin();
		advance(baseFrame, floor(t));
		vector<RigTForm> tform0 = (*baseFrame);
		vector<RigTForm> tform1 = (*++baseFrame);
		vector<RigTForm> interpolatedFrame;
		for(size_t i = 0; i< (*baseFrame).size(); i++){
			interpolatedFrame.push_back(lerp(tform0[i],tform1[i],t-floor(t)));
		}
		FrameToGraph f(interpolatedFrame);
		g_world->accept(f);
		glutPostRedisplay();
		return false;
	}
}
// Interpret "ms" as milliseconds into the animation
static void animateTimerCallback(int ms) {
float t = (float)ms/(float)g_msBetweenKeyFrames;
bool endReached = interpolateAndDisplay(t);
if (!endReached)
glutTimerFunc(1000/g_animateFramesPerSecond,
animateTimerCallback,
ms + 1000/g_animateFramesPerSecond);
else {
	currentFrame = keyFrames.end();//Acc to my logic pointing to n
	currentFrame--; // Now pointing to n-1
}
}
static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {

  const double ARM_LEN = 0.45,
               ARM_THICK = 0.1,
               TORSO_LEN = 1.0,
               TORSO_THICK = 0.25,
               TORSO_WIDTH = 0.7,
			   LEG_LEN = 0.5,
			   LEG_THICK = 0.15,
			   HEAD_HEIGHT = 0.25,
			   HEAD_WIDTH = 0.2,
			   ELBOW_RADIUS = 0.07,
			   KNEE_RADIUS = 0.1;

  const int NUM_JOINTS = 10,
            NUM_SHAPES = 16;

  struct JointDesc {
    int parent;
    float x, y, z;
  };

  JointDesc jointDesc[NUM_JOINTS] = {
    {-1}, // torso
    {0,  TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper right shoulder
    {1,  ARM_LEN, 0, 0}, // lower right elbow
    {0,  TORSO_WIDTH/2 - LEG_THICK/2, -TORSO_LEN/2, 0}, // upper right hip
    {3,  0,-LEG_LEN, 0}, // lower right knee
	{0, 0,TORSO_LEN/2+HEAD_HEIGHT/2,0}, // neck
    {0,  -TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper left shoulder
    {6,  -ARM_LEN, 0, 0}, // lower left elbow
    {0,  -TORSO_WIDTH/2 + LEG_THICK/2, -TORSO_LEN/2, 0}, // upper left hip
    {8,  0,-LEG_LEN, 0}, // lower left knee
  };

  struct ShapeDesc {
    int parentJointId;
    float x, y, z, sx, sy, sz;
    shared_ptr<Geometry> geometry;
  };

  ShapeDesc shapeDesc[NUM_SHAPES] = {
    {0, 0, 0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
	{1, 0, 0, 0, ARM_THICK, ARM_THICK, ARM_THICK, g_arcBall}, //shoulder
    {1, ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper right arm
    {2, 0, 0, 0, ELBOW_RADIUS, ARM_THICK, ELBOW_RADIUS, g_arcBall}, //elbow
    {2, ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
	{3, 0,-LEG_LEN/2, 0,  LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // upper right leg
	{3, 0, -LEG_LEN, 0,KNEE_RADIUS,KNEE_RADIUS,KNEE_RADIUS, g_arcBall}, // right knee
    {4, 0,-LEG_LEN/2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg 
	{5, 0, HEAD_HEIGHT/2, 0, HEAD_WIDTH,HEAD_HEIGHT,HEAD_WIDTH,g_arcBall},//head
	{6, 0, 0, 0, ARM_THICK, ARM_THICK, ARM_THICK, g_arcBall}, //shoulder
	{6, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper left arm
    {7, 0, 0, 0, ELBOW_RADIUS, ARM_THICK, ELBOW_RADIUS, g_arcBall}, //elbow
	{7, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
	{8, 0,-LEG_LEN/2, 0,  LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // upper left leg
	{8, 0,-LEG_LEN, 0,KNEE_RADIUS,KNEE_RADIUS,KNEE_RADIUS, g_arcBall}, // left knee
    {9, 0,-LEG_LEN/2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower left leg 
  };

  shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (jointDesc[i].parent == -1)
      jointNodes[i] = base;
    else {
      jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
      jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
    }
  }
  for (int i = 0; i < NUM_SHAPES; ++i) {
    shared_ptr<MyShapeNode> shape(
      new MyShapeNode(shapeDesc[i].geometry,
                      color,
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}
// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

static unsigned getMode(){
	if(g_currentPickedRbtNode == NULL || g_currentPickedRbtNode == g_currViewRbt){
		if(g_currViewRbt == g_skyNode && g_activeCameraFrame == WORLD_SKY)
			return 1; //ARCBALL ON SKY
		else
			return 2; // EGO MOTION 
	}
	else
		return 3; //ARCBALL ON PICKED
}
static RigTForm getArcBallRBT(){
	switch(getMode()){
	case 1:
		return RigTForm();
	case 2:
		return getPathAccumRbt(g_world,g_currViewRbt);
	case 3:
		return getPathAccumRbt(g_world,g_currentPickedRbtNode);
	}
	return RigTForm();
}

static void drawStuff(const ShaderState& curSS, bool picking) {
  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // use the skyRbt as the eyeRbt
  const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currViewRbt);
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  if (!picking) {
	  Drawer drawer(invEyeRbt, curSS);
	  g_world->accept(drawer);

	  //draw arcBall
	  //============
	  if(g_showArcBall){
		  Matrix4 MVM,NMVM;
	  
		  //Get accumulated path from current view rbt to arcball
		  RigTForm arcballEye = inv(getPathAccumRbt(g_world,g_currViewRbt))*getArcBallRBT();

		  //if not zooming
		  if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))){
			  double depth = arcballEye.getTranslation()[2];
			  g_arcBallScale = getScreenToEyeScale(depth, g_frustFovY, g_windowHeight);
		  }
		  double alpha = g_arcBallScale* g_arcBallScreenRadius;
		  //g_objectRbt[2] = RigTForm(g_objectRbt[2].getTranslation(),g_objectRbt[2].getRotation()*alpha);
		  MVM = rigTFormToMatrix( arcballEye) * Matrix4::makeScale(Cvec3(1,1,1)*alpha);
		  NMVM = normalMatrix(MVM);
		  sendModelViewNormalMatrix(curSS, MVM, NMVM);
		  safe_glUniform3f(curSS.h_uColor, g_objectColors[2][0], g_objectColors[2][1], g_objectColors[2][2]);
		  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE);
		  g_arcBall->draw(curSS);
		  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	  }
  }
  else {
	  Picker picker(invEyeRbt, curSS);
	  g_world->accept(picker);
	  glFlush();
	  g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
	  if(!g_currentPickedRbtNode)
		  cout<<"Nothing Picked"<<endl;
	  if (g_currentPickedRbtNode == g_groundNode)
		g_currentPickedRbtNode = shared_ptr<SgRbtNode>();   // set to NULL
	  cout << "Picking Off" << endl;
	}
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  // using PICKING_SHADER as the shader
  glUseProgram(g_shaderStates[PICKING_SHADER]->program);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff(*g_shaderStates[PICKING_SHADER], true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  //glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}
static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff(*g_shaderStates[g_activeShader], false);

  glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  g_arcBallScreenRadius = 0.25*min(g_windowHeight,g_windowWidth);
  glutPostRedisplay();
}

static Cvec3 getDirection(const Cvec2& p){
	double n2 = norm2(p);
	if(n2 >= g_arcBallScreenRadius*g_arcBallScreenRadius)
	{
		return normalize(Cvec3(p,0));
	}
	else
	{
		return normalize(Cvec3(p,sqrt(g_arcBallScreenRadius*g_arcBallScreenRadius - n2)));
	}
}
static RigTForm getMRbt(const double dx, const double dy){
	RigTForm m;
	if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
		if(!g_isArcBallDisable && getMode()!=2){
		  const RigTForm eyeRbt = getPathAccumRbt(g_world,g_currViewRbt);
		  const RigTForm invEyeRbt = inv(eyeRbt);
		  Matrix4 projMatrix = makeProjectionMatrix();
		  Cvec3 arcWRTeye = Cvec3(invEyeRbt * Cvec4(getArcBallRBT().getTranslation(),1));
		  Cvec2 arcBall_center;
		  arcBall_center = getScreenSpaceCoord(arcWRTeye,projMatrix,g_frustNear,
					g_frustFovY,g_windowWidth,g_windowHeight);
		  const Cvec3 v0 = normalize(getDirection((Cvec2(g_mouseClickX, g_mouseClickY)-arcBall_center)));
		  const Cvec3 v1 = normalize(getDirection((Cvec2(g_mouseClickX + dx, g_mouseClickY + dy)-arcBall_center)));
		  m = RigTForm(Quat(0,v1[0],v1[1],v1[2]) * Quat(0, -v0[0],-v0[1],-v0[2]));
		  }
		else
			m= RigTForm(Quat::makeXRotation(-dy)*Quat::makeYRotation(dx));
	}
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
	if(!g_isArcBallDisable)
		m = m.setTranslation(Cvec3(dx, dy, 0) * g_arcBallScale);	
	else
		m = m.setTranslation(Cvec3(dx, dy, 0) * 0.01);
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
    m = m.setTranslation(Cvec3(0, 0, -dy) * 0.01);
  }
  switch (getMode()){
  case 1:
	  m = inv(m);
	  break;
  case 2:
	  if(g_mouseLClickButton && !g_mouseRClickButton)
		  m = inv(m);
	  break;
  }
  return m;
}

static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;
  RigTForm m,a; //m is transformation matrix, a is auxillary frame matrix.
  m = getMRbt(dx,dy);
  a = makeMixedFrame(getArcBallRBT(),getPathAccumRbt(g_world,g_currViewRbt));
  shared_ptr<SgRbtNode> target;
  switch (getMode()) {
	  case 1:
		  target = g_skyNode;
		  break;
	  case 2:
		  target = g_currViewRbt;
		  break;
	  case 3:
		  target = g_currentPickedRbtNode;
		  break;
	}
  a = inv(getPathAccumRbt(g_world, target, 1)) * a;
  target->setRbt(doMtoOwrtA(m, target->getRbt(), a));
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
  glutPostRedisplay();
}

static void reset()
{
	// =========================================================
	// TODO:
	// - reset g_skyRbt and g_objectRbt to their default values
	// - reset the views and manipulation mode to default
	// - reset sky camera mode to use the "world-sky" frame
	// =========================================================
	g_world->removeChild(g_skyNode);
	g_world->removeChild(g_groundNode);
	g_world->removeChild(g_robot1Node);
	g_world->removeChild(g_robot2Node);

	g_skyNode->setRbt(g_defSkyRbt);
	g_currentPickedRbtNode = g_currViewRbt = g_skyNode;
	g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
	g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

	constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
	constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot
	g_viewmode = SKY;
	g_showArcBall = true;
	g_activeCameraFrame = WORLD_SKY;
	g_world->addChild(g_skyNode);
	g_world->addChild(g_groundNode);
	g_world->addChild(g_robot1Node);
	g_world->addChild(g_robot2Node);
	cout << "reset objects and modes to defaults" << endl;
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;
  if(g_isPicking && g_mouseLClickButton){
	  g_isPicking = false;
	  pick();
  }
  glutPostRedisplay();
}
static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "o\t\tCycle object to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
  // ============================================================
  // TODO: add the following functionality for 
  //       keybaord inputs
  // - 'v': cycle through the 3 views
  // - 'o': cycle through the 3 objects being manipulated
  // - 'm': switch between "world-sky" frame and "sky-sky" frame
  // - 'r': reset the scene
  // ============================================================
  case 'v':
	  g_viewmode = Obj((g_viewmode + 1) % 3);
	  switch(g_viewmode){
		case SKY:
			cout << "Viewing from Sky" << endl;
			g_currViewRbt = g_skyNode;
			break;
		case CUBE1:
			cout << "Viewing from Robot 1"<< endl;
			g_currViewRbt = g_robot1Node;
			break;
		case CUBE2:
			cout << "Viewing from Robot 2"<< endl;
			g_currViewRbt = g_robot2Node;
			break;
	  }
	  break;
  case 'm':
	  //if(g_viewmode == SKY && g_currObj == SKY){ //Not needed according to the executable
	  g_activeCameraFrame = SkyMode((g_activeCameraFrame+1)%2);
	  if(g_activeCameraFrame == SKY_SKY)
		  cout<<"Ego Motion On"<<endl;
	  else
		  cout<<"Ego Motion Off"<<endl;
	  //}
	  break;
  case 'r':
	  reset();
	  break;
  case 'a':
	  g_isArcBallDisable = !g_isArcBallDisable;
	  break;
  case 'p':
	  cout << "Picking On" << endl;
	  g_isPicking = true;
	  break;
/*
	11/6 Key Frame Animation
*/
  case 'n':
	  {
		  Frame f;
		  dumpSgRbtNodes(g_world,rbtNodes);
		  for(size_t i =0; i<rbtNodes.size();i++)
		  {
			  f.push_back(rbtNodes[i]->getRbt());
		  }
		  rbtNodes.clear();
		  keyFrames.insert(currentFrame,f);
	  break;
	  }
  case 'u':
	  {
			Frame f;
			dumpSgRbtNodes(g_world,rbtNodes);
			f.clear();
			for(size_t i =0; i<rbtNodes.size();i++)
			{
				f.push_back(rbtNodes[i]->getRbt());
			}
			rbtNodes.clear();
			if(keyFrames.size() == 0)
			{
				keyFrames.insert(currentFrame, f);
			}
			else
			{
				--currentFrame;
				keyFrames.insert(currentFrame,f);
				list<Frame>::iterator temp = currentFrame;
				++temp;
				keyFrames.erase(currentFrame);
				currentFrame = temp;
			}
			break;
	  }
  case '>':
	  if(currentFrame != keyFrames.end())
	  {
		  //g_world.reset(new SgRootNode());
		  FrameToGraph f(*currentFrame);
		  g_world->accept(f);
		  ++currentFrame;
	  }
	  break;
  case '<':
	  {
	  list<Frame>::iterator temp = currentFrame;
	  if(keyFrames.size()>=2 && temp != keyFrames.begin() && --temp != keyFrames.begin())
	  {
		  --currentFrame; //actual frame.
		  FrameToGraph f(*(--currentFrame));//previous frame
		  ++currentFrame; //so that the iterator always points one frame after
		  g_world->accept(f);
	  }
	  break;
	  }
  case 'd':
	  if(keyFrames.size() == 1)
	  {
		  keyFrames.clear();
		  currentFrame = keyFrames.end();
	  }
	  else if(keyFrames.size() > 1)
	  {
		  list<Frame>::iterator temp = currentFrame; // next to the actual frame
		  --currentFrame; //the actual frame
		  keyFrames.erase(currentFrame);
		  currentFrame = temp;
		  if(temp == keyFrames.begin()) //current frame is at the beginning
		  {
			  ++currentFrame;//setting to the frame after the actual frame
		  }
		  else{ //deleted frame not at the beginning then set the current frame to the frame immediately before.
			  --temp; //setting temp to the actual frame
		  }
		  FrameToGraph f(*temp);//actual frame
		  g_world->accept(f);
	  }
	  break;
  case 'w':
	  {
		  ofstream ofs;
		  ofs.open("animation.txt");
		  cout<<"Start of:: write to file...." << endl;
		  for(size_t i =0; i<keyFrames.size();i++){
			  list<Frame> :: iterator baseFrame;
			  baseFrame  = keyFrames.begin();
			  advance(baseFrame, i);
			  for(size_t j =0; j<(*baseFrame).size();j++){
				  ofs<<(*baseFrame)[j]<<endl;
			  }
			  ofs<< END_FRAME<< endl;
		  }
		  ofs.close();
		  cout<<"End of:: write to file...." << endl;
	  }
	  break;
  case 'i':
	  {
		  cout<<"Start of:: read from file...." << endl;
		  keyFrames.clear();
		  ifstream ifs("animation.txt");
		  string text;
		  Frame f;
		  while(ifs>>text){
			  cout<< text;
			  if(text == END_FRAME){
				  keyFrames.push_back(f);
				  f.clear();
			  }
			  else
			  {
				  RigTForm r;
				  ifs >> r;
				  double d = stod(text);
				  r.setRotation(Quat(d,
					  r.getRotation()[1],
					  r.getRotation()[2],
					  r.getRotation()[3]));
				  f.push_back(r);
			  }
		  }
		  ifs.close();
		  cout<<"End of:: read from file...." << endl;
	  }
	  break;
  case 'y':
	  if(keyFrames.size() >= 4)
		  animateTimerCallback(0);
	  else
		  cerr<<"Warning:Not enough frames in the animation"<<endl;
	  break;
  case '+':
	  g_msBetweenKeyFrames -= 1000/g_animateFramesPerSecond;
	  break;
  case '-':
	  g_msBetweenKeyFrames += 1000/g_animateFramesPerSecond;
	  break;
  }
  // checks to show arcball interface.
  if((g_currentPickedRbtNode != g_currViewRbt				 
	  && !((g_viewmode == CUBE1 || g_viewmode == CUBE2) 
	  && g_currentPickedRbtNode == g_skyNode))
	  || g_currentPickedRbtNode == g_skyNode 
	  && g_viewmode == SKY && g_activeCameraFrame != SKY_SKY)
	  g_showArcBall = true;
  else
	  g_showArcBall = false;

  if (g_isArcBallDisable)
	  g_showArcBall = false;

  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 4");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  initGround();
  initCubes();
  initArcBall();
}
static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
  constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
  g_currViewRbt = g_currentPickedRbtNode = g_skyNode;
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initShaders();
    initGeometry();
	initScene();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
