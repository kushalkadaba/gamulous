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
#include "frametograph.h"
#include "geometry.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
// using namespace tr1; // for shared_ptr

void initScene();
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

// --------- Materials
// This should replace all the contents in the Shaders section, e.g., g_numShaders, g_shaderFiles, and so on
static shared_ptr<Material> g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat;

shared_ptr<Material> g_overridingMaterial;

// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;

// --------- Scene
// ===================================================================
// Declare the scene graph and pointers to suitable nodes in the scene
// graph
// ===================================================================

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode = g_skyNode; // used later when you do picking
static shared_ptr<SgRbtNode> g_light1Node, g_light2Node;
static const Cvec3 g_light1(2.0, 3.0, -14.0), g_light2(-2, 3.0, 5.0);  // define two lights positions in world space
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
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}


// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if t has reached the end of the animation
// sequence, or false otherwise. Be careful with the range of t (recall that the
// keyframes are numbered from -1 to n, while only keyframes between 0 and n-1 are
// shown
bool interpolateAndDisplay(float t) {
	if(floor(t) >= keyFrames.size()-3)
		return true;
	else
	{
		list<Frame> :: iterator baseFrame;
		baseFrame  = keyFrames.begin();
		advance(baseFrame, floor(t));
		vector<RigTForm> tform0 = (*baseFrame);
		vector<RigTForm> tform1 = (*++baseFrame);
		vector<RigTForm> tform2 = (*++baseFrame);
		vector<RigTForm> tform3 = (*++baseFrame);
		vector<RigTForm> interpolatedFrame;
		for(size_t i = 0; i< (*baseFrame).size(); i++){
			interpolatedFrame.push_back(interpolateCatmullRom(tform0[i],tform1[i],
				tform2[i],tform3[i],t-floor(t)));
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
static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {

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
	{1, 0, 0, 0, ARM_THICK, ARM_THICK, ARM_THICK, g_sphere}, //shoulder
    {1, ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper right arm
    {2, 0, 0, 0, ELBOW_RADIUS, ARM_THICK, ELBOW_RADIUS, g_sphere}, //elbow
    {2, ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
	{3, 0,-LEG_LEN/2, 0,  LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // upper right leg
	{3, 0, -LEG_LEN, 0,KNEE_RADIUS,KNEE_RADIUS,KNEE_RADIUS, g_sphere}, // right knee
    {4, 0,-LEG_LEN/2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg 
	{5, 0, HEAD_HEIGHT/2, 0, HEAD_WIDTH,HEAD_HEIGHT,HEAD_WIDTH,g_sphere},//head
	{6, 0, 0, 0, ARM_THICK, ARM_THICK, ARM_THICK, g_sphere}, //shoulder
	{6, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper left arm
    {7, 0, 0, 0, ELBOW_RADIUS, ARM_THICK, ELBOW_RADIUS, g_sphere}, //elbow
	{7, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
	{8, 0,-LEG_LEN/2, 0,  LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // upper left leg
	{8, 0,-LEG_LEN, 0,KNEE_RADIUS,KNEE_RADIUS,KNEE_RADIUS, g_sphere}, // left knee
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
                      material,
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}
// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
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

static void drawStuff(bool picking) {
	Uniforms uniforms;
  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

  // use the skyRbt as the eyeRbt
  const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currViewRbt);
  const RigTForm invEyeRbt = inv(eyeRbt);

  // get world space coordinates of the light
	Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
	Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();
	// transform to eye space, and set to uLight uniform
	uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(light1, 1)));
	uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(light2, 1)));
  if (!picking) {
	  Drawer drawer(invEyeRbt, uniforms);
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
		  sendModelViewNormalMatrix(uniforms, MVM, NMVM);
		  g_arcballMat->draw(*g_sphere, uniforms);
	  }
  }
  else {
	  Picker picker(invEyeRbt, uniforms);
	  // set overiding material to our picking material
	  g_overridingMaterial = g_pickingMat;
	  g_world->accept(picker);
	  // unset the overriding material
	  g_overridingMaterial.reset();
	  glFlush();
	  g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
	  if(!g_currentPickedRbtNode)
		  cout<<"Nothing Picked"<<endl;
	  if (g_currentPickedRbtNode == g_groundNode)
		g_currentPickedRbtNode = shared_ptr<SgRbtNode>();   // set to NULL
	  cout << "Picking Off" << endl;
	}
}
static void display() {
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff(false);

  glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

  checkGlErrors();
}
static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff( true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  //glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

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
	initScene();
	//g_skyNode->setRbt(g_defSkyRbt);
	//g_currentPickedRbtNode = g_currViewRbt = g_skyNode;
	//g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
	//g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

	//constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
	//constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot
	//g_viewmode = SKY;
	//g_showArcBall = true;
	//g_activeCameraFrame = WORLD_SKY;
	//g_world->addChild(g_skyNode);
	//g_world->addChild(g_groundNode);
	//g_world->addChild(g_robot1Node);
	//g_world->addChild(g_robot2Node);
	//cout << "reset objects and modes to defaults" << endl;
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
	/*
  case 'f':
    g_activeShader ^= 1;
    break;*/
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

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

  // normal mapping material
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<Texture>(new ImageTexture("Fieldstone.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<Texture>(new ImageTexture("FieldstoneNormal.ppm", false)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
  initGround();
  initCubes();
  initSphere();
}
static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

  //Lights
  g_light1Node.reset(new SgRbtNode(RigTForm(g_light1)));
  g_light2Node.reset(new SgRbtNode(RigTForm(g_light2)));
  g_light1Node->addChild(shared_ptr<MyShapeNode>(
	  new MyShapeNode(g_sphere, g_lightMat)));
  g_light2Node->addChild(shared_ptr<MyShapeNode>(
	  new MyShapeNode(g_sphere, g_lightMat)));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
  constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

  g_world->addChild(g_skyNode);
  g_world->addChild(g_light1Node);
  g_world->addChild(g_light2Node);
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
    initMaterials();
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
