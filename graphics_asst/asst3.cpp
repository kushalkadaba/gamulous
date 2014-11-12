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
#include <math.h>
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
static const bool g_Gl2Compatible = false;


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
static Obj g_viewmode = SKY;			// default viewmode.
static Obj g_currObj = SKY;			// default object.
static RigTForm g_defSkyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0),Quat());//holds the E matrix of default view.
bool g_egoMotion = false;	//to control ego motion when in sky view and camera is being manipulated.
bool g_showArcBall = true; //to control the visibility of the arc ball interface
double g_arcBallScreenRadius = 0.25*min(g_windowHeight,g_windowWidth);
double g_arcBallScale = 0.25*min(g_windowHeight,g_windowWidth);
bool g_isArcBallDisable = false;

struct ShaderState {
  GlProgram program;
  // Handles to uniform variables
  GLint h_uLight, h_uLight2;
  GLint h_uProjMatrix;
  GLint h_uModelViewMatrix;
  GLint h_uNormalMatrix;
  GLint h_uColor;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aNormal;

  ShaderState(const char* vsfn, const char* fsfn) {
    readAndCompileShader(program, vsfn, fsfn);

    const GLuint h = program; // short hand

    // Retrieve handles to uniform variables
    h_uLight = safe_glGetUniformLocation(h, "uLight");
    h_uLight2 = safe_glGetUniformLocation(h, "uLight2");
    h_uProjMatrix = safe_glGetUniformLocation(h, "uProjMatrix");
    h_uModelViewMatrix = safe_glGetUniformLocation(h, "uModelViewMatrix");
    h_uNormalMatrix = safe_glGetUniformLocation(h, "uNormalMatrix");
    h_uColor = safe_glGetUniformLocation(h, "uColor");

    // Retrieve handles to vertex attributes
    h_aPosition = safe_glGetAttribLocation(h, "aPosition");
    h_aNormal = safe_glGetAttribLocation(h, "aNormal");

    if (!g_Gl2Compatible)
      glBindFragDataLocation(h, 0, "fragColor");
    checkGlErrors();
  }

};

static const int g_numShaders = 2;
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"}
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


// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_arcBall;

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static RigTForm g_skyRbt = g_defSkyRbt;
// ============================================
// TODO: add a second cube's 
// 1. transformation
// 2. color
// ============================================
static RigTForm g_objectRbt[3] = {RigTForm(Cvec3(-0.65,0,0)),RigTForm(Cvec3(0.65,0,0)), RigTForm()};  // currently only 1 obj is defined
static Cvec3f g_objectColors[3] = {Cvec3f(1, 0, 0),Cvec3f(0, 0, 1),Cvec3f(0,1,0)};
static RigTForm* g_currObjRbt = &g_skyRbt; // set default to the sky camera
static RigTForm* g_currViewRbt = &g_skyRbt;// set default view to sky camera

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
// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// takes MVM and its normal matrix to the shaders
static void sendModelViewNormalMatrix(const ShaderState& curSS, const Matrix4& MVM, const Matrix4& NMVM) {
  GLfloat glmatrix[16];
  MVM.writeToColumnMajorMatrix(glmatrix); // send MVM
  safe_glUniformMatrix4fv(curSS.h_uModelViewMatrix, glmatrix);

  NMVM.writeToColumnMajorMatrix(glmatrix); // send NMVM
  safe_glUniformMatrix4fv(curSS.h_uNormalMatrix, glmatrix);
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
static RigTForm getArcBallRBT(){
	if(g_viewmode == SKY && g_currObj == SKY)
	{
		return RigTForm();
	}
	else
	{
		return *g_currObjRbt;
	}
}
static void drawStuff() {
  // short hand for current shader state
  const ShaderState& curSS = *g_shaderStates[g_activeShader];
  
  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // use the skyRbt as the eyeRbt
  const RigTForm eyeRbt = *g_currViewRbt;
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  // draw ground
  // ===========
  //
  const RigTForm groundRbt = RigTForm();  // identity
  Matrix4 MVM = rigTFormToMatrix(invEyeRbt * groundRbt);
  Matrix4 NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, 0.1, 0.95, 0.1); // set color
  g_ground->draw(curSS);

  // draw cubes
  // ==========
  MVM = rigTFormToMatrix(invEyeRbt * g_objectRbt[0]);
  NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_objectColors[0][0], g_objectColors[0][1], g_objectColors[0][2]);
  g_cube->draw(curSS);

  MVM = rigTFormToMatrix(invEyeRbt * g_objectRbt[1]);
  NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_objectColors[1][0], g_objectColors[1][1], g_objectColors[1][2]);
  g_cube->draw(curSS);

  //draw arcBall
  //============
  if(g_showArcBall){
	  RigTForm arcballEye = inv(*g_currViewRbt);
	  arcballEye = arcballEye * getArcBallRBT();
	  
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

static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff();

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

static Cvec3 getDirection(const Cvec2& p, double x, double y){
	double radius2 = g_arcBallScreenRadius * g_arcBallScreenRadius;
	Cvec2 scrPoint(x-p[0],y-p[1]);
	double n2 = norm2(scrPoint);
	if(n2 > radius2)
	{
		float ratio = g_arcBallScreenRadius / norm(scrPoint);
		scrPoint /= ratio;
		return Cvec3(scrPoint[0],scrPoint[1],0);
	}
	else
	{
		return normalize(Cvec3(scrPoint[0],scrPoint[1],sqrt(radius2 - n2)));
	}
}
static RigTForm getMRbt(int x, int y,double dx, double dy, bool isSkySky){
	RigTForm m;
	y = g_windowHeight - y -1;
	if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
		// are we showing the arcball?
	  if(g_showArcBall){
			const RigTForm eyeRbt = *g_currViewRbt;
			const RigTForm invEyeRbt = inv(eyeRbt);
			Matrix4 projMatrix = makeProjectionMatrix();
			Cvec3 arcWRTeye = Cvec3(invEyeRbt * (Cvec4(getArcBallRBT().getTranslation(),1)));
			Cvec2 arcBall_center;
			arcBall_center = getScreenSpaceCoord(arcWRTeye,projMatrix,g_frustNear,
					g_frustFovY,g_windowWidth,g_windowHeight);
			const Cvec3 v0 = normalize(getDirection(arcBall_center, g_mouseClickX, g_mouseClickY));
			const Cvec3 v1 = normalize(getDirection(arcBall_center, x,y));
			if(g_currObj != SKY)
				m = RigTForm(Quat(0,v1[0],v1[1],v1[2]) * Quat(0, -v0[0],-v0[1],-v0[2]));
			else
				m = RigTForm(Quat(0,v0[0],v0[1],v0[2]) * Quat(0, -v1[0],-v1[1],-v1[2]));
	  }
	  else if(g_viewmode == g_currObj){
		m= RigTForm(Quat::makeXRotation(dy)*Quat::makeYRotation(-dx));//m = Matrix4::makeXRotation(dy) * Matrix4::makeYRotation(-dx);
		}
	  else{
		m= RigTForm(Quat::makeXRotation(-dy)*Quat::makeYRotation(dx));
	  }
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
	if(g_showArcBall && !isSkySky){
		m = m.setTranslation(Cvec3(dx, dy, 0) * g_arcBallScale);	
	}
	else if(isSkySky && !g_egoMotion)
		m = m.setTranslation(Cvec3(-dx, -dy, 0) * g_arcBallScale);
	else 
		m = m.setTranslation(Cvec3(dx, dy, 0) * 0.01);
	
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
    m = m.setTranslation(Cvec3(0, 0, -dy) * 0.01);
  }
  return m;
}

static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;
  //Check if the sky camera is being controlled from the sky viewmode.
  bool isSkySky = false;
  if(g_viewmode == SKY && g_currObj == SKY){
	  isSkySky = true;
  }
  RigTForm m,a; //m is transformation matrix, a is auxillary frame matrix.
  m = getMRbt(x,y,dx,dy,isSkySky);
  if(isSkySky){
	  if (g_mouseClickDown  && g_egoMotion) {
		  *g_currObjRbt = doMtoOwrtA(m,*g_currObjRbt,*g_currObjRbt); // Modeling E = EM for ego motion
		  glutPostRedisplay(); // we always redraw if we changed the scene
	  }
	  else if(g_mouseClickDown  && !g_egoMotion){
		  a = makeMixedFrame(g_skyRbt,RigTForm());
		  *g_currObjRbt = doMtoOwrtA(m,*g_currObjRbt,a); // Modeling E = wE for rotation around center
		  glutPostRedisplay(); // we always redraw if we changed the scene
	  }
  }
  else{
	  if (g_mouseClickDown && !((g_viewmode == CUBE1 || g_viewmode == CUBE2) && g_currObj == SKY)) {
		  a = makeMixedFrame(*g_currViewRbt,*g_currObjRbt);
		  *g_currObjRbt = doMtoOwrtA(m,*g_currObjRbt,a); // Simply right-multiply is WRONG
		  glutPostRedisplay(); // we always redraw if we changed the scene
	  }
	  else if(g_mouseClickDown && ((g_viewmode == CUBE1 || g_viewmode == CUBE2) && g_currObj == SKY))
		  cerr << "Cannot manipulate sky camera from cube view" << endl;
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}

static void reset()
{
	// =========================================================
	// TODO:
	// - reset g_skyRbt and g_objectRbt to their default values
	// - reset the views and manipulation mode to default
	// - reset sky camera mode to use the "world-sky" frame
	// =========================================================
	g_skyRbt = g_defSkyRbt;
	g_objectRbt[0] = RigTForm(Cvec3(-0.65,0,0));
	g_objectRbt[1] = RigTForm(Cvec3(0.65,0,0));
	g_currObjRbt = &g_skyRbt;
	g_currViewRbt = &g_skyRbt;
	//if (g_viewmode == SKY) //Not needed according to the executable
		g_egoMotion = false;
	g_viewmode = g_currObj = SKY;
	g_showArcBall = true;
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
			g_currViewRbt = &g_skyRbt;
			break;
		case CUBE1:
			cout << "Viewing from Cube 1"<< endl;
			g_currViewRbt = &g_objectRbt[0];
			break;
		case CUBE2:
			cout << "Viewing from Cube 2"<< endl;
			g_currViewRbt = &g_objectRbt[1];
			break;
	  }
	  break;
  case 'o':
	  g_currObj = Obj((g_currObj + 1) % 3);
	  switch(g_currObj){
		case SKY:
			g_currObjRbt = &g_skyRbt;
			g_objectRbt[2] = RigTForm();
			//If in cube viewmode then should not be able to modify the sky camera
			if(g_viewmode == CUBE1 || g_viewmode == CUBE2)
				g_currObj = Obj((g_currObj + 1) % 3);
			else{
				cout << "Manipulating Sky Camera" << endl;
			break;
			}
		case CUBE1:
			cout << "Manipulating Cube 1"<< endl;
			g_currObjRbt = &g_objectRbt[0];
			g_objectRbt[2] = g_objectRbt[0]; 
			break;
		case CUBE2:
			cout << "Manipulating Cube 2"<< endl;
			g_currObjRbt = &g_objectRbt[1];
			g_objectRbt[2] = g_objectRbt[1];
			break;
	  }
	  break;
  case 'm':
	  //if(g_viewmode == SKY && g_currObj == SKY){ //Not needed according to the executable
	  g_egoMotion = !g_egoMotion;
	  if(g_egoMotion)
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
  }
  // checks to show arcball interface.
  if((g_currObj != g_viewmode				 
	  && !((g_viewmode == CUBE1 || g_viewmode == CUBE2) && g_currObj == SKY))
	  || g_currObj == SKY && g_viewmode == SKY && !g_egoMotion)
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
  glutCreateWindow("Assignment 2");                       // title the window

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

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
