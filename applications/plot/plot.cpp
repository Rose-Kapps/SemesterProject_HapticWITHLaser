//==============================================================================
//
//  TELEOPERATION
//
//==============================================================================
#include "chai3d.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include "Data_process.h"

#include <random>
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <algorithm>
#include <array> 
#include "tetgen.h"
#include "test_sample.h"
#include <filesystem>

#include <Eigen/Dense>
#include <Eigen/SVD>

using Vec3 = Eigen::Vector3d;
using MatX = Eigen::MatrixXd;

namespace fs = filesystem;

using namespace chai3d;
using namespace std;

Vec3 centroid;
Vec3 normal;
Vec3 singularValues;
Vec3 u;
Vec3 v;

Vec3 bufferA;
Vec3 bufferB;
atomic<Vec3*> active;

//------------------------------------------------------------------------------
// STATE MACHINE
// 
// STATE_IDLE: 
// The system is waiting for the operator to press the button. The robot is 
// holding its latest desired position.
// 
// STATE_TELEOPERATION:
// The operator is moving the haptic device and driving the robot to a new 
// desired position.
// 
// FIND_INTERFACE: (haptic state)
// The operator can move freely to find interface points
// 
// LINEAR_EXPLORATION: (haptic state)
// The operator is constrained to move along a line
// 
//------------------------------------------------------------------------------
enum cState
{
    STATE_IDLE,
    STATE_TELEOPERATION,
	FIND_INTERFACE,
    LINEAR_EXPLORATION
};

//------------------------------------------------------------------------------
// GENERAL VARIABLES
//------------------------------------------------------------------------------

bool scan_x = false;
bool scan_y = false;
bool scan_z = false;

// points defining the line for linear exploration
Vec3 PointA;
Vec3 PointB;

// raw and smoothed voltage values from THG sensor
int smoothed_ADCvalue;
double voltageLevel;
double voltageRaw;
double voltageSmoothed;

// a flag to indicate if the simulation currently running
bool simulationRunning = false;

// a flag to indicate if the simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the sensor rate
cFrequencyCounter freqCounterSensor;

// a frequency counter to measure the robot control rate
cFrequencyCounter freqCounterRobotDevice;

// a frequency counter to measure the haptic control rate
cFrequencyCounter freqCounterHapticDevice;

// sensor thread
cThread* sensorThread;

// robot thread
cThread* robotDeviceThread;

// haptic thread
cThread* hapticDeviceThread;

// device handler
cHapticDeviceHandler* handler;

// haptic device object
cGenericHapticDevicePtr hapticDevice = nullptr;

// robot device object
cGenericHapticDevicePtr robotDevice = nullptr;

// scale factor between haptic device and robot device
double scaleFactor = 0.2;

// haptic damping factor computed by the laser sensor
double hapticDampingFactor = 0.0;

// desired robot position
cVector3d robotPosDes(0, 0, 0);

// desired robot position simulation for scanning
cVector3d robotPosDesScan(0, 0, 0);

// current robot position
cVector3d robotPosCur(0, 0, 0);

// current robot velocity
cVector3d robotVelCur(0, 0, 0);

// rotation matrix of robot device
cMatrix3d robotRot;

// offset position between robot and graphic cube/cursor
cVector3d offset;

// mutex robot
cMutex mutexDevices;

cMutex mutexSharedPos;

// state machine
cState state;

// haptic state machine
cState haptic_state = FIND_INTERFACE;

// virtual robot position used for haptic rendering
cVector3d virtualRobotPos(0, 0, 0);

// list of interface points collected during FIND_INTERFACE state
vector<Vec3> list_of_interface_points;

//------------------------------------------------------------------------------
// CHAI3D GRAPHIC VARIABLES AND OBJECTS
//------------------------------------------------------------------------------

// a flag which indicates if the fullscreen mode is activated
bool fullscreen = false;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// camera intialisation position
cVector3d cameraPosition(0.03, 0.0, 0.02);

// a light source to illuminate the objects in the world
cDirectionalLight* light;

// a font for rendering text
cFontPtr font;

// a text label at the bottom of the window to display status
cLabel* labelMessage;

// a text label at the bottom of the window to display the value of the gradient
cLabel* labelGradient;

// a small sphere (cursor) representing the desired position of the robot
cShapeSphere* cursorRobotPosDes;

// a small sphere (cursor) representing the desired position of the robot while scanning
//cShapeSphere* cursorRobotPosDesScan;

// Shared camera state
double cameraAngle = 0.0;
double cameraDistance = 0.03;
double cameraHeight = 0.02;
cVector3d lookAt = cVector3d(0, 0, 0);

// a virtual voxel like object
cVoxelObject* object;

// 3D image data
cMultiImagePtr image;

// resolution of voxel model
int voxelModelResolution = 256;

// mutex for voxel object
cMutex mutexVoxel;

// 3D texture object
cTexture3dPtr texture;

// region of voxels being updated
cCollisionAABBBox volumeUpdate;

// flag that indicates that voxels have been updated
bool flagMarkVolumeForUpdate = false;

// a scope to monitor position values of haptic device
cScope* scope;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

ofstream* outAdress;

tetgenio in;  // input structure

tetgenio out; // output structure
tetgenio full_out;

cMesh* myMesh(nullptr);

deque <double> meshPositionsVector;

double area_threshold (0.0004);
double length_threshold (0.004);
//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback for mouse scrollwheel
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

// this function renders the scene
void updateGraphics(void);

// this function contains the laser sensor aquisition loop
void updateSensor(void);

// this function contains the robot device control loop
void updateRobotDevice(void);

// this function contains the haptic device control loop
void updateHapticDevice(void);

// this function set the color of a voxel at a given position in the volume
void setVoxel(cVector3d& a_pos, cColorb& a_color);

// this function closes the application
void close(void);

// This function locks restricts mouvement along x,y or z  axis
// by applying a spring force to the axis coordinate, that is set when
// user presses the button.
void axis_locking(double* forcex, double* forcey, double* forcez);

// This function does a z-axis sweep across the sample to the find the 
// maximum z-coordinate, which corresponds to the surface of the sample
void auto_scan(void);

//Ths fuction adds the current position to the sample vector according to the THG
//signal and represents the point-cloud
void add_value(void);

//Update camera parameteres according to mouse zoom and arrows
void updateCamera(void);

void renderTetGenMesh(tetgenio* out, cWorld* world);

cMesh* createSurfaceMeshFromTetgen(tetgenio& out);
cMesh* createSurfaceMeshFromTetgen_noFilter(tetgenio& out);


//==============================================================================
/*
    DEMO:   plot.cpp

    This application connects a haptic device to a robot device in a
    teleoperation mode.

    A small graphical scope plots a signal value acquired by an DAC board
    (laser sensor)

    A 3D volume image is used to render laser sensor data.
*/
//==============================================================================


//Added variables and functions (to be sorted)
ofstream outFile;
bool reverseMode=false;
bool lock_y = false;
bool lock_x = false;
bool lock_z = false;
cVector3d posX, posY, posZ;
double maxSignal;
cVector3d maxPosition;
bool scan_finished(false);
void reset_sample();
cVector3d gradient(0, 0, 0);
cVector3d signalDiff(0, 0, 0);

// Added classes and functions ------------------------------------------


class LowPassFilter {
public:
    LowPassFilter(double alpha = 0.1) : alpha(alpha), initialized(false), y(0.0) {}

    double apply(double x) {
        if (!initialized) {
            y = x;
            initialized = true;
        }
        else {
            y = alpha * x + (1.0 - alpha) * y;
        }
        return y;
    }

private:
    double alpha;       // 0 < alpha < 1
    bool initialized;
    double y;
};

// Fit a plane to points using SVD. Returns centroid, unit normal, singular values.
void fitPlaneSVD(const std::vector<Vec3>& points, Vec3& centroid, Vec3& normal, Vec3& singularValues) {
    if (points.size() < 3) throw std::runtime_error("at least 3 points needed");
    centroid.setZero();
    for (const auto& p : points) centroid += p;
    centroid /= double(points.size());
    MatX X(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        Vec3 v = points[i] - centroid;
        X.row(i) = v.transpose();
    }

    // SVD
    Eigen::JacobiSVD<MatX> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    singularValues = svd.singularValues();
    // V columns are principal components. The normal = last column of V (smallest singular value)
    Eigen::Matrix3d V = svd.matrixV();
    normal = V.col(2); // column associated to smallest variance
    normal.normalize();
}

// Signed distance from point to plane
double signedDistanceToPlane(const Vec3& point, const Vec3& centroid, const Vec3& normal) {
    return normal.dot(point - centroid);
}

// Orthogonal projection of a point onto the plane
Vec3 projectPointToPlane(const Vec3& point, const Vec3& centroid, const Vec3& normal) {
    double d = signedDistanceToPlane(point, centroid, normal);
    return point - d * normal;
}

// Build two orthonormal basis vectors u, v lying in the plane
void planeBasis(const Vec3& normal, Vec3& u, Vec3& v) {
    Vec3 n = normal.normalized();
    Vec3 tmp;
    if (std::abs(n.x()) < 0.9) tmp = Vec3(1, 0, 0);
    else tmp = Vec3(0, 1, 0);
    u = n.cross(tmp).normalized();
    v = n.cross(u).normalized();
}

// Orthogonal projection of a point onto the line AB
Vec3 projectPointToLine(const Vec3& P, const Vec3& A, const Vec3& B) {
    Vec3 AB = B - A;
    Vec3 AP = P - A;
	double t = AP.dot(AB) / AB.dot(AB);  // Scalar projection
    return A + t * AB;
}

// Orthogonal projection of a vector onto the line AB
Vec3 projectVectorToLine(const Vec3& vector, const Vec3& A, const Vec3& B) {
    Vec3 AB = B - A;
	Vec3 projected_vector = (vector.dot(AB) / AB.dot(AB)) * AB;  // Scalar projection
    return projected_vector;
}

// Signed distance from point to line AB
double distanceToLine(const Vec3& P, const Vec3& A, const Vec3& B) {
    Vec3 proj = projectPointToLine(P, A, B);
    return (P - proj).norm();
}

// Vector between point and its projection onto line AB
Vec3 vectorToLine(const Vec3& P, const Vec3& A, const Vec3& B) {
    Vec3 proj = projectPointToLine(P, A, B);
    return (P - proj);
}

// Normalized perpendicular direction from point to line AB
Vec3 perpendicularDirectionToLine(const Vec3& P, const Vec3& A, const Vec3& B) {
    Vec3 proj = projectPointToLine(P, A, B);
    Vec3 d = P - proj;
    double n = d.norm();
    if (n < 1e-9) return Vec3::Zero();
    return d / n;
}

double computeT(const Vec3& P, const Vec3& A, const Vec3& B)
{
    Vec3 AB = B - A;
    Vec3 AP = P - A;

    double denom = AB.dot(AB);
    if (denom < 1e-12) return 0.0; 

    double t = AP.dot(AB) / denom;

    return t;
}


// Verify if newPoint is at least 'threshold' distance away from all points in 'points'
bool isFarEnough(const vector<Vec3>& points, const Vec3& newPoint, double threshold) {
    for (const auto& p : points) {
        if ((p - newPoint).norm() < threshold) {
            return false; // Too close
        }
    }
    return true; // Far enough
}


// -------------------------------------------------------------------------    

int main(int argc, char* argv[])
{

    

    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: plot" << endl;
    cout << "Copyright 2003-2023" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[0] - scale factor 0.005x" << endl;
    cout << "[1] - scale factor 0.02x" << endl;
    cout << "[2] - scale factor 0.05x" << endl;
    cout << "[3] - scale factor 0.20x" << endl;
    cout << "[4] - scale factor 0.50x" << endl;
    cout << "[c] - reset offset" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[x] - Lock translation in x direction" << endl;
    cout << "[y] - Lock translation in y direction" << endl;
    cout << "[z] - Lock translation in z direction" << endl;
    cout << "[g] - X-axis Scan" << endl;
    cout << "[h] - Y-axis scan" << endl;
    cout << "[j] - Z-axis scan" << endl;
    cout << "[u] - Render sample" << endl;
    cout << "[r] - Reset sample render" << endl;
    cout << "[S] - Set new maximum obtained voltage [V]. Default set to 5 V. Can also be found doing a z scan (press[j])" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // ---------------------- Buffer ------------------- //
    active.store(&bufferA);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.9 * mode->height;
    int h = 0.9 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // enable double buffering
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    // set the desired number of samples to use for multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(1);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlueLight();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);
  
    // position and orient the camera
    camera->set(cameraPosition,    // camera position (eye)
        cVector3d(0.0, 0.0, 0.0),    // look at position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

// set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.0001, 1.0);

    // create a directional light source
    light = new cDirectionalLight(world);
    light->setLocalPos(0.00, 0.00, 0.0);
    // Set light direction (light coming from above, like sunlight)
    //light->setDir(cVector3d(0.0, 0.0, -1.0));  // Points in negative Z direction

    // insert light source inside world
    world->addChild(light);
    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    //light->setDir(-1.0, 0.0, 0.0);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONT_CALIBRI_20();

    // create a label to display the haptic and graphic rate of the simulation
    labelMessage = new cLabel(font);
    camera->m_frontLayer->addChild(labelMessage);

    // creer label pour afficher valeur gradient 
    labelGradient = new cLabel(font);
    camera->m_frontLayer->addChild(labelGradient);




    ////////////////////////////////////////////////////////////////////////////
    // In the following lines we set up several widgets to display position
    // and velocity data coming from the haptic device. For each widget we
    // define a range of values to expect from the haptic device. In this
    // example the units are meters (as we are tracking a position signal!) and 
    // have a set a default range between -0.1 to 0.1 meters. If you are using 
    // devices with a small or larger workspace, you may want to adjust these 
    // values accordingly. The other settings will modify the visual appearance
    // of the widgets. Have fun playing with these values!
    ////////////////////////////////////////////////////////////////////////////

    // create a scope to plot haptic device position data
    scope = new cScope();
    camera->m_frontLayer->addChild(scope);
    scope->setLocalPos(100, 60);
    scope->setRange(0.0, 2.0);
    scope->setSignalEnabled(true, false, false, false);
    scope->setTransparencyLevel(0.7);


    //--------------------------------------------------------------------------
    // CREATE ROBOT SPHERE
    //--------------------------------------------------------------------------

    // create small sphere

    cursorRobotPosDes = new cShapeSphere(0.0003);

    // add cursor to world
    world->addChild(cursorRobotPosDes);

    // set cursor color
    cursorRobotPosDes->m_material->setBlueCornflower();

    //--------------------------------------------------------------------------
    // CREATE VOXEL OBJECT
    //--------------------------------------------------------------------------

    // create a volumetric model
    object = new cVoxelObject();

    // add object to world
    world->addChild(object);

    // set object position in scene
    object->setLocalPos(0.0, 0.0, 0.0);

    // set the dimensions by assigning the position of the min and max corners
    object->m_minCorner.set(-0.016, -0.016, -0.016);
    object->m_maxCorner.set(0.016, 0.016, 0.016);

    // set the texture coordinate at each corner.
    object->m_minTextureCoord.set(0.0, 0.0, 0.0);
    object->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set material color
    //object->m_material->setOrangeCoral();
    // show/hide boundary box
    object->setShowBoundaryBox(true);


    // ------------------------------------------------------------------------
    // CREAT WORLD FLOOR 
    //--------------------------------------------------------------------------

    // Create a new mesh object for the floor
    cMesh* floor = new cMesh();
    world->addChild(floor);

    // Size of the floor (length of one side)
    double floorSize = 0.05;

    // Height at which the floor is placed
    double floorHeight = -0.017;  // slightly below origin

    // Create four vertices of the floor
    int v0 = floor->newVertex(-floorSize, -floorSize , floorHeight );
    int v1 = floor->newVertex(-floorSize, floorSize, floorHeight);
    int v2 = floor->newVertex(floorSize, floorSize, floorHeight );
    int v3 = floor->newVertex(floorSize, -floorSize, floorHeight );

    // Create two triangles (a quad made of two triangles)
    floor->newTriangle(v0, v1, v2);
    floor->newTriangle(v0, v2, v3);

    // Set material properties (light gray)
    floor->m_material->setGrayDark();

    // Compute normals for proper lighting
    floor->computeAllNormals();

    // Add the floor to the world (as previously done)
    world->addChild(floor);

    //--------------------------------------------------------------------------
    // CREATE MESH
    //--------------------------------------------------------------------------
    

    //--------------------------------------------------------------------------
    // CREATE VOXEL DATA
    //--------------------------------------------------------------------------

    // create multi image data structure
    image = cMultiImage::create();

    // allocate 3D image data
    image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);

    // create texture
    texture = cTexture3d::create();

    // assign texture to voxel object
    object->setTexture(texture);

    // assign volumetric image to texture
    texture->setImage(image);

    // set quality of graphic rendering
    object->setQuality(0.5);

    // set default rendering mode
    object->setRenderingModeIsosurfaceMaterial();
    object->setRenderingModeIsosurfaceColors();


    //--------------------------------------------------------------------------
    // CREATE ROTATION MATRIX FOR ROBOT
    //--------------------------------------------------------------------------

    // set identity matrix
    robotRot.identity();

    //// rotate the robot device base to the desired angle
    //robotRot.rotateAboutGlobalAxisDeg(0, 0, 1, 180);

    //--------------------------------------------------------------------------
    // DETECT ROBOT AND HAPTIC DEVICES
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of devices
    unsigned int numDevices = handler->getNumDevices();

    // get pointer to devices from handler (if available)
    cGenericHapticDevicePtr device0 = cGenericHapticDevice::create();
    cGenericHapticDevicePtr device1 = cGenericHapticDevice::create();

    cHapticDeviceInfo infoDevice0;
    cHapticDeviceInfo infoDevice1;

    if (numDevices >= 1)
    {
        handler->getDevice(device0, 0);
        infoDevice0 = device0->getSpecifications();
    }

    if (numDevices >= 2)
    {
        handler->getDevice(device1, 1);
        infoDevice1 = device1->getSpecifications();
    }

    // assign devices based on their type (omega.3 or sigma.7)
    if (infoDevice0.m_model == cHapticDeviceModel::C_HAPTIC_DEVICE_SIGMA_7)
    {
        robotDevice = device0;
        hapticDevice = device1;
    }
    else if (infoDevice1.m_model == cHapticDeviceModel::C_HAPTIC_DEVICE_SIGMA_7)
    {
        robotDevice = device1;
        hapticDevice = device0;
    }
    else
    {
        robotDevice = device1;
        hapticDevice = device0;
    }
    // open connection to robot device
    robotDevice->open();

    // update position data
    cVector3d posTemp;
    robotDevice->getPosition(posTemp);

    // update current robot position by taking into account robot rotation matrix
    robotPosCur = robotRot * posTemp;

    // set desired robot position to current robot position
    robotPosDes = robotPosCur;

    // open connection to haptic device
    hapticDevice->open();

    // set the cursor to the origin
    cVector3d pos(0, 0, 0);
    robotDevice->getPosition(pos);
    offset = robotRot * pos;


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // intialize state machine
    state = STATE_IDLE;

    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // create a thread which starts the main laser sensor loop
    sensorThread = new cThread();
    sensorThread->start(updateSensor, CTHREAD_PRIORITY_GRAPHICS);
    

    // create a thread which starts the robot device control loop
    robotDeviceThread = new cThread();
    robotDeviceThread->start(updateRobotDevice, CTHREAD_PRIORITY_HAPTICS);
    

    // create a thread which starts the haptic device control loop
    hapticDeviceThread = new cThread();
    hapticDeviceThread->start(updateHapticDevice, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);
    

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    //Mouse callback to zoom in or out 
    glfwSetScrollCallback(window, scrollCallback);
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;

    // update position of scope
    scope->setSize(width - 200, 70);
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    //Move robot to max position (currently does not work correctly)
    if (a_key == GLFW_KEY_T)
    {
        //robotPosDes = maxPosition;
        //cout << "Moved to :" << maxPosition.x() << " " << maxPosition.y() << " " << maxPosition.z() << endl;
        //cout << "Moved to :" << robotPosCur.x() << " " << robotPosCur.y() << " " << robotPosCur.z() << endl;
        //cout << "Moved to :" << robotPosDes.x() << " " << robotPosDes.y() << " " << robotPosDes.z() << endl;

    }
    //Resets all voxels if R key is pressed
    if (a_key == GLFW_KEY_R && a_action == GLFW_PRESS)
    {
        reset_sample();
    }
    //Lock X translation
    if (a_key == GLFW_KEY_X && (a_action != GLFW_PRESS))
    {   
        hapticDevice->getPosition(posX);
        lock_x = !lock_x;
    }
    
    //Lock Y translation
        if (a_key == GLFW_KEY_Y && (a_action != GLFW_PRESS))
    {   
        hapticDevice->getPosition(posY);
        lock_y = !lock_y;
    }
    //Lock Z translation
    if (a_key == GLFW_KEY_Z && (a_action != GLFW_PRESS))
    {
        hapticDevice->getPosition(posZ);
        lock_z = !lock_z;
    }


    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    //Scans sample on one-axis and writes output signal to file
    //Warning: old file is written over each time this mode is activated
    if ((a_key == GLFW_KEY_G || a_key == GLFW_KEY_H || a_key == GLFW_KEY_J) && (a_action == GLFW_PRESS))
    {
        if (((!scan_x) && a_key == GLFW_KEY_G) || ((!scan_y) && a_key == GLFW_KEY_H) || (!scan_z && a_key == GLFW_KEY_J)) {
            outFile.open("output.csv", std::ios::trunc);
            if (outFile.is_open()) {
                cVector3d current_position(0,0,0);
                switch (a_key) {
                    case GLFW_KEY_G:
                        outFile << "Position in x [m],Voltage [V]" << std::endl;
                        break;
                    case GLFW_KEY_H:
                        outFile << "Position in y [m],Voltage [V]" << std::endl;
                        break;
                    case GLFW_KEY_J:
                        outFile << "Position in z [m],Voltage [V]" << std::endl;
                        robotDevice->getPosition(current_position);
                        
                        // This part is supposed to find the absolute maximum position when doing a z scan but
                        // does not work correctly
                        //current_position = robotRot * current_position;
                        //updateMax(current_position , 0, maxPosition, true);
                        //cout << "Postion update to :" << maxPosition.x() << " " << maxPosition.y() << " " << maxPosition.z() << endl;
                        break;
                    default:
                        break;
                }
                cout << "Data appended to output.csv successfully." << endl;
            }
            else cout << "Error opening file." << std::endl;
        }
        else
        {
            outFile.close();
            cout << "Data written and doc closed" << std::endl;
        }
        switch (a_key){
            case GLFW_KEY_G:
                scan_x = !scan_x;
                break;
            case GLFW_KEY_H:
                scan_y = !scan_y;
                break;
            case GLFW_KEY_J:
                scan_z = !scan_z;
                break;
            default:
                break;
        } 
    }
    
    static bool show(false);
    //Toggles the pixel coloring update
    if ((a_key == GLFW_KEY_U) && (a_action == GLFW_PRESS))
    {
        if (scan_finished) scan_finished = false;
        else scan_finished = true;
        
        cout << scan_finished << endl;
        unique_ptr<REAL[]> pointlist;
        in.numberofpoints = 5000;

        // Default mesh shape is a rectangular cuboid with different sized holes
        //  for which the array is stored in the test_sample.h file
        in.pointlist = test_5000;

        //Once the sensor sampples enough points this mesh  is computer 
        //instead of the test model
        if (meshPositionsVector.size() > 12) {
            in.numberofpoints = meshPositionsVector.size();
            pointlist = make_unique<double[]>(meshPositionsVector.size());
            // Copy values from deque to the pointlist
            size_t index = 0;
            for (double value : meshPositionsVector) {
                pointlist[index++] = value;
            }
            in.pointlist = pointlist.get();
        }
        tetrahedralize((char*)"zQ", &in, &out);
        
        if (show == false) {
            myMesh = createSurfaceMeshFromTetgen(out);
            world->addChild(myMesh);
            show = true;
        }
        else {
            world->removeChild(myMesh);
            show = false;
        }
    }

    // option - exit
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    //Modify scale factor accord to which button is pressed
    if (state == STATE_IDLE) {
        switch (a_key) {
        case GLFW_KEY_0:
            scaleFactor = 0.001;
            break;
        case GLFW_KEY_1:
            scaleFactor = 0.02;
            break;
        case GLFW_KEY_2:
            scaleFactor = 0.05;
            break;
        case GLFW_KEY_3:
            scaleFactor = 0.2;
            break;
        case GLFW_KEY_4:
            scaleFactor = 0.5;
            break;
        default:
            break;
        }
    }


    // option - reset offset
    if (a_key == GLFW_KEY_C)
    {
        offset = robotPosCur;
    }


    // option - reset offset
    if (a_key == GLFW_KEY_W)
    {
        area_threshold += 0.00001;
        cout << area_threshold << endl;
        if (myMesh) {
            world->removeChild(myMesh);
            delete myMesh;
        }
        myMesh = createSurfaceMeshFromTetgen(out);
        world->addChild(myMesh);

    }
    // option - reset offset
    if (a_key == GLFW_KEY_S)
    {
        area_threshold -=0.00001;
        cout << area_threshold << endl;
        if (myMesh) {
            world->removeChild(myMesh);
            delete myMesh;
        }
        myMesh = createSurfaceMeshFromTetgen(out);
        world->addChild(myMesh);

    }
    // option - reset offset
    if (a_key == GLFW_KEY_E)
    {
        length_threshold +=0.0001;
        cout << length_threshold << endl;
        if (myMesh) {
            world->removeChild(myMesh);
            delete myMesh;
        }
        myMesh = createSurfaceMeshFromTetgen(out);
        world->addChild(myMesh);
    }
    // option - reset offset
    if (a_key == GLFW_KEY_D)
    {
        length_threshold -=0.0001;
        cout << length_threshold << endl;
        if (myMesh) {
            world->removeChild(myMesh);
            delete myMesh;
        }
        myMesh = createSurfaceMeshFromTetgen(out);
        world->addChild(myMesh);

    }

    // option - toggle fullscreen
    if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(1);

            // set the desired number of samples to use for multisampling
            glfwWindowHint(GLFW_SAMPLES, 4);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(1);

            // set the desired number of samples to use for multisampling
            glfwWindowHint(GLFW_SAMPLES, 4);
        }
    }


    if (a_action == GLFW_PRESS || a_action == GLFW_REPEAT)
    {
        if (a_key == GLFW_KEY_LEFT)       cameraAngle -= 0.05;
        else if (a_key == GLFW_KEY_RIGHT) cameraAngle += 0.05;
        else if (a_key == GLFW_KEY_UP)    cameraHeight += 0.001;
        else if (a_key == GLFW_KEY_DOWN)  cameraHeight -= 0.001;

        updateCamera();
    }
    //Resets all voxels if R key is pressed
    if (a_key == GLFW_KEY_R && a_action == GLFW_PRESS)
    {
        reset_sample();
        show = false;
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(500); }

    // delete resources
    delete handler;
    delete sensorThread;
    delete hapticDeviceThread;
    delete robotDeviceThread;
    delete world;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
  
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update status message
    labelMessage->setText("Graphics: " + cStr(freqCounterGraphics.getFrequency(), 0) + " Hz  /  " +
        "Sensor: " + cStr(freqCounterSensor.getFrequency(), 0) + " Hz  /  " +
        "Haptic Device: " + cStr(freqCounterHapticDevice.getFrequency(), 0) + " Hz  /  " +
        "Robot Device: " + cStr(freqCounterRobotDevice.getFrequency(), 0) + " Hz  /  " +
        "Scale Factor: " + cStr(scaleFactor, 4) + "x");

    // update position of label
    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getWidth())), 15);

    // update labelgradient et mettre à jour la pos du label, mettre au dessus de labelMessage
    labelGradient->setLocalPos((int)(0.5 * (width - labelGradient->getWidth())), 35);//centre le message au bas de l'écran

    labelGradient->setText("X scan: " + cStr(scan_x) + "  /  " +
        "Y Scan : " + cStr(scan_y) + "  /  " +
        "Z Scan : " + cStr(scan_z) + "/ Voltage: " + cStr(voltageLevel,5) + "  /  " +
        "x: " + cStr(signalDiff.x()) + ", y: " + cStr(signalDiff.y()) + ", z: " + cStr(signalDiff.z()));

    /////////////////////////////////////////////////////////////////////
    // VOLUME UPDATE
    /////////////////////////////////////////////////////////////////////

    // update region of voxels to be updated
    if (flagMarkVolumeForUpdate)
    {
        mutexVoxel.acquire();
        cVector3d min = volumeUpdate.m_min;
        cVector3d max = volumeUpdate.m_max;
        volumeUpdate.setEmpty();
        mutexVoxel.release();
        texture->markForPartialUpdate(min, max);
        flagMarkVolumeForUpdate = false;
    }

    // update position of cursor
    mutexDevices.acquire();
    cursorRobotPosDes->setLocalPos(robotPosDes - offset);
    mutexDevices.release();



    /////////////////////////////////////////////////////////////// //////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void setVoxel(cVector3d& a_pos, cColorb& a_color)
{
    // compute size of volume
    double sizeX = object->m_maxCorner.x() - object->m_minCorner.x();
    double sizeY = object->m_maxCorner.y() - object->m_minCorner.y();
    double sizeZ = object->m_maxCorner.z() - object->m_minCorner.z();

    // compute number of voxels for each side
    int numVoxelX = image->getWidth();
    int numVoxelY = image->getHeight();
    int numVoxelZ = image->getImageCount();

    // compute voxel indices
    int voxelIndexX = (int)((a_pos.x() - object->m_minCorner.x()) * numVoxelX / sizeX);
    int voxelIndexY = (int)((a_pos.y() - object->m_minCorner.y()) * numVoxelY / sizeY);
    int voxelIndexZ = (int)((a_pos.z() - object->m_minCorner.z()) * numVoxelZ / sizeZ);

    // clamp index values within image volume
    voxelIndexX = cClamp(voxelIndexX, 0, numVoxelX - 1);
    voxelIndexY = cClamp(voxelIndexY, 0, numVoxelY - 1);
    voxelIndexZ = cClamp(voxelIndexZ, 0, numVoxelZ - 1);

    // update voxel color
    object->m_texture->m_image->setVoxelColor(voxelIndexX, voxelIndexY, voxelIndexZ, a_color);

    // mark voxel for update
    mutexVoxel.acquire();
    volumeUpdate.enclose(cVector3d(voxelIndexX, voxelIndexY, voxelIndexZ));
    mutexVoxel.release();
    flagMarkVolumeForUpdate = true;

}



//------------------------------------------------------------------------------

void updateSensor(void)
{   
   
    //--------------------------------------------------------------------------
    // INIT I/O BOARD
    //--------------------------------------------------------------------------

    int Row, Col;
    int BoardNum = 0;
    int ULStat = 0;
    int LowChan = 0;
    int HighChan = 0;
    int Range = UNI1PT25VOLTS;
    short Status = 0;
    long CurCount;
    long CurIndex;
    int Count = 100;
    long Rate = 10000;
    unsigned Options;
    HANDLE MemHandle = 0;
    WORD* ADData;
    DWORD* ADData32;
    float    RevLevel = (float)CURRENTREVNUM;
    BOOL HighResAD = FALSE;
    int  ADRes;

    // Declare UL Revision Level
    ULStat = cbDeclareRevision(&RevLevel);

    // Initiate error handling
    //     Parameters:
    //         PRINTALL :all warnings and errors encountered will be printed
    //         DONTSTOP :program will continue even if error occurs.
    //                  Note that STOPALL and STOPFATAL are only effective in
    //                  Windows applications, not Console applications.
    cbErrHandling(PRINTALL, DONTSTOP);

    // Get the resolution of A/D
    cbGetConfig(BOARDINFO, BoardNum, 0, BIADRES, &ADRes);

    // If ADRes is equal to zero, exit as card has not acquisition card has not been detected
    if (ADRes == 0)
    {
        return;
    }

    // check If the resolution of A/D is higher than 16 bit.
    //    If it is, then the A/D is high resolution.
    if (ADRes > 16)
        HighResAD = TRUE;

    //  set aside memory to hold data
    if (HighResAD)
    {
        MemHandle = cbWinBufAlloc32(Count);
        ADData32 = (DWORD*)MemHandle;
    }
    else
    {
        MemHandle = cbWinBufAlloc(Count);
        ADData = (WORD*)MemHandle;
    }

    if (!MemHandle)
    {
        printf("\nout of memory\n");
        exit(1);
    }

    WORD DataValue;
    ULStat = cbFromEngUnits(BoardNum, Range, 0.0, &DataValue);
    ULStat = cbAOut(BoardNum, 0, Range, DataValue);

    // Collect the values with cbAInScan() in BACKGROUND mode
    //     Parameters:
    //          BoardNum    :the number used by CB.CFG to describe this board
    //          LowChan     :low channel of the scan
    //          HighChan    :high channel of the scan
    //          Count       :the total number of A/D samples to collect
    //          Rate        :sample rate in samples per second
    //          Gain        :the gain for the board
    //          ADData[]    :the array for the collected data values
    //          Options     :data collection options
    Options = NOCONVERTDATA + FOREGROUND; // BACKGROUND;

     //----------------------------------------------------------------------
    // CSV FILE SETUP
    //----------------------------------------------------------------------

    int StepSmoother_tolerance = 1;
    int StepSmoother_persistance = 3;


    int windowSize = 10;
    double sigma = 1;

    double alpha = 0.003;

    
    //----------------------------------------------------------------------
    // FILTER INITIALIZATION
    //----------------------------------------------------------------------

    static StepSmoother adcSmoother(StepSmoother_tolerance, StepSmoother_persistance);

    static GaussianFilter filter(windowSize, sigma);

    static LowPassFilter THGfilter(alpha);

    //----------------------------------------------------------------------
    // MAIN LOOP
    //----------------------------------------------------------------------
    while (simulationRunning)
    {
        // start data scanning operation
        ULStat = cbAInScan(BoardNum, LowChan, HighChan, Count, &Rate, Range, MemHandle, Options);

        // wait for data aquisition to complete
        Status = RUNNING;
        while (Status == RUNNING)
        {
            // check the status of the current background operation
            // parameters:
            //     BoardNum  :the number used by CB.CFG to describe this board
            //     Status    :current status of the operation (IDLE or RUNNING)
            //     CurCount  :current number of samples collected
            //     CurIndex  :index to the last data value transferred
            //     FunctionType: A/D operation (AIFUNCTIOM)
            ULStat = cbGetStatus(BoardNum, &Status, &CurCount, &CurIndex, AIFUNCTION);
        }

        // process data
        for (int i = 0; i < Count; i++)
        {
            // get next data value
            WORD dataValue = 0;

            if (HighResAD)
            {
                dataValue = ADData32[i];
            }
            else
            {
                dataValue = ADData[i];
            }

            // --- Compute smooth ADC Value (filtering of the steps) ---

            //if (first_loop == true) {
            //    first_loop = false;
            //    cout << "first adc value: " << dataValue << endl;
            //}

            ///*cout << dataValue << endl;*/

            //if ()

            smoothed_ADCvalue = adcSmoother.apply(dataValue);
            
            
            // --- Compute raw voltage ---
            voltageRaw = cClamp(5.0 * (((double)dataValue - 2048.0) / 964.0), 0.0, 5.0);

            voltageSmoothed = cClamp(5.0 * (((double)smoothed_ADCvalue - 2048.0) / 964.0), 0.0, 5.0);

            // --- Apply Gaussian filter ---
            /*voltageLevel = filter.applyFilter(voltageRaw);*/

           /* voltageLevel = THGfilter.apply(voltageRaw);*/

            voltageLevel = voltageRaw;
            

            // compute a haptic damping factor based on laser signal
            double dampingGain = 0.4;
            hapticDampingFactor = dampingGain * voltageLevel;
            
            // display data to scope
            scope->setSignalValues(voltageLevel);
          
        }

        // update frequency counter
        freqCounterSensor.signal(Count);
    }

    // the BACKGROUND operation must be explicitly stopped
    //     Parameters:
    //          BoardNum    :the number used by CB.CFG to describe this board
    //          FunctionType: A/D operation (AIFUNCTIOM)
    ULStat = cbStopBackground(BoardNum, AIFUNCTION);

    cbWinBufFree(MemHandle);

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

void updateRobotDevice(void)
{
    // activate robot forces
    robotDevice->enableForces(true);

    // get time point 0
    chrono::high_resolution_clock::time_point timePoint0 = chrono::high_resolution_clock::now();


    // main robot control loop
    while (simulationRunning)
    {
        // get time point 1

        chrono::high_resolution_clock::time_point timePoint1 = chrono::high_resolution_clock::now();
        double timeInSeconds = chrono::duration<double>(timePoint1 - timePoint0).count();

        // get current position of robot
        cVector3d pos(0, 0, 0);
        robotDevice->getPosition(pos);

        Vec3 robotPos(pos.x(), pos.y(), pos.z());

        // ----- Buffer to store robot pos ----- //

		// Get address of current active buffer
        Vec3* currentActive = active.load(memory_order_acquire);

		// Choose the target buffer (the one that is NOT active)
        Vec3* target = (currentActive == &bufferA) ? &bufferB : &bufferA;

		// Write in new robot position
        *target = robotPos;

		// Set this buffer as active
        active.store(target, std::memory_order_release);

        // ----------------------------------------

        // get current velocity of robot
        cVector3d vel(0, 0, 0);
        robotDevice->getLinearVelocity(vel);


        // acquire mutex
        mutexDevices.acquire();

        // update variables by taking into account rotation matrix of robot
        robotPosCur = robotRot * pos;
        robotVelCur = robotRot * vel;
       

        //// compute spring force to move robot toward desired position (robotPosDes) 
        double Kp = 5000;
        double Kv = 25;
        cVector3d force = Kp * (robotPosDes - robotPosCur) - Kv * robotVelCur;


        auto_scan();
        // release mutex
        mutexDevices.release();

        // apply force after taking into account rotation matrix of robot
        robotDevice->setForce(cTranspose(robotRot) * force);

        add_value();

        // update frequency counter
        freqCounterRobotDevice.signal();


    }

    // close connection to robot
    robotDevice->close();
}


//------------------------------------------------------------------------------


void updateHapticDevice(void)
{

    //INITIALIZATION

    // variable to store robot device position when user button is pressed
    cVector3d robotPosDes0;

    // variable to store haptic device position when user button is pressed
    cVector3d hapticPos0;

    // variable to store robot device position when user button is pressed
    cVector3d robotPos0(0, 0, 0);

	// variable to store virtual robot position in the haptic frame
    cVector3d virtualRobotPos(0, 0, 0);


    // activate haptic forces
    hapticDevice->enableForces(true);

    // main haptic control loop
    while (simulationRunning)
    {
        //--------------------------------------------------------------------------
        // GET DATA FROM HAPTIC DEVICE
        //--------------------------------------------------------------------------

        // get current position of haptic device
        cVector3d hapticPos(0, 0, 0);
        hapticDevice->getPosition(hapticPos);

        // get current velocity of haptic device
        cVector3d hapticVel(0, 0, 0);
        hapticDevice->getLinearVelocity(hapticVel);

        // get user button state
        bool userButton0 = false;
        hapticDevice->getUserSwitch(0, userButton0);

        bool userButton1 = false;
        hapticDevice->getUserSwitch(1, userButton1);

        bool userButton = userButton0 | userButton1;

        
		//  Get the positon of the manipulating robot from the buffer  
        Vec3 robotPos;
        Vec3* src = active.load(memory_order_acquire);
        robotPos = *src; 

        
		// THG_signal_normalization (max value depends on the scale factor --> TO ADAPT depending on the input power) 
        double min_voltage = 0.047;
        double max_voltage = 0.0;

        if (scaleFactor == 0.02) {
            max_voltage = 0.3;
        }
        else if (scaleFactor == 0.001) {
            max_voltage = 0.9;
        }
        
        double THGsignal = cClamp((voltageLevel - min_voltage) / (max_voltage - min_voltage), 0.0, 1.0);  

		// for the other scale factors, we don't want any haptic feedback based on THG signal
        if (scaleFactor != 0.02 && scaleFactor != 0.001)  
        {
            THGsignal = 0.0;
        }

        //--------------------------------------------------------------------------
        // STATE MACHINE
        //--------------------------------------------------------------------------

        // acquire mutex
        mutexDevices.acquire();

        // initialize haptic force
        cVector3d force(0.0, 0.0, 0.0);

        //
        // STATE IDLE
        //
        if (state == STATE_IDLE)
        {

            // check if user has pressed the button
            if (userButton == true)
            {
                // reset clutching positions for robot device and haptic device
                robotPosDes0 = robotPosDes;
                hapticPos0 = hapticPos;

                // get current position of robot device
                robotDevice->getPosition(robotPos0);

                // go into teleoperation mode
                state = STATE_TELEOPERATION;
            }


        }

        //
        // STATE TELEOPERATION
        //
        if (state == STATE_TELEOPERATION)
        {

            if (userButton == false)
            {

                // user has released button, go into idle mode
                state = STATE_IDLE;
            }
            else
            {
                // compute new desired robot position based on new haptic device position and axis locking conditions
                if (!(lock_x || lock_y || lock_z)) {

                    robotPosDes = robotPosDes0 + scaleFactor * (hapticPos - hapticPos0);
                }
                else {
                    if (lock_x && lock_y && lock_z) {
                        robotPosDes = robotPosDes0;

                    }
                    else if (lock_x && lock_y) {
                        robotPosDes = cVector3d(robotPosDes0.x(), robotPosDes0.y(), robotPosDes0.z() + scaleFactor * (hapticPos.z() - hapticPos0.z()));

                    }
                    else if (lock_x && lock_z) {
                        robotPosDes = cVector3d(robotPosDes0.x(), robotPosDes0.y() + scaleFactor * (hapticPos.y() - hapticPos0.y()), robotPosDes0.z());

                    }
                    else if (lock_y && lock_z) {
                        robotPosDes = cVector3d(robotPosDes0.x() + scaleFactor * (hapticPos.x() - hapticPos0.x()), robotPosDes0.y(), robotPosDes0.z());

                    }
                    else if (lock_y) {
                        robotPosDes = cVector3d(robotPosDes0.x() + scaleFactor * (hapticPos.x() - hapticPos0.x()), robotPosDes0.y(), robotPosDes0.z() + scaleFactor * (hapticPos.z() - hapticPos0.z()));

                    }
                    else if (lock_z) {
                        robotPosDes = cVector3d(robotPosDes0.x() + scaleFactor * (hapticPos.x() - hapticPos0.x()), robotPosDes0.y() + scaleFactor * (hapticPos.y() - hapticPos0.y()), robotPosDes0.z());

                    }
                    else if (lock_x) {
                        robotPosDes = cVector3d(robotPosDes0.x(), robotPosDes0.y() + scaleFactor * (hapticPos.y() - hapticPos0.y()), robotPosDes0.z() + scaleFactor * (hapticPos.z() - hapticPos0.z()));
                    }
                }

                // controller gains

                double Kteleop = 100;
                double Kv = 1;


                //// compute spring force to resist movement on haptic side if the manipulation robot has not moved to the desired positon
                force = Kteleop * (robotPosCur - robotPosDes);

                //// damping
                force += -Kv * hapticVel;

                // ----------------------------------------- NEW HAPTIC MODEL ------------------------------------------- //

                if (haptic_state == FIND_INTERFACE) {


                    // damping force to help find the interface
                    double z_damping = 70 * pow(THGsignal, 3);
                    double xy_damping = 70 * (1 - pow(THGsignal, 3));

                    // normal force
                    cVector3d F_normal(0.0, 0.0, -z_damping * hapticVel.z());

                    // tangeantial force
                    cVector3d F_tangent(-xy_damping * hapticVel.x(), -xy_damping * hapticVel.y(), 0.0);

                    // during fast positionning, we are far from the interface --> no force feedback
                    if (scaleFactor == 0.2) {
                        F_normal.x(0.0);
                        F_normal.y(0.0);
                        F_normal.z(0.0);

                        F_tangent.x(0.0);
                        F_tangent.y(0.0);
                        F_tangent.z(0.0);
                    }

                    force += F_normal + F_tangent;

                    // Store the points that corresponds to an interface point --> Take the position of the manipulating robot
                    if (voltageLevel > 0.5) { //max value TO ADAPT

                        Vec3 interface_point = { robotPos.x(),robotPos.y(),robotPos.z() };

                        double minDistance = 0.0005; //TO ADAPT

                        if (isFarEnough(list_of_interface_points, interface_point, minDistance)) {

                            cout << "Interface point detected at voltage: " << voltageLevel << " V and position ( " << robotPos.x() << ", " << robotPos.y() << ", " << robotPos.z() << " )" << endl;
                            list_of_interface_points.emplace_back(robotPos.x(), robotPos.y(), robotPos.z());
                        }

                    }

                    // If enough interface points --> linear approximation + next state
                    if (list_of_interface_points.size() >= 2) {

                        cout << "Enough interface points collected: " << list_of_interface_points.size() << " points." << endl;

                        PointA = list_of_interface_points[0];
                        PointB = list_of_interface_points[1];

                        // Change to the next haptic state
                        haptic_state = LINEAR_EXPLORATION;
                        find_interface = 0;

                    }
                }

                else if (haptic_state == LINEAR_EXPLORATION) {

                    virtualRobotPos = robotPos + scaleFactor * (hapticPos - hapticPos0);

                    double correction_factor = 0.7;

                    // -------------------- Correction of the virtual robot position to match the real one ------------------- //
                    virtualRobotPos += correction_factor * (robotPos - virtualRobotPos);

                    //conversion in Vec3 to use eigen functions
                    Vec3 virtualRobotPos_Vec3(virtualRobotPos.x(), virtualRobotPos.y(), virtualRobotPos.z());

                    // Compute haptic force to follow the line defined by points A and B
                    double dist_perp = distanceToLine(virtualRobotPos_Vec3, PointA, PointB);
                    Vec3 dir_perp = perpendicularDirectionToLine(virtualRobotPos_Vec3, PointA, PointB);

                    // Exponential filtering of dir perp

                    double alpha = 0.5; //TO ADAPT

                    Vec3 dir_perp_filtered;

                    if (!dir_perp_initialized) {
                        last_dir_perp_filtered = dir_perp;
                        dir_perp_initialized = true;
                    }
                    else {
                        dir_perp_filtered = alpha * dir_perp + (1.0 - alpha) * last_dir_perp_filtered;

                        if (dir_perp_filtered.norm() > 1e-6)
                            dir_perp_filtered.normalize();

                        last_dir_perp_filtered = dir_perp_filtered;
                    }

					//// if we don't want to use filtering
     //               dir_perp_filtered = dir_perp;


                    // Stiffness & damping
                    double K_line = 700 / scaleFactor;
                    double B_line = 100;

                    // Vitesse perpendiculaire : projeter la vitesse sur la direction perpendiculaire
                    Vec3 V(hapticVel.x(), hapticVel.y(), hapticVel.z());
                    double vel_normal = V.dot(dir_perp_filtered);

                    // Force haptique perpendiculaire
                    Vec3 F_perp = -K_line * dist_perp * dir_perp_filtered - B_line * vel_normal * dir_perp_filtered;

                    double force_magnitude = F_perp.norm();

                    // Conversion vers CHAI3D
                    cVector3d F_line_haptic(F_perp[0], F_perp[1], F_perp[2]);
                    force += F_line_haptic;



                }


            }
        }


            //////////////////////////////////////////////////////////////////////////////////////////////
            //
            // compute a planar haptic spring force on all three axis. This force is computed
            // for both IDLE and TELEOPERATION states
            //
            //////////////////////////////////////////////////////////////////////////////////////////////
            cVector3d springforce;
            double forcex = 0;
            double forcey = 0;
            double forcez = 0;
            //Kp = 20; // définir coefficent approprié.
            //double Kv = 5;

            // Compute the force to lock haptic in x,y or z direction according to which variable is set to true
            axis_locking(&forcex, &forcey, &forcez);
            force += cVector3d(forcex, forcey, forcez);
            //force.set(forcex, forcey, forcez);//applying force to the haptic device


            // release mutex
            mutexDevices.release();

            updateCamera();
            //--------------------------------------------------------------------------
            // SEND FORCE TO HAPTIC DEVICE
            //--------------------------------------------------------------------------

            // apply force to haptic device
            hapticDevice->setForce(force);

            // update frequency counter
            freqCounterHapticDevice.signal();
        }     

        // close connection to haptic device
        hapticDevice->close();
    }

// 
// 
// 
// 
// 
// 
// ----------------------------------------------------------------------








// This function locks restricts mouvement along x,y or z  axis
// by applying a spring force to the axis coordinate, that is set when
// user presses the button.
void axis_locking(double* forcex, double* forcey, double* forcez) {
    double K_axis = 2000, Kd = 5;
    cVector3d hapticPos(0, 0, 0);
    hapticDevice->getPosition(hapticPos);

    // get current velocity of haptic device
    cVector3d hapticVel(0, 0, 0);
    hapticDevice->getLinearVelocity(hapticVel);


    if (lock_x) *forcex += cClamp(K_axis * (posX.x() - hapticPos.x()),-30.0, 30.0); 
    else if (!lock_x)  *forcex += 0;

    if (lock_y) *forcey += cClamp(K_axis * (posY.y() - hapticPos.y()), -30.0, 30.0);
    else if (!lock_y) *forcey += 0;

    if (lock_z) *forcez += cClamp(K_axis * (posZ.z() - hapticPos.z()), -30.0, 30.0);
    else if (!lock_z) *forcez += 0;

    return;
}

// This function does a z-axis sweep across the sample to the find the 
// maximum z-coordinate, which corresponds to the surface of the sample
void auto_scan(void) {
    cVector3d maxPos;
    cVector3d scan_vector(0, 0, 0);
    if (scan_x == true || scan_y == true || scan_z == true) {
        static int i = 0;
        if (i % 10 == 0) {
            if (scan_x) outFile << robotPosCur.x() << "," << voltageLevel << endl;
            else if (scan_y) {
                outFile << robotPosCur.y() << "," << voltageLevel << endl;
            }
            else if (scan_z) {
                outFile << robotPosCur.z() << "," << voltageLevel << endl;
                maxSignal = updateMax(robotPosCur, voltageLevel, maxPosition, false);
                //threshold = maxSignal;
            }
            outFile.flush();
            i = 0;
 
        }
        i++;
        if (scan_x) scan_vector.set(0.01 * microns, 0, 0);
        else if (scan_y) scan_vector.set(0, 0.01 * microns, 0);
        else if (scan_z) scan_vector.set(0, 0, 0.01 * microns);  
        robotPosDes = robotPosDes + scan_vector;
    }
    return;
}

//Ths fuction adds sampled value to a vector so that the mesh can be computed

void add_value(void) {
    cVector3d render_position = robotPosDes - offset;
    if (meshPositionsVector.size() > 1000 && voltageLevel> 0.1) {
        meshPositionsVector.pop_front();
        meshPositionsVector.pop_front();
        meshPositionsVector.pop_front();
        meshPositionsVector.push_back(render_position.x());
        meshPositionsVector.push_back(render_position.y());
        meshPositionsVector.push_back(render_position.z());
    }
    else if (voltageLevel > 0.1) {
        meshPositionsVector.push_back(render_position.x());
        meshPositionsVector.push_back(render_position.y());
        meshPositionsVector.push_back(render_position.z());
    }
    createSurfaceMeshFromTetgen(out);
}

//This function resest all drawn pixels 
void reset_sample() {
    meshPositionsVector.clear();
    if (myMesh) world->removeChild(myMesh);
}

 // This function will be called when the user scrolls
void scrollCallback(GLFWwindow * window, double xoffset, double yoffset)
    {
    if (yoffset > 0)
        cameraDistance *= 0.9;  // Zoom in
    else if (yoffset < 0)
        cameraDistance *= 1.1;  // Zoom out

    // Clamp
    if (cameraDistance < 0.005) cameraDistance = 0.005;
    if (cameraDistance > 2.0)  cameraDistance = 2.0;

    updateCamera();
}


void updateCamera()
{
    double x = cameraDistance * cos(cameraAngle);
    double y = cameraDistance * sin(cameraAngle); 
    double z = cameraHeight;
    

    cVector3d cameraPosition(x, y, z);
    cVector3d up(0.0, 0.0, 1.0);  // Up vector
    lookAt = cVector3d(0, 0, 0);//cursorRobotPosDes->getLocalPos();
    camera->set(cameraPosition, lookAt, up);
}


//  Function to compute the area of a triangle
double triangleArea(const cVector3d& a, const cVector3d& b, const cVector3d& c) {
    cVector3d temp1 = b - a;
    cVector3d temp2 = c - a;
    temp1.cross(temp2);
    double area = 0.5 * temp2.length();
    return area;
}


// Function to compute if any side of the triangle exceeds the threshold
bool anyEdgeExceedsLength(const cVector3d& a, const cVector3d& b, const cVector3d& c)
{
    return ((a-b).length() > length_threshold) ||
        ((b-c).length() > length_threshold) ||
        ((c-a).length() > length_threshold);
}

cMesh* createSurfaceMeshFromTetgen(tetgenio& out) {
    using Triangle = tuple<int, int, int>;
    map<Triangle, int> faceCount;

    //Count faces from all tetgen tetrahedra
    for (int i = 0; i < out.numberoftetrahedra; i++) {
        int* tet = &out.tetrahedronlist[i * 4];
        int v[4] = { tet[0], tet[1], tet[2], tet[3] };

        vector<array<int, 3>> faceVerts = {
        {v[0], v[1], v[2]},
        {v[0], v[1], v[3]},
        {v[0], v[2], v[3]},
        {v[1], v[2], v[3]},
        };

        for (auto& verts : faceVerts) {
            sort(verts.begin(), verts.end());  // Sort the 3 vertex indices
            Triangle f = Triangle(verts[0], verts[1], verts[2]);
            faceCount[f]++;
        }
    }

    //Create the cha3d mesh and add vertices
    auto* mesh = new cMesh();
    vector<cVector3d> vertices;
    for (int i = 0; i < out.numberofpoints; ++i) {
        double x = out.pointlist[i * 3 + 0];
        double y = out.pointlist[i * 3 + 1];
        double z = out.pointlist[i * 3 + 2];
        
        cVector3d vert(x, y, z);
        mesh->newVertex(vert);
        vertices.push_back(vert);
    }

    // Filter triangles that might be noise
    for (const auto& [face, count] : faceCount) {
            int i0 = get<0>(face);
            int i1 = get<1>(face);
            int i2 = get<2>(face);

            const auto& v0 = vertices[i0];
            const auto& v1 = vertices[i1];
            const auto& v2 = vertices[i2];

            double area = triangleArea(vertices[i0], vertices[i1], vertices[i2]);
            if(area <= area_threshold && !anyEdgeExceedsLength(v0, v1, v2)) {
                mesh->newTriangle(i0, i1, i2);
            }
    }

    // Compute normals for proper lighting
    mesh->computeAllNormals();
    mesh->m_material->setRedCrimson();
    mesh->setUseMaterial(true);

    return mesh;
}