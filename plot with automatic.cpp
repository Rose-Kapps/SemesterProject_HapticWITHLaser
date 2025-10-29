//==============================================================================
//
//  TELEOPERATION
//
//==============================================================================

#include "chai3d.h"
#include "cbw.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <chrono>
#include <thread>

using namespace chai3d;
using namespace std;

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
//------------------------------------------------------------------------------
enum cState
{
    STATE_IDLE,
    STATE_TELEOPERATION
};
enum cStateGradient
{
    STATE_AQUIREPOINT1,
    STATE_AQUIREPOINT2
};
enum cStateMode
{
    STATE_MANUEL,
    STATE_AUTO
}
enum cStateScan
{
    STATE_FORWARD_X,
    STATE_BACKWARD_X,
    STATE_FORWARD_Y
}



//------------------------------------------------------------------------------
// GENERAL VARIABLES
//------------------------------------------------------------------------------

//variables for gradient:

bool enable_magnet_Z = false;
double gradient;
double deltaz = 0.00005; // chercher la bonne valeur !!
vector<double> vectDeltaz = { 0, 0, deltaz };
cVector3d P1;
double P1z;
cVector3d P2;
double P2z;
double I1;
double I2;
double diffz;
cVector3d forceGradient;
cVector3d hapticPosGrad0;// position enregistrée quand le mode approach est activé.


double intensityThreshold = 0.3;//verifier si coeff est approprié.

//variable for vibration:
bool vibration = false;

//variable for lock in plan xy:
bool plan_xy = false;
cVector3d hapticPosPlan0; // position enregistrée quand le mode plan est activée

double voltageLevel;
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

// state machine
cState state;

// state gradient
cStateGradient stateGradient = STATE_AQUIREPOINT1;

// state mode
cStateMode StateMode = STATE_MANUEL; 




//------------------------------------------------------------------------------
// CHAI3D GRAPHIC VARIABLES AND OBJECTS
//------------------------------------------------------------------------------

// a flag which indicates if the fullscreen mode is activated
bool fullscreen = false;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

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

cShapeSphere* cursorRobotPosDesScan;

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



//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

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
    cout << "[g] - Enable Z magnet" << endl;
    cout << "[v] - Enable XY vibration" << endl;
    cout << "[p] - Enable XY plan" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

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
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.03, 0.0, 0.02),    // camera position (eye)
        cVector3d(0.0, 0.0, 0.0),    // look at position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

// set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.0001, 1.0);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);


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
    
    RobotPosDes = new cShapeSphere(0.0003);

    // add cursor to world
    world->addChild(cursorRobotPosDes);

    // set cursor color
    cursorRobotPosDes->m_material->setBlueCornflower();


    //--------------------------------------------------------------------------
    // CREATE ROBOT SPHERE simulation (scanning)
    //--------------------------------------------------------------------------

    // create small sphere

    RobotPosDesScan = new cShapeSphere(0.0003);

    // add cursor to world
    world->addChild(cursorRobotPosDesScan);

    // set cursor color
    cursorRobotPosDesScan->m_material->setRed();



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
    object->m_minCorner.set(-0.008, -0.008, -0.008);
    object->m_maxCorner.set(0.008, 0.008, 0.008);

    // set the texture coordinate at each corner.
    object->m_minTextureCoord.set(0.0, 0.0, 0.0);
    object->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set material color
    object->m_material->setOrangeCoral();

    // show/hide boundary box
    object->setShowBoundaryBox(true);


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

    // rotate the robot device base to the desired angle
    robotRot.rotateAboutGlobalAxisDeg(0, 0, 1, 180);


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
    cVector3d pos;
    robotDevice->getPosition(pos);

    // update current robot position by taking into account robot rotation matrix
    robotPosCur = robotRot * pos;

    // set desired robot position to current robot position
    robotPosDes = robotPosCur;

    // open connection to haptic device
    hapticDevice->open();


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

    //option - activation of the scanning mode
    if (a_key == GLFW_KEY_A)
    {
        StateMode = STATE_AUTO;

    }
    //option - activation of the manuel mode
    if (a_key == GLFW_KEY_M)
    {
        StateMode = STATE_MANUEL;

    }
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    //option - activation of the gradient approach
    if (a_key == GLFW_KEY_G)
    {
        enable_magnet_Z = !enable_magnet_Z;
        hapticDevice->getPosition(hapticPosGrad0);

    }

    //option - activation of the vibration in x and y
    if (a_key == GLFW_KEY_V)
    {
        vibration = !vibration;
    }

    //option - activation of lock in pln xy
    if (a_key == GLFW_KEY_P)
    {
        plan_xy = !plan_xy;
        hapticDevice->getPosition(hapticPosPlan0);
    }


    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - scale factor 0.005
    if ((a_key == GLFW_KEY_0) && (state == STATE_IDLE))
    {
        scaleFactor = 0.005;
    }


    // option - scale factor 0.02
    if ((a_key == GLFW_KEY_1) && (state == STATE_IDLE))
    {
        scaleFactor = 0.02;
    }

    // option - scale factor 0.05
    if ((a_key == GLFW_KEY_2) && (state == STATE_IDLE))
    {
        scaleFactor = 0.05;
    }

    // option - scale factor 0.2
    if ((a_key == GLFW_KEY_3) && (state == STATE_IDLE))
    {
        scaleFactor = 0.2;
    }

    // option - scale factor 0.5
    if ((a_key == GLFW_KEY_4) && (state == STATE_IDLE))
    {
        scaleFactor = 0.5;
    }

    // option - reset offset
    if (a_key == GLFW_KEY_C)
    {
        offset = robotPosDes;
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
        "Scale Factor: " + cStr(scaleFactor, 2) + "x");

    // update position of label
    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getWidth())), 15);

    // update labelgradient et mettre à jour la pos du label, mettre au dessus de labelMessage
    labelGradient->setLocalPos((int)(0.5 * (width - labelGradient->getWidth())), 35);//centre le message au bas de l'écran

    labelGradient->setText("Vibration: " + cStr(vibration) + "  /  " +
        "Z magnet : " + cStr(enable_magnet_Z) + "  /  " +
        "XY plan : " + cStr(plan_xy));






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

    // update position of cursor for scanning
    mutexDevices.acquire();
    cursorRobotPosDesScan->setLocalPos(robotPosDesScan);
    mutexDevices.release();


    /////////////////////////////////////////////////////////////////////
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
    int voxelIndexX = (double)((a_pos.x() - object->m_minCorner.x()) * numVoxelX) / sizeX;
    int voxelIndexY = (double)((a_pos.y() - object->m_minCorner.y()) * numVoxelY) / sizeY;
    int voxelIndexZ = (double)((a_pos.z() - object->m_minCorner.z()) * numVoxelZ) / sizeZ;

    // clamp index values within image volume
    voxelIndexX = cClamp(voxelIndexX, 0, numVoxelX);
    voxelIndexY = cClamp(voxelIndexY, 0, numVoxelY);
    voxelIndexZ = cClamp(voxelIndexZ, 0, numVoxelZ);

    // update voxel color
    object->m_texture->m_image->setVoxelColor(voxelIndexX, voxelIndexY, voxelIndexZ, a_color);

    // mark voxel for update
    mutexVoxel.acquire();
    volumeUpdate.enclose(cVector3d(voxelIndexX, voxelIndexY, voxelIndexZ));
    mutexVoxel.release();
    flagMarkVolumeForUpdate = true;

    // cout << voxelIndexX << " " << voxelIndexY << " " << voxelIndexZ << endl;
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
    int Range = UNI5VOLTS;
    short Status = 0;
    long CurCount;
    long CurIndex;
    int Count = 100;
    long Rate = 100000;
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

    // main haptic simulation loop
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

            // convert data value to a sensor voltage level
            voltageLevel = 5.0 * (((double)(dataValue)-2048.0) / 964.0);

            // compute a haptic damping factor based on laser signal
            double dampingGain = 0.2;
            hapticDampingFactor = dampingGain * voltageLevel;

            // display data to scope
            scope->setSignalValues(voltageLevel);

            // draw a voxel if voltage level reaches a certain value
           // draw a voxel if voltage level reaches a certain value
            if (voltageLevel > 0.5)
            {
                cColorb color;
                color.setR(0);
                color.setG(0);
                color.setB(128);
                setVoxel(robotPosCur - offset, color);
            }
            if (voltageLevel > 1)
            {
                cColorb color;
                color.setR(0);
                color.setG(0);
                color.setB(255);
                setVoxel(robotPosCur - offset, color);
            }
            if (voltageLevel > 1.5)
            {
                cColorb color;
                color.setR(0);
                color.setG(128);
                color.setB(0);
                setVoxel(robotPosCur - offset, color);
            }
            if (voltageLevel > 2)
            {
                cColorb color;
                color.setR(0);
                color.setG(255);
                color.setB(0);
                setVoxel(robotPosCur - offset, color);
            }
            if (voltageLevel > 2.5)
            {
                cColorb color;
                color.setR(255);
                color.setG(255);
                color.setB(0);
                setVoxel(robotPosCur - offset, color);
            }
            if (voltageLevel > 3)
            {
                cColorb color;
                color.setR(255);
                color.setG(160);
                color.setB(122);
                setVoxel(robotPosCur - offset, color);
            }

            if (voltageLevel > 3.5)
            {
                cColorb color;
                color.setR(255);
                color.setG(127);
                color.setB(160);
                setVoxel(robotPosCur - offset, color);
            }
            if (voltageLevel > 4)
            {
                cColorb color;
                color.setR(255);
                color.setG(0);
                color.setB(0);
                setVoxel(robotPosCur - offset, color);
            }

            //compute gradient along Z axis:
            if (stateGradient == STATE_AQUIREPOINT1)
            {

                I1 = voltageLevel;//save voltage intensity at the current position of the robot
                robotDevice->getPosition(P1); //save current position of the robot
                stateGradient = STATE_AQUIREPOINT2;
            }
            else if (stateGradient == STATE_AQUIREPOINT2)
            {

                robotDevice->getPosition(P2);//save current position of the robot
                diffz = P2.z() - P1.z();// calcul of the displacement in z-axis
                if (abs(diffz) > deltaz)// check if the displacement in z-axis is big enough to calulate a gradient 
                {
                    I2 = voltageLevel;
                    gradient = (I1 - I2) / diffz;//calculate the gradient
                    //gradientData.push_back(gradient);
                    stateGradient = STATE_AQUIREPOINT1;
                }
                else
                {
                    stateGradient = STATE_AQUIREPOINT2;
                }


            }
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

        // get current velocity of robot
        cVector3d vel(0, 0, 0);
        robotDevice->getLinearVelocity(vel);

        // acquire mutex
        mutexDevices.acquire();

        // update variables by taking into account rotation matrix of robot
        robotPosCur = robotRot * pos;
        robotVelCur = robotRot * vel;

        // compute spring force to move robot toward desired position (robotPosDes) 
        double Kp = 2000;
        double Kv = 10;
        cVector3d force = Kp * (robotPosDes - robotPosCur) - Kv * robotVelCur;

        //code vibration ICI !!!!!
        
        if (vibration == true)
        {
            cVector3d robotPosDesMod;
            cVector3d oscillation;
            double ampl = 0.0003; // amplitude A = 0.1 mm
            double freq = 20; //frequency f = 10Hz
            double Modx = robotPosDes.x() + ampl * sin(2 * M_PI * freq * timeInSeconds);
            double Mody = robotPosDes.y() + ampl * sin(2 * M_PI * freq * timeInSeconds);
            double Modz = robotPosDes.z() + ampl * sin(2 * M_PI * freq * timeInSeconds);

            robotPosDesMod.set(Modx, Mody, Modz);
            force = Kp * (robotPosDesMod - robotPosCur) - Kv * robotVelCur;
        }


        //code scan automatique ici :
        if (StateMode == STATE_AUTO)
        {
            cVector3d robotPosDesMod;
            cVector3d robotPosScan0;
            robotDevice->getPosition(robotPosScan0);
            double Modx = robotPosScan0.x() + robotVelCur.x() * timeInSeconds;
            double Mody;
            double Modz;
            cursorRobotPosDesScan.set(Modx, Mody, Modz);
        }


        // release mutex
        mutexDevices.release();

        // apply force after taking into account rotation matrix of robot
        robotDevice->setForce(cTranspose(robotRot) * force);

        // update frequency counter
        freqCounterRobotDevice.signal();


    }

    // close connection to robot
    robotDevice->close();
}

//------------------------------------------------------------------------------

void updateHapticDevice(void)
{
    // variable to store robot device position when user button is pressed
    cVector3d robotPosDes0;

    // variable to store haptic device position when user button is pressed
    cVector3d hapticPos0;

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
                // compute new desired robot position based on new haptic device position
                robotPosDes = robotPosDes0 + scaleFactor * (hapticPos - hapticPos0);

                //////////////////////////////////////////////////////////////////////////////////////////////
                // 
                // compute a haptic spring force between the desired robot position and its current position.
                // if the robot cannot reach a desired position, the user will feel the constraint as a spring
                // force.
                // 
                //////////////////////////////////////////////////////////////////////////////////////////////

                // set spring stiffness constant
                double Kp = 200;

                // compute spring force
                force = Kp * (robotPosCur - robotPosDes);


                //////////////////////////////////////////////////////////////////////////////////////////////
                // 
                // compute a damping force based of the laser sensor data.
                // 
                //////////////////////////////////////////////////////////////////////////////////////////////

                // set damping constant
                double Kv = 30;

                // set damping factor based of hapticDampingFactor value computed in laser sensor thread; the value
                // is clamped between 0.0 and 1.0 to avoid any instabilities from the haptic device
                double damping = cClamp(hapticDampingFactor, 0.0, 1.0);

                // calculate damping force as proportional de haptic device velocity; add force to previously computed force
                force = force - Kv * hapticDampingFactor * hapticVel;
            }
        }


        //////////////////////////////////////////////////////////////////////////////////////////////
        // 
        // compute a planar haptic spring force on axis X and Z. This force is computed 
        // for both IDLE and TELEOPERATION states
        // 
        //////////////////////////////////////////////////////////////////////////////////////////////
        if (enable_magnet_Z == true)
        {

            //threshold to block the robot on the xyplane 
            if (voltageLevel > intensityThreshold)
            {
                hapticPosPlan0 = hapticPos;
                enable_magnet_Z = false;
                plan_xy = true;


            }

            else
            {
                //write code for spring force here : 
                cVector3d springforce;
                double forcex;
                double forcey;
                double forcez;
                double Kp = 2000; // définir coefficent approprié. 
                double Kv = 5;

                // calcul of the spring force that need to be applied to the haptic device
                forcex = Kp * (hapticPosGrad0.x() - hapticPos.x()) - Kv * hapticVel.x();
                forcey = Kp * (hapticPosGrad0.y() - hapticPos.y()) - Kv * hapticVel.y();
                forcez = 0.0;

                // set spring force vector
                springforce.set(forcex, forcey, forcez);//applying force to the haptic device



                // add spring force to final force
                force = force + springforce;



                // compute a haptic damping factor based on laser signal
                double dampingGain = 0.2;
                hapticDampingFactor = dampingGain * voltageLevel;

            }


        }


        if (plan_xy == true)
        {
            cVector3d springforce;
            double forcex;
            double forcey;
            double forcez;
            double Kp_factor;

            Kp_factor = voltageLevel * 500; // définir coefficent approprié. (max signal = 2V (23.08.23) & max Kp =1000 -> coeff = 500
            double Kv = 5;
            double Kp = cClamp(Kp_factor, 0.0, 1000.0);//mettre dépendance k à l'intensité
            //hapticPosPlan0 = PosMax;

            forcez = -Kp * (hapticPos.z() - hapticPosPlan0.z()) - Kv * hapticVel.z();
            forcex = 0;
            forcey = 0;

            // set spring force vector
            springforce.set(forcex, forcey, forcez);//applying force to the haptic device

            // add spring force to final force
            force = force + springforce;

        }




        if (stateMode == STATE_AUTO)
        {

        }

        // release mutex
        mutexDevices.release();


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

