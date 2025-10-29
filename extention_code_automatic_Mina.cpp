//////////////////////////////////////////////////////////////////////////////////////////////
        // 
        // Automatic scan code (Not runed yet) (Mina.Thor@epfl.ch)
        //
         // psodocode:
// Have a boothon a_key == non used button e.g 'w'
// initialte start by the STATE_IDEL (booten is pressed on the haptic device)
// implement scan simulationFinished() so that the movment happends in the arm (!)
// NB check that the scan work WITOUT the sample (not with the sample, may destroy it all)

///////////////////////////////////////////////////////////////////////////////////////////
//Notes:
// could add bibber steps?
//Needs trace in the window- wandt both trace and output 
//There is a bug sothe automatic code only run if i do it as teh first thing i do 
// (There is a bug in the a key atleast (the automatic code from before does not run and seems to "halt" the sytem) 


//////////////////////////////////////////////////////////////////////////////////////////////
// Schould be placed in "void while loop with key apllications"

 //--------------------------------------------------------------------------
 // Placemnt in overall code: 
 //--------------------------------------------------------------------------

// main placment visula interface

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
    cour << "[T] - Test automatic code Mina" << endl; // Addition to test implementation of Mina 
    cout << endl << endl;

// option - reset offset
    if (a_key == GLFW_KEY_T)
    {
        StateMode = STATE_AUTO;
    }


 //--------------------------------------------------------------------------
 // Code to test with robot output
 //--------------------------------------------------------------------------


 if (StateMode == STATE_AUTO_M)
        {
            cVector3d robotPosDesMod;
            if (timeInSeconds >= 0.1)
            {
                double diffx = robotPosDesScan.x() - robotPosScan0.x();
                double diffy = robotPosDesScan.y() - robotPosScan0.y();
                double diffz = robotPosDesScan.z() - robotPosScan0.z();
                double maxx = robotPosScan0.x() + 0.001; // Movment in robot or movment in something else?
                double maxy = robotPosScan0.y() + 0.001;
                double maxz = robotPosScan0.z() + 0.001;
                double stepx = 0.0001;
                double stepy = 0.0001;
                double stepz = 0.0001;

                if (StateScanx == STATE_FORWARD_X)
                {
                    double Modx = robotPosDesScan.x() + stepx;
                    double Mody = robotPosDesScan.y();
                    double Modz = robotPosDesScan.z();
                    robotPosDesScan.set(Modx, Mody, Modz);
                    obotPosDes=robotPosDesScan; //Movment from haptic to robot (?)
                    timePoint0 = chrono::high_resolution_clock::now();
                    if (diffx >= maxx)
                    {

                        if (StateScany == STATE_FORWARD_Y)
                        {
                            Mody = robotPosDesScan.y() + stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy >= maxy)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                obotPosDes=robotPosDesScan; //Movment from haptic to robot (?)
                                StateScany = STATE_BACKWARD_Y;
                            }

                        }
                        else if (StateScany == STATE_BACKWARD_Y)
                        {
                            Mody = robotPosDesScan.y() - stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy <= 0)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                obotPosDes=robotPosDesScan; //Movment from haptic to robot (?)
                                StateScany = STATE_FORWARD_Y;
                            }
                        }
                        StateScanx = STATE_BACKWARD_X;
                    }

                }
                else if (StateScanx == STATE_BACKWARD_X)
                {
                    double Modx = robotPosDesScan.x() - stepx;
                    double Mody = robotPosDesScan.y();
                    double Modz = robotPosDesScan.z();
                    robotPosDesScan.set(Modx, Mody, Modz);
                    obotPosDes=robotPosDesScan; //Movment from haptic to robot (?)
                    timePoint0 = chrono::high_resolution_clock::now();
                    stepx = -0.0003;
                    if (diffx <= 0)
                    {
                        if (StateScany == STATE_FORWARD_Y)
                        {
                            Mody = robotPosDesScan.y() + stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy >= maxy)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                obotPosDes=robotPosDesScan; //Movment from haptic to robot (?)
                                StateScany = STATE_BACKWARD_Y;
                            }
                        }
                        else if (StateScany == STATE_BACKWARD_Y)
                        {
                            Mody = robotPosDesScan.y() - stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy <= 0)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                robotPosDes=robotPosDesScan; //Movment from haptic to robot (?)
                                StateScany = STATE_FORWARD_Y;
                            }
                        }
                        StateScanx = STATE_FORWARD_X;
                    }
                }
                if (diffz >= maxz)
                {
                    StateMode = STATE_MANUEL;
                }
            }
            
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
                double Kp = 2000; // d�finir coefficent appropri�. 
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

            Kp_factor = voltageLevel * 500; // d�finir coefficent appropri�. (max signal = 2V (23.08.23) & max Kp =1000 -> coeff = 500
            double Kv = 5;
            double Kp = cClamp(Kp_factor, 0.0, 1000.0);//mettre d�pendance k � l'intensit�
            //hapticPosPlan0 = PosMax;

            forcez = -Kp * (hapticPos.z() - hapticPosPlan0.z()) - Kv * hapticVel.z();
            forcex = 0;
            forcey = 0;

            // set spring force vector
            springforce.set(forcex, forcey, forcez);//applying force to the haptic device

            // add spring force to final force
            force = force + springforce;

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




// check if user has pressed the button
        if (userButton == true)
            {
                // reset clutching positions for robot device and haptic device
                robotPosDes0 = robotPosDes;
                hapticPos0 = hapticPos;

                // go into teleoperation mode
                state = STATE_TELEOPERATION;
            }
// apply force to haptic device
        hapticDevice->setForce(force);

        // update frequency counter
        freqCounterHapticDevice.signal();


//itterativt hente data fra automatisc code så lenge knapp er dyttet ned... (begynner her)

while userButton == true {
    robotPosDes0 = robotPosDes;
    hapticPos0 = hapticPos;
}

int STATE_AUTO_HAPTIC()
{
   if (StateMode == STATE_AUTO)
        {
            cVector3d robotPosDesMod;
            if (timeInSeconds >= 0.1)
            {
                double diffx = robotPosDesScan.x() - robotPosScan0.x();
                double diffy = robotPosDesScan.y() - robotPosScan0.y();
                double diffz = robotPosDesScan.z() - robotPosScan0.z();
                double maxx = robotPosScan0.x() + 0.001;
                double maxy = robotPosScan0.y() + 0.001;
                double maxz = robotPosScan0.z() + 0.001;
                double stepx = 0.0001;
                double stepy = 0.0001;
                double stepz = 0.0001;

                if (StateScanx == STATE_FORWARD_X)
                {
                    double Modx = robotPosDesScan.x() + stepx;
                    double Mody = robotPosDesScan.y();
                    double Modz = robotPosDesScan.z();
                    robotPosDesScan.set(Modx, Mody, Modz);
                    timePoint0 = chrono::high_resolution_clock::now();
                    if (diffx >= maxx)
                    {

                        if (StateScany == STATE_FORWARD_Y)
                        {
                            Mody = robotPosDesScan.y() + stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy >= maxy)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                StateScany = STATE_BACKWARD_Y;
                            }

                        }
                        else if (StateScany == STATE_BACKWARD_Y)
                        {
                            Mody = robotPosDesScan.y() - stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy <= 0)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                StateScany = STATE_FORWARD_Y;
                            }
                        }
                        StateScanx = STATE_BACKWARD_X;
                    }

                }
                else if (StateScanx == STATE_BACKWARD_X)
                {
                    double Modx = robotPosDesScan.x() - stepx;
                    double Mody = robotPosDesScan.y();
                    double Modz = robotPosDesScan.z();
                    robotPosDesScan.set(Modx, Mody, Modz);
                    timePoint0 = chrono::high_resolution_clock::now();
                    stepx = -0.0003;
                    if (diffx <= 0)
                    {
                        if (StateScany == STATE_FORWARD_Y)
                        {
                            Mody = robotPosDesScan.y() + stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy >= maxy)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                StateScany = STATE_BACKWARD_Y;
                            }
                        }
                        else if (StateScany == STATE_BACKWARD_Y)
                        {
                            Mody = robotPosDesScan.y() - stepy;
                            robotPosDesScan.set(Modx, Mody, Modz);
                            if (diffy <= 0)
                            {
                                Modz = robotPosDesScan.z() + stepz;
                                robotPosDesScan.set(Modx, Mody, Modz);
                                StateScany = STATE_FORWARD_Y;
                            }
                        }
                        StateScanx = STATE_FORWARD_X;
                    }
                }
                if (diffz >= maxz)
                {
                    StateMode = STATE_MANUEL;
                }
            }
            
        }



        // release mutex
        mutexDevices.release();

        // apply force after taking into account rotation matrix of robot
        robotDevice->setForce(cTranspose(robotRot) * force);


        // update frequency counter
        freqCounterRobotDevice.signal();


    }

return 

}

if (state == STATE_AUTO_M) //Somtin wong with this implementation of the auto_state
{
    if (userButton == true)
    {
        // user has released button, go into idle mode
        state = STATE_AUTO_M;
    }
    else {
        state = STATE_IDLE;
    }