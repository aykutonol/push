#include "push_control.h"
#include "mj_render.h"

void addSeparator()
{
    std::cout << "\n=======================================================\n";
}

int main()
{
    /// Initialization
    // activate MuJoCo license
    const char* mjKeyPath = std::getenv("MJ_KEY");
    mj_activate(mjKeyPath);
    // load the model
    mjModel* m = mj_loadXML("/home/aykut/Development/push_ws/src/push/model/ur3e_push.xml", NULL, NULL, 0);
    addSeparator();
    if( !m ) mju_error("ERROR: Model cannot be loaded.");
    else     std::cout << "INFO: Model successfully loaded.";
    // make data
    mjData* d = mj_makeData(m);
    mju_copy(d->qpos, m->key_qpos, m->nq);
    mj_forward(m, d);
    // create objects
    MjRender mr(m, d);
    PushControl pc(m);
    /// Model parameters
    int objSiteID;
    double rScallop = m->geom_size[mj_name2id(m, mjOBJ_GEOM, "object1")*3];
    /// Pose variables
    Eigen::VectorXd eePos0(3), eePosD(3), eeQuatD(4), objPos(3), objPosD(3);
    mju_copy(eePos0.data(), d->site_xpos+pc.eeSiteID*3, 3);
    /// Task definition
    // push direction is aligned with the -x axis
    int axis = 0;   // x axis, i.e., first component
    int dir = -1;   // direction
    double targetLine = -0.1;
    eeQuatD << 0, 0.7071, 0.7071, 0;
    double stepSize = 0.05;
    /// Objects
    int nObj = 3;
    std::vector<std::string> objectList;
    addSeparator();
    std::cout << "INFO: Objects to be pushed:\n";
    for( int i=0; i<nObj; i++ )
    {
        objectList.push_back("object"+std::to_string(i+1));
        std::cout << "\t\t" << objectList[i] << "\n";
    }
    addSeparator();
    std::cout << "Target line: x = " << targetLine;
    addSeparator();
    /// Simulation
    bool stop = false, success = false, switchObject = false, go2Object=true;
    int tStep = 0;
    int objID = 0;

    bool letsMakeAVideo = true;

    while( !glfwWindowShouldClose(mr.window) )
    {
        if( letsMakeAVideo )
        {
            mr.paused = true;
            letsMakeAVideo = false;
        }
        if( !mr.paused )
        {
            objSiteID = mj_name2id(m, mjOBJ_SITE, objectList[objID].c_str());
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0/60.0 && !stop)
            {
                // step part 1
                mj_step1(m, d);
                // set reference for the controller
                if( tStep%10 == 0 )
                {
                    // Push the object
                    if( !go2Object && !switchObject )
                    {
                        // set the site position as target
                        mju_copy(eePosD.data(), d->site_xpos+objSiteID*3, 3);
                        // shift it by radius in the direction to opposite to the push direction
                        eePosD[axis] += -dir*rScallop;
                    }
                    // go to object
                    else if( go2Object && !switchObject )
                    {
                        // set the site position as target
                        mju_copy(eePosD.data(), d->site_xpos+objSiteID*3, 3);
                        // shift it by 10 cm in the direction opposite to the push direction
                        eePosD[axis] += -dir*0.1;
                        // check if initial push pose reached
                        if( tStep>0 && pc.e.head(3).norm() < 5e-3 )
                        {
                            std::cout << "\nINFO: Initial push pose reached.\n";
                            addSeparator();
                            go2Object = false;
                            tStep = 0;
                        }
                    }
                    // go to start pose
                    else if( !go2Object && switchObject )
                    {
                        eePosD = eePos0;
                        // check if start pose reached
                        if( pc.e.head(3).norm() < 5e-3 )
                        {
                            std::cout << "\nINFO: Start pose reached.\n";
                            addSeparator();
                            switchObject = false;
                            go2Object = true;
                            tStep = -1;
                        }
                    }
                    // During task execution
                    if( !go2Object && !switchObject )
                    {
                        // check if task is completed
                        if( eePosD[0] < targetLine )
                        {
                            success=true;
                            printf("\nINFO: Task completed for %s.\n", objectList[objID].c_str());
                            if( objID == nObj-1 )
                                stop = true;
                        }
                            // set new target along the push direction until the task is completed
                        else
                        {
                            eePosD[axis] += dir*stepSize;
                        }
                    }
                }
                // calculate and set control input
                pc.setControl(d, eePosD, eeQuatD);
                // step part 2
                mj_step2(m, d);
                tStep++;
                if( success )
                {
                    success = false;
                    switchObject = true;
                    tStep = 0;
                    if( !stop )
                        objID++;
                }
            }
            if( fmod(d->time, 1.0) < 2e-2 && !stop )
            {
                mju_copy(objPos.data(), d->site_xpos+objSiteID*3, 3);
                std::cout << "time: " << d->time << "\neePos: " << pc.eePos.transpose() <<
                             ", eeQuat: " << pc.eeQuat.transpose() <<
                             "\nposition error: " << pc.e.head(3).transpose() <<
                             "\norientation error: " << pc.e.tail(3).transpose() <<
                             "\nnorm(pe): " << pc.e.head(3).norm() << ", norm(oe): " << pc.e.tail(3).norm() <<
                             "\nobjPos: " << objPos.transpose();
                addSeparator();
            }
        }
        mr.render();
    }

    return 0;
}