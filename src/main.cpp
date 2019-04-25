#include "push_control.h"
#include "mj_render.h"


int main()
{
    /// Initialize MuJoCo
    // activate MuJoCo license
    const char* mjKeyPath = std::getenv("MJ_KEY");
    mj_activate(mjKeyPath);
    // load the model
    mjModel* m = mj_loadXML("/home/aykut/Development/push_ws/src/push/model/ur3e_push.xml", NULL, NULL, 0);
    if( !m ) mju_error("ERROR: Model cannot be loaded.");
    else     std::cout << "INFO: Model successfully loaded.\n";
    // make data
    mjData* d = mj_makeData(m);
    mju_copy(d->qpos, m->key_qpos, m->nq);
    mj_forward(m, d);
    // create objects
    MjRender mr(m, d);
    PushControl pc(m);

    int objBodyID = mj_name2id(m, mjOBJ_BODY, "object");
    int objSiteID = mj_name2id(m, mjOBJ_SITE, "object1");

    Eigen::VectorXd eePosD(3), eeQuatD(4), objPos(3), objPosD(3);
    // aligned with -x axis
    eeQuatD << 0, 0.7071, 0.7071, 0;
    double stepSize = 0.05;

    /// Simulation
    bool stop = false;
    int tStep = 0;
    while( !glfwWindowShouldClose(mr.window) )
    {
        if( !mr.paused )
        {
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0/60.0 && !stop)
            {
                // step part 1
                mj_step1(m, d);
                // set reference for the controller
                if( tStep%10 == 0 )
                {
                    // get the site position
                    mju_copy(eePosD.data(), d->site_xpos+objSiteID*3, 3);
                    // add the radius of the object
                    eePosD[0] += 0.02;
                    // check if task is completed
                    if( eePosD[0] < -0.1 )
                    {
                        printf("\nINFO: Task completed for %s.\n\n", "object1");
                        stop = true;
                    }
                    // if not complete, set new target along -x axis
                    eePosD[0] += -stepSize;
                }
                // calculate and set control input
                pc.setControl(d, eePosD, eeQuatD);
                // step part 2
                mj_step2(m, d);
                tStep++;
            }
            if( fmod(d->time, 1.0) < 2e-2 && !stop )
            {
                mju_copy(objPos.data(), d->site_xpos+objSiteID*3, 3);
                std::cout << "time: " << d->time << "\neePos: " << pc.eePos.transpose() <<
                             ", eeQuat: " << pc.eeQuat.transpose() <<
                             "\nposition error: " << pc.e.head(3).transpose() <<
                             "\norientation error: " << pc.e.tail(3).transpose() <<
                             "\nnorm(pe): " << pc.e.head(3).norm() << ", norm(oe): " << pc.e.tail(3).norm() <<
                             "\nobjPos: " << objPos.transpose() <<
                             "\n=======================================================\n";
            }
        }
        mr.render();
    }

    return 0;
}