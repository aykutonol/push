#include <Eigen/Dense>

#include "mj_render.h"

int main()
{
    // activate MuJoCo license
    const char* mjKeyPath = std::getenv("MJ_KEY");
    mj_activate(mjKeyPath);
    // load the model
    mjModel* m = mj_loadXML("/home/aykut/Development/push_ws/src/push/model/ur3e.xml", NULL, NULL, 0);
    if( !m ) mju_error("ERROR: Model cannot be loaded.");
    else     std::cout << "INFO: Model successfully loaded.\n";
    // make data
    mjData* d = mj_makeData(m);
    // create rendering object
    MjRender mr(m, d);
    // simulation
    while( !glfwWindowShouldClose(mr.window) )
    {
        if( !mr.paused )
        {
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0/60.0)
            {
                mj_step(m, d);
            }
        }
            mr.render();
    }

    return 0;
}