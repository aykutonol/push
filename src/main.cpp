#include <Eigen/Dense>

#include "mj_render.h"

typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eigMm;

int main()
{
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
    // create rendering object
    MjRender mr(m, d);

    eigMm Jt(3, m->nv), Jr(3, m->nv), J(6, m->nv), Jinv(6, m->nv);
    Jt.setZero(); Jr.setZero(); J.setZero(); Jinv.setZero();
    int eeBodyID = mj_name2id(m, mjOBJ_BODY, "wrist_3_link");

    // simulation
    while( !glfwWindowShouldClose(mr.window) )
    {
        if( !mr.paused )
        {
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0/60.0)
            {
                mj_step1(m, d);
                mju_copy(d->ctrl, d->qfrc_bias, m->nu);
                mj_step2(m, d);
            }
            mj_jacBody(m, d, Jt.data(), Jr.data(), eeBodyID);
            J << Jt,
                    Jr;
            Jinv = J.inverse();
            std::cout << "Time: " << d->time << "\n";
            std::cout << "Jt:\n" << Jt << "\nJr:\n" << Jr << "\nJ:\n" << J << "\n";
            std::cout << "Jinv:\n" << Jinv << "\n--------------------------------\n";
        }
        mr.render();
    }

    return 0;
}