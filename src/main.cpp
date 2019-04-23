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
    // create rendering object
    MjRender mr(m, d);


    eigMm Jt(3, m->nv), Jr(3, m->nv);
    Jt.setZero(); Jr.setZero();
    mj_forward(m, d);
    mj_jacBody(m, d, Jt.data(), Jr.data(), mj_name2id(m, mjOBJ_BODY, "wrist_3_link"));
    std::cout << "Jt:\n" << Jt << "\n";
    std::cout << "Jr:\n" << Jr << "\n";

    mjtNum *eefJacPosMJ = mj_stackAlloc(d, 3 * m->nv);
    mjtNum *eefJacRotMJ = mj_stackAlloc(d, 3 * m->nv);
    mjtNum *armQDotCmdMJ = mj_stackAlloc(d, m->nv);

    mj_jacBody(m, d, eefJacPosMJ, eefJacRotMJ, mj_name2id(m, mjOBJ_BODY, "wrist_3_link"));

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eefJacPos, eefJacRot;
    eefJacPos = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(eefJacPosMJ, 3, m->nv);
    eefJacRot = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(eefJacRotMJ, 3, m->nv);
    std::cout << "Tarik styla:\n";
    std::cout << "Jt:\n" << eefJacPos << "\n";
    std::cout << "Jr:\n" << eefJacRot << "\n";

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
        }
        mr.render();
    }

    return 0;
}