#include <Eigen/Dense>

#include "mj_render.h"

/// Type definitions
typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eigMm;

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
    // create rendering object
    MjRender mr(m, d);
    /// Parameters
    // get the indices of actuated DOF
    int dAct[m->nu];
    for( int i=0; i<m->nu; i++ )
    {
        dAct[i] = m->jnt_dofadr[m->actuator_trnid[i*2]];
    }
    /// Initialize of Eigen variables
    // Jacobian
    eigMm Jt(3, m->nv), Jr(3, m->nv), J(6, m->nu), Jinv(6, m->nu);
    Jt.setZero(); Jr.setZero(); J.setZero(); Jinv.setZero();
    int eeBodyID = mj_name2id(m, mjOBJ_BODY, "wrist_3_link");
    int eeSiteID = mj_name2id(m, mjOBJ_SITE, "end_effector");
    int obSiteID = mj_name2id(m, mjOBJ_SITE, "object_rear");
    // control and state vectors
    Eigen::VectorXd u(m->nu), e(6), eePos(3), eePosD(3), eeQuat(4), eeQuatD(4), eeRVelD(3);
    Eigen::VectorXd difQuat(4), negQuat(4), velTest(3);
    e.setZero();
//    eePosD << 0, 0.3, 0.05;
    eeQuatD << 0, 0.7071, 0.7071, 0;
    /// Controller gains
    double Kp = 0.5, Kd = 0;
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
                // controller
                mju_copy(eePosD.data(), d->site_xpos+obSiteID*3, 3);
                mju_copy(eePos.data(),  d->site_xpos+eeSiteID*3, 3);
//                mj_jacBody(m, d, Jt.data(), Jr.data(), eeBodyID);
                mj_jacSite(m, d, Jt.data(), Jr.data(), eeSiteID);
                J << Jt.block(0, dAct[0], 3, m->nu),
                        Jr.block(0, dAct[0], 3, m->nu);
                e.head(3) = eePosD - eePos;
                mju_copy(eeQuat.data(), d->xquat+eeBodyID*4, 4);
                mju_subQuat(eeRVelD.data(), eeQuatD.data(), eeQuat.data());
                e.tail(3) = eeRVelD;
                u = Kp*J.inverse()*e;
                for (int i = 0; i < m->nu; i++) {
                    d->qfrc_applied[dAct[i]] = d->qfrc_bias[dAct[i]];
                    d->ctrl[i] = u(i) - Kd*d->qvel[dAct[i]];
                }
                // step part 2
                mj_step2(m, d);
                tStep++;
            }
            if( fmod(d->time, 1.0) < 2e-2 && !stop )
            {
                std::cout << "time: " << d->time << "\npos: " << eePos.transpose() <<
                             ", quat: " << eeQuat.transpose() <<
                             "\nposition error: " << e.head(3).transpose() <<
                             "\norientation error: " << e.tail(3).transpose() <<
                             "\nnorm(pe): " << e.head(3).norm() << ", norm(oe): " << e.tail(3).norm() <<
                             "\n=======================================================\n";
            }
        }
        mr.render();
    }

    return 0;
}