#include <Eigen/Dense>

#include "mj_render.h"

typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eigMm;
typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> eigVm;

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
    mj_jacBody(m, d, Jt.data(), Jr.data(), eeBodyID);

    Eigen::VectorXd u(m->nu), e(6), eePos(3), eePosD(3), qVel(m->nv);
    e.setZero();
    eePosD << -0.2, 0.25, 0.1;

    double Kp = 1, Kd = 0.1;

    // simulation
    bool stop = false;
    while( !glfwWindowShouldClose(mr.window) && !stop)
    {
        if( !mr.paused )
        {
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0/60.0)
            {
                mj_step1(m, d);
                mju_copy(eePos.data(), d->xpos+eeBodyID*3, 3);
                mju_copy(qVel.data(), d->qvel, m->nv);
                mj_jacBody(m, d, Jt.data(), Jr.data(), eeBodyID);
                J << Jt,
                     Jr;
                Jinv = J.inverse();
                e.head(3) = eePosD-eePos;
                u = Jinv*e;
//                std:: cout << "e: " << e.transpose() << ", u: " << u.transpose() << std::endl;
//              u = Jt.transpose()*(Kp*(eePosD-eePos));
                for( int i=0; i<m->nu; i++ )
                {
                    d->qfrc_applied[i] = d->qfrc_bias[i];
                    d->ctrl[i] = u(i);
//                    d->ctrl[i] = u(i) - Kd*d->qvel[i] + d->qfrc_bias[i];
                }
//                int jID = 5;
//                d->ctrl[jID] = 1e-2;

                mj_step2(m, d);

//                std::cout << "t: " << d->time << ", qvel: ";
//                mju_printMat(d->qvel, 1, m->nv);
//                if( fabs(d->ctrl[jID]-d->qvel[jID])<1e-6 )
//                    stop = true;
            }
            if( fmod(d->time, 1.0) < 2e-2 )
            {
                std::cout << "t: " << d->time << "\tpos: " << eePos.transpose() <<
                          ",\terror: " << (eePosD-eePos).transpose() <<
                          ",\t\tnorm(error): " << (eePosD-eePos).norm() << "\n";
            }
            mr.render();
        }
    }

    return 0;
}