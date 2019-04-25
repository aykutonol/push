#include "push_control.h"

PushControl::PushControl(const mjModel* model) : m(model)
{
    // Get model parameters
    dAct = new int(m->nu);
    for( int i=0; i<m->nu; i++ )
    {
        dAct[i] = m->jnt_dofadr[m->actuator_trnid[i*2]];
    }
    eeBodyID = mj_name2id(m, mjOBJ_BODY, "wrist_3_link");
    eeSiteID = mj_name2id(m, mjOBJ_SITE, "end_effector");
    // Jacobian
    Jt.resize(3, m->nv);    Jr.resize(3, m->nv);    J.resize(6, m->nu);
    Jt.setZero();           Jr.setZero();           J.setZero();
    // Control and error variables
    u.resize(m->nu);    e.resize(6);
    u.setZero();        e.setZero();
    // Pose variables
    eePos.resize(3); eeQuat.resize(4); eeRVelD.resize(3);
    // Controller gains
    Kp = 1; Kd = 0;
}

void PushControl::setControl(mjData* d, const Eigen::VectorXd eePosD, const Eigen::VectorXd eeQuatD)
{
    // get end-effector position
    mju_copy(eePos.data(),  d->site_xpos+eeSiteID*3, 3);
    // calculate the position error
    e.head(3) = eePosD - eePos;
    // get the Jacobian
    mj_jacSite(m, d, Jt.data(), Jr.data(), eeSiteID);
    J << Jt.block(0, dAct[0], 3, m->nu),
         Jr.block(0, dAct[0], 3, m->nu);
    mju_copy(eeQuat.data(), d->xquat+eeBodyID*4, 4);
    mju_subQuat(eeRVelD.data(), eeQuatD.data(), eeQuat.data());
    e.tail(3) = eeRVelD;
    u = J.inverse()*e;
    for (int i = 0; i < m->nu; i++) {
        d->qfrc_applied[dAct[i]] = d->qfrc_bias[dAct[i]] - 0*d->qfrc_constraint[dAct[i]];
        d->ctrl[i] = Kp*u(i) - Kd*d->qvel[dAct[i]];
    }
}