#ifndef PUSH_PUSH_CONTROL_H
#define PUSH_PUSH_CONTROL_H

#include "mujoco.h"
#include <Eigen/Dense>

/// Type definitions
typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eigMm;

class PushControl
{
public:
    /// Constructor & Destructor
    PushControl(const mjModel* model);
    ~PushControl() {}
    /// Functions
    void setControl(mjData* d, const Eigen::VectorXd eePosD, const Eigen::VectorXd eeQuatD);
    /// Model parameters
    int *dAct;
    int eeBodyID, eeSiteID;
    /// Jacobian matrices
    eigMm Jt, Jr, J, Jinv;
    /// Control, error, and pose variables
    Eigen::VectorXd u, e, eePos, eeQuat, eeRVelD;
    /// Controller gains
    double Kp, Kd;
private:
    /// MuJoCo model
    const mjModel* m;
};


#endif //PUSH_PUSH_CONTROL_H
