#include "ekfstate.h"

namespace ORB_SLAM2
{
//constructor
EKFState::EKFState()
{
    //_qR.setIdentity();     // rotation
    _P.setZero();         // position
    _V.setZero();         // velocity

    _BiasGyr.setZero();   // bias of gyroscope
    _BiasAcc.setZero();   // bias of accelerometer

    _dBias_g.setZero();
    _dBias_a.setZero();
}
//copy constructor
EKFState::EKFState(const EKFState &_ns):
    _P(_ns._P), _V(_ns._V), _R(_ns._R),
    _BiasGyr(_ns._BiasGyr), _BiasAcc(_ns._BiasAcc),
    _dBias_g(_ns._dBias_g), _dBias_a(_ns._dBias_a)
{
    normalizeRotation();
}

/*InSmall(delta)
 * input: delta = [dP, dV, dTheta, dBg, dBa,dS]
 *
 *
*/
void EKFState::IncSmall(Vector16d delta)
{
    Vector3d upd_P = delta.segment(0,3);
    Vector3d upd_V = delta.segment(3,3);
    Vector3d upd_Theta = delta.segment(6,3);
    Vector3d upd_dBg = update.segment(9,3);
    Vector3d upd_dBa = update.segment(12,3);

    //Position,Velocity
    //_P += upd_P;
    //_V += upd_V;

    // rotation matrix before update
    //Matrix3d R = Get_qR().toRotationMatrix();
    // rotation
    //Matrix3d dR = Sophus::SO3::exp(upd_Theta).matrix();
    //_R = Get_R()*dR;
    //_qR = Quaterniond(R*dR);
    //normalizeRotation();    // remember to normalize rotation

}
void EKFState::IncSmallPVR(Vector9d dPVQ)
{

}
void EKFState::IncSmallBias(Vector6d dBias)
{

}

}
