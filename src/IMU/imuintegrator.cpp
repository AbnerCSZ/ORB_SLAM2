#include "imuintegrator.h"

namespace ORB_SLAM2
{

IMUIntegrator::IMUIntegrator()
{

}

IMUIntegrator::IMUIntegrator(const IMUIntegrator& integrator)
{

}

void IMUIntegrator::update(const Vector3d& omega, const Vector3d& acc, const double& dt)
{
    double dt_2 = dt*dt;
    double dt_3 = dt*dt*dt;
    _R = _q.toRotationMatrix();

    //TimeUpdate
    _P_minus = _P+_V*dt;
    _V_minus = _V+_R*acc*dt;
    _q_minus = 0.5*_q*wToq(omega,dt);
    _Bg_minus = _BiasGyr;
    _Ba_minus = _BiasAcc;
    _scale_minus = _scale;

    _sys_F_d.setIdentity();
    _sys_F_c.setZero();
    _sys_F_c.block<3,3>(0,3) = Matrix3d::Identity();
    _sys_F_c.block<3,3>(3,6) = -1*_R*skew(acc);
    _sys_F_c.block<3,3>(3,12) = -1*_R;
    _sys_F_c.block<3,3>(6,6) = -1*skew(omega);
    _sys_F_c.block<3,3>(6,9) = -1*Matrix3d::Identity();

    _sys_F_d = _sys_F_d + _sys_F_c*dt;          //Fd=I+Fc*dt

    _cov_G_c.setZero();
    _cov_G_c.block<3,3>(3,0) = -1*_R;
    _cov_G_c.block<3,3>(6,6) = -1*Matrix3d::Identity();
    _cov_G_c.block<3,3>(9,9) = Matrix3d::Identity();
    _cov_G_c.block<3,3>(12,3)= Matrix3d::Identity();

    _cov_M = _cov_G_c*_cov_Q_c*_cov_G_c.transpose();
    _cov_Q_d = M*dt+0.5*(_cov_M*_sys_F_c.transpose()+_sys_F_c*_cov_M)*dt_2+
            _sys_F_c*_cov_M*_sys_F_c.transpose()*dt_3;

    _cov_P_minus = _sys_F_d*_cov_P*_sys_F_d.transpose()+_cov_Q_d;

    //Mea Update

}

void IMUIntegrator::Initial()
{
    _cov_Q_c.setZero();
    _cov_Q_c.block<3,3>(0,0) = IMUData::getAccMeasCov();
    _cov_Q_c.block<3,3>(3,3) = IMUData::getAccBiasRWCov();
    _cov_Q_c.block<3,3>(6,6) = IMUData::getGyrMeasCov();
    _cov_Q_c.block<3,3>(9,9) = IMUData::getGyrBiasRWCov();
    std::cout<<"Qc:  "<<_cov_Q_c<<std::endl;
}
void IMUIntegrator::TimeUpdate()
{

}

Quaterniond IMUIntegrator::wToq(const Vector3d w, double _dt) //[0 0.5*w*dt]
{
    Quaterniond _qw;
    _qw.w()=0;
    _qw.x()=0.5*w(0)*_dt;
    _qw.y()=0.5*w(1)*_dt;
    _qw.z()=0.5*w(2)*_dt;
    return _wq;
}



}
