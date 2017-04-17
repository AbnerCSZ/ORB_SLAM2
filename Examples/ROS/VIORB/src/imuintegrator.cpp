#include "imuintegrator.h"

//namespace ORB_SLAM2
//{
using namespace std;
IMUIntegrator::IMUIntegrator()
{
    Initial();

}

IMUIntegrator::IMUIntegrator(const IMUIntegrator& integrator)
{

}

void IMUIntegrator::updateNOv(const Vector3d& omega, const Vector3d& acc, const double& dt)
{
    double dt2 = dt*dt;
    double dt3 = dt*dt*dt;
    _R = _q.toRotationMatrix();

    //TimeUpdate
    TimeUpdate(omega,acc,_R,dt,dt2,dt3);   //get X_minus and P_minus

    //Measurement Update
    MUpdateNo();

    cout<<"_P-"<<_P_minus<<endl;
    cout<<"a:"<<endl<<_R*(acc-_BiasAcc)-_gw<<endl;

}
void IMUIntegrator::updateWithv(const Vector3d& omega, const Vector3d& acc, const double& dt, Vector3d Pcw, Quaterniond qcw)
{
    double dt2 = dt*dt;
    double dt3 = dt*dt*dt;
    _R = _q.toRotationMatrix();

    //TimeUpdate
    TimeUpdate(omega,acc,_R,dt,dt2,dt3);   //get X_minus and P_minus

    //Measurement Update
    MUpdate();
}

void IMUIntegrator::Initial()
{
    _cov_Q_c.setZero();
    _cov_Q_c.block<3,3>(0,0) = IMUData::getAccMeasCov();
    _cov_Q_c.block<3,3>(3,3) = IMUData::getAccBiasRWCov();
    _cov_Q_c.block<3,3>(6,6) = IMUData::getGyrMeasCov();
    _cov_Q_c.block<3,3>(9,9) = IMUData::getGyrBiasRWCov();

    //estimate state
    _P.setZero();         // position
    _V.setZero();         // velocity
    _q.setIdentity();      // rotation w+xi+yj+zk double
    _R.setZero();         // rotation matrix
    // keep unchanged
    _BiasGyr.setZero();   // bias of gyroscope
    _BiasAcc.setZero();   // bias of accelerometer
    // scale
     _scale = 0.0;

    _P_minus.setZero();
    _V_minus.setZero();
    _q_minus.setIdentity();
    _Bg_minus.setZero();
    _Ba_minus.setZero();
    _scale_minus = 1.0;

    _sys_F_d.setIdentity();
    _sys_F_c.setZero();

    _cov_G_c.setZero();
    _cov_M.setIdentity();
    _cov_Q_d.setIdentity();

    _cov_P_minus.setIdentity();      //Pk+1-=Fd*Pk*Fd^T+Qd
    _cov_P.setIdentity();            //Pk+1=(I-Kk+1H)Pk+1-

    _P_cw.setZero();    //Zp=pcw
    _q_cw.setIdentity(); //Zq=qcw

    //measurement error
    _Z_p_e.setZero();   //Zp~=_P_cw-_scale_minus^*_P_minus
    _Z_q_e.setIdentity();//Zq~=_q_cw*(_q_minus*qci)^-1
    _Z_w_e.setZero();   //theta
    _Z_e.setZero();     //z_e=[_Z_p_e, _Z_w_e] 6*1

    _K.setZero(); //Kalman K=P*H^T*S^-1

    _cov_R_m.setZero(); //cov of measurement
    _cov_S.setZero();   //cov of measurement error S=HPH^T+Rm

    _H.setZero();
    _delta_x.setZero();

    _gw<<9.80665,0,0;
    _Rci<<0.0125552670891, -0.999755099723, 0.0182237714554,
            0.999598781151, 0.0130119051815, 0.0251588363115,
           -0.0253898008918, 0.0179005838253, 0.999517347078;
    Quaterniond _qci(_Rci);
    cout<<"IMU Initialed"<<endl;
}
void IMUIntegrator::TimeUpdate(const Vector3d& omega, const Vector3d& acc, Matrix3d _R, double dt, double dt_2, double dt_3)
{
    _P_minus = _P+_V*dt;
    _V_minus = _V+(_R*(acc-_BiasAcc)-_gw)*dt;
    _q_minus = qps(_q*wToq(omega-_BiasGyr,dt),0.5);
    _Bg_minus = _BiasGyr;
    _Ba_minus = _BiasAcc;
    _scale_minus = _scale;

    _sys_F_d.setIdentity();
    _sys_F_c.setZero();
    _sys_F_c.block<3,3>(0,3) = Matrix3d::Identity();
    _sys_F_c.block<3,3>(3,6) = -1*_R*skew(acc-_BiasAcc);
    _sys_F_c.block<3,3>(3,12) = -1*_R;
    _sys_F_c.block<3,3>(6,6) = -1*skew(omega-_BiasGyr);
    _sys_F_c.block<3,3>(6,9) = -1*Matrix3d::Identity();

    _sys_F_d = _sys_F_d + _sys_F_c*dt;          //Fd=I+Fc*dt

    _cov_G_c.setZero();
    _cov_G_c.block<3,3>(3,0) = -1*_R;
    _cov_G_c.block<3,3>(6,6) = -1*Matrix3d::Identity();
    _cov_G_c.block<3,3>(9,9) = Matrix3d::Identity();
    _cov_G_c.block<3,3>(12,3)= Matrix3d::Identity();

    _cov_M = _cov_G_c*_cov_Q_c*_cov_G_c.transpose();
    _cov_Q_d = _cov_M*dt+0.5*(_cov_M*_sys_F_c.transpose()+_sys_F_c*_cov_M)*dt_2+
            _sys_F_c*_cov_M*_sys_F_c.transpose()*dt_3;

    _cov_P_minus = _sys_F_d*_cov_P*_sys_F_d.transpose()+_cov_Q_d;
}
void IMUIntegrator::MUpdate()
{
    _Z_p_e = _P_cw-_scale_minus*_P_minus;
    _Z_q_e = _q_cw*(_q_minus*_qci).inverse();
    _Z_w_e<<2*_Z_q_e.x(),2*_Z_q_e.y(),2*_Z_q_e.z(); //theta
    _Z_e.block<3,1>(0,0) = _Z_p_e;
    _Z_e.block<3,1>(3,0) = _Z_w_e;
    
    _H_p.setZero();
    _H_p.block<3,3>(0,0) = Matrix3d::Identity()*_scale_minus;
    _H_p.block<3,1>(0,15) = _P_minus;

    _H_q.setZero();
    _H_q.block<3,3>(0,3) = _Rci.transpose();

    _H.block<3,16>(0,0) = _H_p;
    _H.block<3,16>(3,0) = _H_q;

    _cov_S = _H*_cov_P_minus*_H.transpose();
    _K = _cov_P_minus*_H.transpose()*_cov_S.inverse();

    _delta_x = _K*_Z_e; //delta_x=[delat p,v,theta,bw,ba,sacle]16*1

    _P = _P_minus+_delta_x.block<3,1>(0,0);
    _V = _V_minus+_delta_x.block<3,1>(3,0);
    //_q =thetaToq(_Z_w_e)*_q_minus;  //
    _BiasGyr = _Bg_minus +_delta_x.block<3,1>(9,0);
    _BiasAcc = _Ba_minus +_delta_x.block<3,1>(12,0);
    _scale = _scale_minus + _delta_x(15,0);
}
void IMUIntegrator::MUpdateNo()
{
    _P = _P_minus;
    _V = _V_minus;
    _q = _q_minus;
    _BiasGyr = _Bg_minus;
    _BiasAcc = _Ba_minus;
    _scale = _scale_minus;

    _cov_P = _cov_P_minus;
}

Quaterniond IMUIntegrator::wToq(const Vector3d w, double _dt) //[0 0.5*w*dt]
{
    Quaterniond _qw;
    _qw.w()=0;
    _qw.x()=0.5*w(0)*_dt;
    _qw.y()=0.5*w(1)*_dt;
    _qw.z()=0.5*w(2)*_dt;
    return _qw;
}
Quaterniond IMUIntegrator::qps(Quaterniond qq, double ss)
{
    qq.x()*=ss;qq.y()*=ss;qq.z()*=ss;qq.w()*=ss;
    return qq;
}
//Quaterniond IMUIntegrator::thetaToq(Vector3d theta)
//{
//    Quaterniond theq;
//    theq.setIdentity();
////    if(theta.squaredNorm()>4)
////    {
////      //theq.w() = sqrt((1-theta.squaredNorm())/4);
////      theq.vec() = theta/2;
////    }
////    else
////    {
////      ////theq.w() = sqrt((1+theta.squaredNorm())/4);
////     // theq.vec() = theta*sqrt((1+theta.squaredNorm())/4)/2;
////    }
//    return theq;
//}

//}
