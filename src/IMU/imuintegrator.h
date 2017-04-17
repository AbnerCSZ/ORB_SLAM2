#ifndef IMUINTEGRATOR
#define IMUINTEGRATOR
#include <Eigen/Dense>

#include "imudata.h"
//#include "ekfstate.h"

namespace ORB_SLAM2
{
using namespace Eigen;
typedef Matrix<double, 9, 9> Matrix9d;
typedef Matrix<double,12,12> Matrix12d;
typedef Matrix<double,16,12> Matrix1612d;
typedef Matrix<double,16,16> Matrix16d;
typedef Matrix<double, 17, 17> Matrix17d;

typedef Matrix<double, 17, 1> Vector17d;
typedef Matrix<double, 16, 1> Vector16d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 4, 1> Vector4d;

class IMUIntegrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUIntegrator();
    IMUIntegrator(const IMUIntegrator& integrator);

    // reset to initial state
    void reset();

    // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
    void update(const Vector3d& omega, const Vector3d& acc, const double& dt);
    void TimeUpdate();
    void Initial();
    Quaterniond wToq(const Vector3d w, double _dt);
    // skew-symmetric matrix v^
    Matrix3d skew(const Vector3d& v)
    {
        Matrix3d skew_v;
        skew_v << 0, -v(2),v(1),
                v(2),   0 ,-v(0),
                -v(1),v(0), 0 ;
        return skew_v;
    }

    // state_minus, position/velocity/q
        inline Eigen::Vector3d getPMinus() const    //
    {
        return _P_minus;
    }
        inline Eigen::Vector3d getVMinus() const    //
    {
        return _V_minus;
    }
        inline Eigen::Quaterniond getRMinus() const   //
    {
        return _q_minus;
    }
        inline double getDeltatime() const
    {
        return _delta_t;
    }
        //q and rotation
        inline Quaterniond normalizeRotationQ(const Quaterniond r)
        {
            Quaterniond _r(r);
            if (_r.w()<0)
            {
                _r.coeffs() *= -1;
            }
            return _r.normalized();
        }

        inline Matrix3d normalizeRotationM (const Matrix3d R)
        {
            Quaterniond qr(R);
            return normalizeRotationQ(qr).toRotationMatrix();
        }

private:
     //estimate state
     Vector3d _P;         // position
     Vector3d _V;         // velocity
     Quaterniond _q;      // rotation w+xi+yj+zk double
     Matrix3d _R;         // rotation matrix
     // keep unchanged
     Vector3d _BiasGyr;   // bias of gyroscope
     Vector3d _BiasAcc;   // bias of accelerometer
     // scale
     double _scale;

     // delta bias
     Vector3d _dBias_g;  // delta bias of gyroscope, correction term computed in optimization
     Vector3d _dBias_a;  // delta bias of accelerometer

     //delta measurements
     Vector3d _P_minus;   //
     Vector3d _V_minus;
     Quaterniond _q_minus;
     Vector3d _Bg_minus;    //gyr
     Vector3d _Ba_minus;    //acc
     double _scale_minus;

     //system matrix
     Matrix16d _sys_F_c;    //Continuous time system matrix
     Matrix16d _sys_F_d;    //discrete time system matrix, Fd=I16+Fc*dt

     // noise covariance propagation of delta measurements
     Matrix12d _cov_Q_c;    //Qc=diag(bias..)
     Matrix1612d _cov_G_c;    //Gc--continuous time cov
     Matrix16d _cov_M;      //M=Gc*Qc*Gc^T
     Matrix16d _cov_Q_d;    //Qd--discrete time cov,Qd=M*dt+0.5*(M*Fc^T+Fc*M)*dt*dt+1/6*(FcMFc^T)dt^3

     //cov P
     Matrix16d _cov_P_minus;      //Pk+1-=Fd*Pk*Fd^T+Qd
     Matrix16d _cov_P;            //Pk+1=(I-Kk+1H)Pk+1-


     //Kalman K

     double _delta_t;

};


}

#endif // IMUINTEGRATOR

