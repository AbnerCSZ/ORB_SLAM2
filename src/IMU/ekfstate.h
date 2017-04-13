#ifndef EKFSTATE
#define EKFSTATE
#include "Eigen/Dense"


namespace ORB_SLAM2
{
using namespace Eigen;
typedef Matrix<double, 9, 9> Matrix9d;
typedef Matrix<double, 17, 17> Matrix17d;
typedef Matrix<double, 17, 1> Vector17d;
typedef Matrix<double, 16, 1> Vector16d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 4, 1> Vector4d;

class EKFState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EKFState();
    EKFState(const EKFState& _ns);


    Vector3d Get_P()        const{return _P;}
    Vector3d Get_V()        const{return _V;}
    Quaterniond Get_qR()    const{return _qR;}
    Matrix3d Get_RotMatrix(){return _qR.toRotationMatrix();}   //quaternion to rotation matrix

    double Get_scale()      const{return _scale;}

    void Set_Pos(const Vector3d &pos){_P = pos;}
    void Set_Vel(const Vector3d &vel){_V = vel;}

    Vector3d Get_BiasGyr() const{return _BiasGyr;}   // bias of gyroscope, keep unchanged after init and during optimization
    Vector3d Get_BiasAcc() const{return _BiasAcc;}   // bias of accelerometer
    void Set_BiasGyr(const Vector3d &bg){_BiasGyr = bg;}
    void Set_BiasAcc(const Vector3d &ba){_BiasAcc = ba;}

    Vector3d Get_dBias_Gyr() const{return _dBias_g;}  // delta bias of gyroscope, init as 0
    Vector3d Get_dBias_Acc() const{return _dBias_a;}  // delta bias of accelerometer
    void Set_DeltaBiasGyr(const Vector3d &dbg){_dBias_g = dbg;}
    void Set_DeltaBiasAcc(const Vector3d &dba){_dBias_a = dba;}

    // incremental addition, delta = [dP, dV, dTheta, dBg, dBa,dS]
    void IncSmall(Vector16d delta);
    void IncSmallPVR(Vector9d dPVQ);
    void IncSmallBias(Vector6d dBias);

    // normalize rotation quaternion. !!! important!!!
    void normalizeRotation(void){_qR = normalizeRotationQ(_qR);}
    //void normalizeRotation(void){_R.normalize();}

        inline Quaterniond normalizeRotationQ(const Quaterniond& r)
        {
            Quaterniond _r(r);
            if (_r.w()<0)
            {
                _r.coeffs() *= -1;
            }
            return _r.normalized();
        }


private:

    Vector3d _P;         // position
    Vector3d _V;         // velocity
    Quaterniond _qR;     // rotation w+xi+yj+zk double


    // keep unchanged during optimization
    Vector3d _BiasGyr;   // bias of gyroscope
    Vector3d _BiasAcc;   // bias of accelerometer

    // scale
    double _scale;

    // update below term during optimization
    Vector3d _dBias_g;  // delta bias of gyroscope, correction term computed in optimization
    Vector3d _dBias_a;  // delta bias of accelerometer

};



}

#endif // EKFSTATE

