#ifndef _KALMAN_H
#define _KALMAN_H
#include <Eigen/Dense>


class KalmanFilter
{
private:
    int stateSize; 			//state variable's dimenssion
    int measSize; 			//measurement variable's dimession
    int uSize; 			//control variables's dimenssion
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P;			//coveriance
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;			//measurement noise covariance
    Eigen::MatrixXd Q;			//process noise covariance
public:
    KalmanFilter(int stateSize_, int measSize_,int uSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    Eigen::VectorXd update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};
 #endif

KalmanFilter::KalmanFilter(int stateSize_ = 0, int measSize_ = 0, int uSize_=0) :stateSize(stateSize_), measSize(measSize_), uSize(uSize_)
{
    x.resize(stateSize);
    x.setZero();

    A.resize(stateSize, stateSize);
    A.setIdentity();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();
}

void KalmanFilter::init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_)
{
    x = x_;
    P = P_;
    R = R_;
    Q = Q_;
}
Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_)
{
    A = A_;
    B = B_;
    u = u_;
    x = A*x + B*u;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q;
    return x;
}

Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_)
{
    A = A_;
    x = A*x;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q; 

    return x;
}

Eigen::VectorXd KalmanFilter::update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas)
{
    H = H_;
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
    temp1 = H*P*Ht + R;
    temp2 = temp1.inverse();
    Eigen::MatrixXd K = P*Ht*temp2;
    z = H*x;
    x = x + K*(z_meas-z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K*H)*P;

    return x;
}


