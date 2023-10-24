#include <Eigen/Dense>

class KalmanFilter
{
private:
    int stateSize;            // Dimension of state variables
    int measSize;             // Dimension of measurement variables
    int uSize;                // Dimension of control variables
    Eigen::VectorXd x;        // State vector
    Eigen::VectorXd z;        // Measurement vector
    Eigen::MatrixXd A;        // State transition matrix
    Eigen::MatrixXd B;        // Control input matrix
    Eigen::VectorXd u;        // Control input vector
    Eigen::MatrixXd P;        // Covariance matrix
    Eigen::MatrixXd H;        // Measurement matrix
    Eigen::MatrixXd R;        // Measurement noise covariance matrix
    Eigen::MatrixXd Q;        // Process noise covariance matrix

public:
    KalmanFilter(int stateSize_ = 0, int measSize_ = 0, int uSize_ = 0);
    ~KalmanFilter() {}

    // Initialize the Kalman filter with initial state, covariance, measurement noise, and process noise.
    void init(const Eigen::VectorXd &x_, const Eigen::MatrixXd &P_, const Eigen::MatrixXd &R_, const Eigen::MatrixXd &Q_);

    // Predict the next state based on state transition matrix A, control input matrix B, and control input vector u.
    Eigen::VectorXd predict(const Eigen::MatrixXd &A_);
    
    // Predict the next state based on state transition matrix A only.
    Eigen::VectorXd predict(const Eigen::MatrixXd &A_, const Eigen::MatrixXd &B_, const Eigen::VectorXd &u_);

    // Update the state using the measurement matrix H and the measured data z_meas.
    Eigen::VectorXd update(const Eigen::MatrixXd &H_, const Eigen::VectorXd &z_meas);
};

KalmanFilter::KalmanFilter(int stateSize_, int measSize_, int uSize_)
    : stateSize(stateSize_), measSize(measSize_), uSize(uSize_)
{
    x = Eigen::VectorXd::Zero(stateSize);
    A = Eigen::MatrixXd::Identity(stateSize, stateSize);
    u = Eigen::VectorXd::Zero(uSize);
    B = Eigen::MatrixXd::Zero(stateSize, uSize);
    P = Eigen::MatrixXd::Identity(stateSize, stateSize);
    H = Eigen::MatrixXd::Zero(measSize, stateSize);
    z = Eigen::VectorXd::Zero(measSize);
    Q = Eigen::MatrixXd::Zero(stateSize, stateSize);
    R = Eigen::MatrixXd::Zero(measSize, measSize);
}

void KalmanFilter::init(const Eigen::VectorXd &x_, const Eigen::MatrixXd &P_, const Eigen::MatrixXd &R_, const Eigen::MatrixXd &Q_)
{
    x = x_;  // Set the initial state
    P = P_;  // Set the initial covariance matrix
    R = R_;  // Set the measurement noise covariance matrix
    Q = Q_;  // Set the process noise covariance matrix
}

Eigen::VectorXd KalmanFilter::predict(const Eigen::MatrixXd &A_, const Eigen::MatrixXd &B_, const Eigen::VectorXd &u_)
{
    A = A_;
    B = B_;
    u = u_;
    
    // Predict the next state based on control input
    x = A * x + B * u;
    Eigen::MatrixXd A_T = A.transpose();
    
    // Update the covariance matrix
    P = A * P * A_T + Q;
    
    return x;
}

Eigen::VectorXd KalmanFilter::predict(const Eigen::MatrixXd &A_)
{
    A = A_;
    
    // Predict the next state without control input
    x = A * x;
    Eigen::MatrixXd A_T = A.transpose();
    
    // Update the covariance matrix
    P = A * P * A_T + Q;
    
    return x;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::MatrixXd &H_, const Eigen::VectorXd &z_meas)
{
    H = H_;
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    
    // Calculate Kalman gain
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse();
    Eigen::MatrixXd K = P * Ht * temp2;
    
    z = H * x;  // Predicted measurement based on current state
    
    // Update the state based on measurement and Kalman gain
    x = x + K * (z_meas - z);
    
    // Update the covariance matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K * H) * P;
    
    return x;
}
