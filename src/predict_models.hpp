/*
 * Prediction (integration) models used in the EKF node.
 * Alexandre Guerra de Oliveira
 */
//#include "Eigen/Core"
//#include "Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
using namespace Eigen;

class PredictionModels{
    public:
    static const int NX = 26; // number of states
    static const int NW = 19; //number of sources of noise in process covariance

    // Autodiff for state
    template<typename scalar_t>
    using state_t = Eigen::Matrix<scalar_t, NX, 1>;
    using state = state_t<double>;

    // Autodiff for noise
    template<typename scalar_t>
    using noise_t = Eigen::Matrix<scalar_t, NW, 1>;


    template<typename T>
    void state_dynamics(state_t<T> x, state_t<T> &xdot,Matrix<double, 3, 1> IMU_omega_b,Matrix<double, 3, 1> IMU_acc,Matrix<double, 3, 1> control_torque_body,double total_mass,Matrix<double, 3, 1> I,Matrix<double, 3, 1> I_inv)
    {
        // -------------- Simulation variables -----------------------------
        T g0 = (T) 9.81;  // Earth gravity in [m/s^2]

        // Orientation of the rocket with quaternion
        Quaternion<T> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Angular velocity omega in quaternion format to compute quaternion derivative

        Matrix<T, 3, 1> var_gyro_bias;
        var_gyro_bias << x(16),x(17),x(18);

        Matrix<T, 3, 1> var_acc_bias;
        var_acc_bias << x(13),x(14),x(15);

        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3,3);

        // Speed variation is acceleration
        xdot.segment(3,3) =  rot_matrix*(IMU_acc - var_acc_bias)-Eigen::Vector3d::UnitZ()*g0;

        // Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
        Quaternion<T> omega_quat(0.0, IMU_omega_b(0)-var_gyro_bias(0), IMU_omega_b(1)-var_gyro_bias(1), IMU_omega_b(2)-var_gyro_bias(2));
        xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();

        // Angular speed static update
        xdot.segment(10, 3) << 0.0,0.0,0.0;

        // acc bias static update
        xdot.segment(13, 3) << 0, 0, 0;

        // gyro bias static update
        xdot.segment(16, 3) << 0, 0, 0;

        // Barometer bias static update
        xdot(19) = 0;

        // Magnetic field vector in inertial frame: static update
        xdot.segment(20, 3) << 0, 0, 0;

        // magnetometer bias static update
        xdot.segment(23, 3) << 0, 0, 0;

    }

    template<typename T>
    void noise_dynamics(noise_t<T> w, state_t<T> &xdot ,const state x)
    {
        xdot.setZero();

        // put variance from sensor in vector form
        Matrix<T,3,1> var_acc;var_acc << w(0),w(1),w(2);
        Matrix<T,3,1> var_gyro;var_gyro << w(3),w(4),w(5);

        Matrix<T,3,1> var_acc_bias;var_acc_bias << w(6),w(7),w(8);
        Matrix<T,3,1> var_gyro_bias;var_gyro_bias <<w(9),w(10),w(11);
        Matrix<T,1,1> var_baro;var_baro << w(12);
        Matrix<T,3,1> var_mag_field;var_mag_field <<w(13),w(14),w(15);
        Matrix<T,3,1> var_mag_bias;var_mag_bias <<w(16),w(17),w(18);


        // Orientation of the rocket with quaternion
        Quaternion<T> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        //quaternion covariance noise from gyro integration
        Quaternion<T> omega_quat(0.0, var_gyro(0), var_gyro(1), var_gyro(2));

        // noise propagation in predict_function
        xdot.head(3) << 0.0,0.0,0.0;
        //Speed noise from acceleration integration
        xdot.segment(3, 3) = rot_matrix*var_acc;

        // Quaternion noise from gyro bias propagation in kinematic equation
        xdot.segment(6, 4) = 0.5*(attitude*omega_quat).coeffs();

        // gyro noise
        xdot.segment(10, 3) =var_gyro;

        // accelerometer bias noise
        xdot.segment(13,3) = var_acc_bias;
        // gyroscope bias noise
        xdot.segment(16,3) = var_gyro_bias;

        // barometer bias noise
        xdot.segment(19,1) =var_baro;

        // magnetic field noise
        xdot.segment(20,3) = var_mag_field;

        // magnetic bias noise
        xdot.segment(23,3) = var_mag_bias;
    }
};
