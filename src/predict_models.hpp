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
    static const int NX = 20; // number of states

    // Autodiff for state
    template<typename scalar_t>
    using state_t = Eigen::Matrix<scalar_t, NX, 1>;
    using state = state_t<double>;

    typedef Eigen::Matrix<double, NX, NX> state_matrix;


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
        Quaternion<T> omega_quat(0.0, IMU_omega_b(0), IMU_omega_b(1), IMU_omega_b(2));

        Matrix<T, 3, 1> dist_force_inertial;
        dist_force_inertial << x(13),x(14),x(15);

        Matrix<T, 3, 1> dist_torque_inertial;
        dist_torque_inertial << x(16),x(17),x(18);

        // compute total torque in body frame
        Matrix<T, 3, 1> total_torque_body;
        total_torque_body = rot_matrix.transpose()*(dist_torque_inertial) + control_torque_body;
        //total_torque_body =  rot_matrix.transpose()*(dist_torque_inertial);


        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3,3);

        // Speed variation is acceleration
        xdot.segment(3,3) =  rot_matrix*IMU_acc - Vector3d::UnitZ().template cast<T>() *g0 +(dist_force_inertial)/(total_mass);

        // Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
        xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();

        // Angular speed (Euler's rigid dynamical equation)
        xdot.segment(10, 3) = (total_torque_body - IMU_omega_b.cross(I.template cast<T>().cwiseProduct(IMU_omega_b))).cwiseProduct(I_inv.template cast<T>());

        // Disturbance forces static update
        xdot.segment(13, 3) << 0, 0, 0;

        // Disturbance moments static update
        xdot.segment(16, 3) << 0, 0, 0;

        // Barometer bias static update
        xdot(19) = 0;
    }

    void process_noise_cov(state x, state_matrix &Q,Matrix<double, 3, 1> var_gyro,Matrix<double, 3, 1> var_acc,Matrix<double, 3, 1> var_dist_force,Matrix<double, 3, 1> var_dist_torque,double var_baro_bias)
    {
        Q.setZero();
        // Orientation of the rocket with quaternion
        Quaternion<double> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

        //quaternion covariance noise from gyro integration
        Quaternion<double> omega_quat(0.0, var_gyro(0), var_gyro(1), var_gyro(2));
        Matrix<double,4,1> quat_cov = 0.5*(attitude*omega_quat).coeffs();
        Q(6,6) =quat_cov(0);
        Q(7,7) =quat_cov(1);
        Q(8,8) =quat_cov(2);
        Q(9,9) =quat_cov(3);


        //Speed noise from acceleration integration
        Matrix<double,3,1> speed_cov = rot_matrix*var_acc;
        Q(3,3) =speed_cov(0);
        Q(4,4) =speed_cov(1);
        Q(5,5) =speed_cov(2);

        // disturbance force noise
        Q(13,13) =var_dist_force(0);
        Q(14,14) =var_dist_force(1);
        Q(15,15) =var_dist_force(2);

        // disturbance torque noise
        Q(16,16) =var_dist_torque(0);
        Q(17,17) =var_dist_torque(1);
        Q(18,18) =var_dist_torque(2);

        // barometer bias noise
        Q(19,19) = var_baro_bias;

    }
};
