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
    static const int NX = 19; // number of states

    // Autodiff for state
    template<typename scalar_t>
    using state_t = Eigen::Matrix<scalar_t, NX, 1>;

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

        // Angular speed
        xdot.segment(10, 3) = (total_torque_body - IMU_omega_b.cross(I.template cast<T>().cwiseProduct(IMU_omega_b))).cwiseProduct(I_inv.template cast<T>());


        // Disturbance forces
        xdot.segment(13, 3) << 0, 0, 0;

        // Disturbance moments
        xdot.segment(16, 3) << 0, 0, 0;

    }

    //    template<typename T>
//    void state_dynamics_forcemodel(state_t<T> x, state_t<T> &xdot)
//    {
//        // -------------- Simulation variables -----------------------------
//        T g0 = (T) 9.81;  // Earth gravity in [m/s^2]
//
//        Eigen::Matrix<T, 3, 1> control_force;
//        control_force << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;
//
//        Eigen::Matrix<T, 3, 1> control_torque;
//        control_torque << rocket_control.torque.x, rocket_control.torque.y, rocket_control.torque.z;
//
//        // Orientation of the rocket with quaternion
//        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
//        attitude.normalize();
//        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();
//
//        Eigen::Matrix<T, 3, 1> dist_force;
//        dist_force << x(14),x(15),x(16);
//
//        Eigen::Matrix<T, 3, 1> dist_torque;
//        dist_torque << x(17),x(18),x(19);
//
//        // compute total force inertial frame
//        Eigen::Matrix<T, 3, 1> total_force_inertial;
//        total_force_inertial = rot_matrix*(control_force) - Eigen::Vector3d::UnitZ().template cast<T>()*(total_mass)*g0 + dist_force;
//
//        // compute total torque in body frame
//        Eigen::Matrix<T, 3, 1> total_torque_body;
//        total_torque_body = control_torque + rot_matrix.transpose()*dist_torque;
//
//        Eigen::Matrix<T, 3, 1> omega;
//        omega << x(10),x(11),x(12);
//        omega = rot_matrix.transpose()*omega;
//
//        // Angular velocity omega in quaternion format to compute quaternion derivative
//        Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));
//
//        //Inertia
//        Matrix<T, 3, 1> I_inv;
//        I_inv << 1 / rocket.total_Inertia[0], 1 / rocket.total_Inertia[1], 1 / rocket.total_Inertia[2];
//        Matrix<T, 3, 1> I;
//        I << rocket.total_Inertia[0], rocket.total_Inertia[1],rocket.total_Inertia[2];
//
//        // -------------- Differential equation ---------------------
//
//        // Position variation is speed
//        xdot.head(3) = x.segment(3,3);
//
//        // Speed variation is acceleration given by newton's law
//        xdot.segment(3,3) =  total_force_inertial/(x(13)+dry_mass);
//
//        // Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
//        xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();
//
//        // Angular speed variation is given by euler's equation if in body frame
//        xdot.segment(10, 3) = rot_matrix*(total_torque_body - omega.cross(I.template cast<T>().cwiseProduct(omega))).cwiseProduct(I_inv.template cast<T>());
//
//        // no mass variation
//        xdot(13) = 0;
//
//        // Disturbance forces
//        xdot.segment(14, 3) << 0, 0, 0;
//
//        // Disturbance moments
//        xdot.segment(17, 3) << 0, 0, 0;
//    }
//




};
