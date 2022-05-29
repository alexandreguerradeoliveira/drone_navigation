/*
 * Mesurement models used in the EKF node.
 * Alexandre Guerra de Oliveira
 */
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <eigen3/unsupported/Eigen/EulerAngles>

using namespace Eigen;

class MesurementModels{
    public:

    static const int NX = 20; // number of states

    static const int NZBARO = 1;
    static const int NZGPS = 3;
    static const int NZMAG = 3;
    static const int NZGYRO = 3;
    static const int NZOPTITRACK = 3;

    // Autodiff for state
    template<typename scalar_t>
    using state_t = Eigen::Matrix<scalar_t, NX, 1>;

    // Autodiff for barometer
    template<typename scalar_t>
    using sensor_data_baro_t = Eigen::Matrix<scalar_t, NZBARO, 1>;

    // Autodiff for gps
    template<typename scalar_t>
    using sensor_data_gps_t = Eigen::Matrix<scalar_t, NZGPS, 1>;

    // Autodiff for magnetometer
    template<typename scalar_t>
    using sensor_data_mag_t = Eigen::Matrix<scalar_t, NZMAG, 1>;

    // Autodiff for gyroscope
    template<typename scalar_t>
    using sensor_data_gyro_t = Eigen::Matrix<scalar_t, NZGYRO, 1>;

    // Autodiff for optitrack
    template<typename scalar_t>
    using sensor_data_optitrack_t = Eigen::Matrix<scalar_t, NZOPTITRACK, 1>;

    template<typename T>
    void mesurementModelBaro(const state_t<T> &x, sensor_data_baro_t<T> &z) {
        z(0) = x(2) - x(19);

    }

    template<typename T>
    void mesurementModelOptitrack(const state_t<T> &x, sensor_data_optitrack_t<T> &z) {
        z = x.segment(0,3);
    }

    template<typename T>
    void mesurementModelMag(const state_t<T> &x, sensor_data_mag_t<T> &z,Eigen::Matrix<double, 3, 1> mag_vec) {
        // get rotation matrix
        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();
        Eigen::Matrix<T, 3, 1> aa;aa << 0,0,0; // this is only used to get around an NAN bug in Eigen Autodiff

        // express inertial magnetic vector estimate in body-frame and remove bias
        z = rot_matrix.transpose()*(mag_vec)+aa;
    }

    template<typename T>
    void mesurementModelGPS(const state_t<T> &x, sensor_data_gps_t<T> &z) {
        z = x.segment(0,3);
    }


    template<typename T>
    void mesurementModelGyro(const state_t<T> &x, sensor_data_gyro_t<T> &z,Eigen::Matrix<double, 3, 1> gyro_bias) {
        Matrix<T, 3, 1> var_gyro_bias; var_gyro_bias << x(16),x(17),x(18);
        z = x.segment(10,3)-gyro_bias;
    }

};