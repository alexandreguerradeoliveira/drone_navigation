/*
* Node to estimate the rocket full state (position, velocity, attitude quaternion, angular rate and mass) 
* from the sensor data and commanded thrust and torque of the rocket engine
*
*
* Parameters:
*   - Rocket model: 		/config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
*   - EKF parameters:       /config/ekf_parameters.yaml

* Outputs:
*   - Complete estimated state : /kalman_rocket_state
 *  - Disturbance forces and torque in inertial frame: /kalman_disturbance
*
*/

#include "ros/ros.h"

#include "rocket_utils/FSM.h"
#include "rocket_utils/State.h"
#include "rocket_utils/ExtendedState.h"
#include "rocket_utils/ControlMomentGyro.h"
#include "rocket_utils/Control.h"
#include "rocket_utils/Sensor.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"


#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/FluidPressure.h"


#include <iostream>
#include <chrono>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <eigen3/unsupported/Eigen/EulerAngles>

#include "rocket_model.hpp"
#include "mesurement_models.hpp"
#include "predict_models.hpp"



#include <random>
#include <autodiff/AutoDiffScalar.h>
using namespace Eigen;

#define DEG2RAD 0.01745329251

class NavigationNode {
	public:
        double dt_ros = 0.05; // [s] time delay between two publishes of the message constaining the kalman state (/kalman_rocket_state)
        bool is_simulation; // 1=runs in simulation (SIL), 0=runs on drone
        bool use_gps; // 0=use optitrack, 1=use px4 gps
        bool predict_on_idle;// 1:navigation runs on idle state, 0:navigation does not run on idle state
        bool use_magnetometer; // 0=do not use px4 magnetometer, 1=use px4 magnetometer
        bool use_barometer; // 0=do not use px4 barometer, 1=use px4 barometer

        static const int NX = 26; // number of states in the EKF
        static const int NW = 19; //number of sources of noise in process covariance

        // observation model dimentions
		static const int NZBARO = 1;
        static const int NZGPS = 3;
        static const int NZMAG = 3;
        static const int NZGYRO = 3;
        static const int NZOPTITRACK = 3;

        /// Autodiff variables
        // Autodiff for state
    	template<typename scalar_t>
    	using state_t = Eigen::Matrix<scalar_t, NX, 1>;
    	using state = state_t<double>;
    	using ad_state = state_t<AutoDiffScalar<state>>;// Autodiff state variable

        // Autodiff for noise
        template<typename scalar_t>
        using noise_t = Eigen::Matrix<scalar_t, NW, 1>;
        using noise = noise_t<double>;
        using ad_noise = noise_t<AutoDiffScalar<noise>>;// Autodiff noise variable
        using ad_state_noise = state_t<AutoDiffScalar<noise>>;// Autodiff state variable (used when we derivate a vector with the size of state but with input a vector with size of noise)


        // Autodiff for barometer
    	template<typename scalar_t>
    	using sensor_data_baro_t = Eigen::Matrix<scalar_t, NZBARO, 1>;
  		using sensor_data_baro = sensor_data_baro_t<double>;
    	using ad_sensor_data_baro = sensor_data_baro_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable barrometer

        // Autodiff for gps
        template<typename scalar_t>
        using sensor_data_gps_t = Eigen::Matrix<scalar_t, NZGPS, 1>;
        using sensor_data_gps = sensor_data_gps_t<double>;
        using ad_sensor_data_gps = sensor_data_gps_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable GPS

    // Autodiff for magnetometer
        template<typename scalar_t>
        using sensor_data_mag_t = Eigen::Matrix<scalar_t, NZMAG, 1>;
        using sensor_data_mag = sensor_data_mag_t<double>;
        using ad_sensor_data_mag = sensor_data_mag_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable magnetometer

        // Autodiff for gyroscope
        template<typename scalar_t>
        using sensor_data_gyro_t = Eigen::Matrix<scalar_t, NZGYRO, 1>;
        using sensor_data_gyro = sensor_data_gyro_t<double>;
        using ad_sensor_data_gyro = sensor_data_gyro_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable gyroscope


        // Autodiff for optitrack
        template<typename scalar_t>
        using sensor_data_optitrack_t = Eigen::Matrix<scalar_t, NZOPTITRACK, 1>;
        using sensor_data_optitrack = sensor_data_optitrack_t<double>;
        using ad_sensor_data_optitrack = sensor_data_optitrack_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable optitrack


        // Sensor mesurement model matrices
    	typedef Eigen::Matrix<double, NX, NX> state_matrix;
    	typedef Eigen::Matrix<double, NZBARO, NZBARO> sensor_matrix_baro;
        typedef Eigen::Matrix<double, NZGPS, NZGPS> sensor_matrix_gps;
        typedef Eigen::Matrix<double, NZMAG, NZMAG> sensor_matrix_mag;
        typedef Eigen::Matrix<double, NZGYRO, NZGYRO> sensor_matrix_gyro;
        typedef Eigen::Matrix<double, NZOPTITRACK, NZOPTITRACK> sensor_matrix_optitrack;

        typedef Eigen::Matrix<double, NW, NW> covariance_input; // covariance matrix of noise in prediction


    private:

        MesurementModels *mesurementModels;
        PredictionModels *predictionModels;

        sensor_data_baro z_baro;
        sensor_data_mag mag_data;
    sensor_data_mag mag_data_normalized;

    sensor_data_mag raw_mag;
        sensor_data_gyro gyro_data;
        sensor_data_optitrack optitrack_data;
        sensor_data_optitrack optitrack_data0; // initial position for optitrack

        // Class with useful rocket parameters and methods
		Rocket rocket;

		// Last received fsm
		rocket_utils::FSM rocket_fsm;

		// Last received control
		rocket_utils::Control rocket_control;

		// Last received sensor data
		rocket_utils::Sensor rocket_sensor;

		// List of subscribers and publishers
		ros::Publisher state_pub;
        ros::Publisher extended_state_pub;

        ros::Subscriber fsm_sub;
		ros::Subscriber control_sub;
		ros::Subscriber rocket_state_sub;
		ros::Subscriber sensor_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber optitrack_sub;
        ros::Subscriber px4_imu_sub;
        ros::Subscriber px4_mag_sub;
        ros::Subscriber px4_gps_sub;
        ros::Subscriber px4_baro_sub;

        // Kalman matrix
        sensor_matrix_baro R_baro;
        sensor_matrix_gps R_gps;
        sensor_matrix_mag R_mag;
        sensor_matrix_gyro R_gyro;
        sensor_matrix_optitrack R_optitrack;

    /// EKF matrices

        // Predition matrices
        state_matrix F;
		state_matrix Q;
		state_matrix P;

        // mesurement model matrices
        Matrix<double,NZBARO,NX> H_baro; // computed using autodiff
        Matrix<double,NZGPS,NX> H_gps; // computed using autodiff
        Matrix<double,NZMAG,NX> H_mag; // computed using autodiff
        Matrix<double,NZGYRO,NX> H_gyro; // computed using autodiff
        Matrix<double,NZOPTITRACK,NX> H_optitrack; // computed using autodiff

        // matrices to calculate the covariance of the process noise
        Matrix<double,NX,NW> G_noise; // computed using autodiff
        Matrix<double,NW,NW> W_noise; // matrix with noise from sensors

        // Kalman state
		state X ;
		ad_state ADx; // state used for autodiff

        //Kalman noise
        noise W ;
        ad_noise ADw; // state used for autodiff

        // Fake GPS params
        double gps_freq = 10; //fake gps prequency
        double gps_noise = 0.8;

        // px4 gps variables
        double kx = 0;
        double ky = 0;
        double gps_latitude0 = 0;
        double gps_longitude0 = 0;
        double gps_alt0 = 0;
        double gps_latitude = 0;
        double gps_longitude = 0;
        double gps_alt = 0;

        // px4 barometer variables
        double pressure0 = 0.0; // pressure of barometer at the origin
        double pressure = 0.0; // pressure of barometer at the origin

        //Inertia
        Matrix<double, 3, 1> I_inv;
        Matrix<double, 3, 1> I;

        //mass of vehicle
        double total_mass = 1.0;
        double dry_mass = 1.0;

        // Current torque from Actuator
        Matrix<double, 3, 1> control_torque_body;

        // Current acceleration and angular rate from IMU
        Matrix<double, 3, 1> IMU_omega_b;
        Matrix<double, 3, 1> IMU_acc;

        // GPS x,y position
        Eigen::Matrix<double, 3, 1> gps_pos;

        // calibration params

        Matrix<double,3,1> acc_bias;
        Matrix<double,3,1> gyro_bias;
        Matrix<double,3,1> mag_bias;

        Matrix<double,3,3> A_mag_calibration;
        Matrix<double,3,1> b_mag_calibration;

        double gps_latitude_sum = 0.0;
        double gps_longitude_sum = 0.0;
        double gps_alt_sum = 0.0;

        double baro_pressure_sum = 0.0;
        Matrix<double,3,1> sum_acc;
        Matrix<double,3,1> sum_gyro;

        Matrix<double,3,1> mag_vec_inertial;
        Matrix<double,3,1> mag_vec_inertial_normalised;

        Matrix<double,3,1> mag_vec_inertial_sum;
        double mag_declination = 0.0;

        int gps_homing_counter = 0;
        int pressure_homing_counter = 0;
        int mag_homing_counter = 0;

        int imu_calibration_counter = 0;
        int imu_calibration_counter_calibrated = 1000;

        int gps_started = 0; // set to 1 once gps is ready
        int optitrack_started = 0; // set to 1 once optitrack is ready
        bool imu_calibrated = true; // 1:imu calibrated, 0:imu not calibrated (imu=gyroscope and accelerometer)


public:
		NavigationNode(ros::NodeHandle nh)
		{
            // Init autodiff derivative variable X
            ADx(X);
            int div_size = ADx.size();
            int derivative_idx = 0;
            for (int i = 0; i < ADx.size(); ++i) {
                ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
                derivative_idx++;
            }

            // Init autodiff  derivative variable W
            ADw(W);
            int div_size_w = ADw.size();
            int derivative_idw = 0;
            for (int i = 0; i < ADw.size(); ++i) {
                ADw(i).derivatives() = noise::Unit(div_size_w, derivative_idw);
                derivative_idw++;
            }


            /// EKF settings initiation
            nh.param<bool>("/navigation/is_simulation", is_simulation, true);
            nh.param<bool>("/navigation/use_gps", use_gps, true);
            nh.param<bool>("/navigation/use_magnetometer", use_magnetometer, false);
            nh.param<bool>("/navigation/use_barometer", use_barometer, true);
            nh.param<bool>("/navigation/predict_on_idle", predict_on_idle, true);
            nh.param<double>("/navigation/dt_state_publish", dt_ros, 0.05);

            nh.param<int>("/navigation/imu_calibration_number_of_samples", imu_calibration_counter_calibrated, 1000);
            nh.param<double>("/navigation/mag_declination", mag_declination, 0.0);

            // fake gps initiation
            nh.param<double>("/navigation/fake_gps_freq", gps_freq, 10);
            nh.param<double>("/navigation/fake_gps_variance", gps_noise, 10);

            // sensor covariance initiation
            R_baro.setIdentity();
            nh.param<double>("/navigation/R_baro", R_baro(0), 0.0000001);

            R_mag.setZero();
            nh.param<double>("/navigation/R_mag_x", R_mag(0,0), 0.0000001);
            nh.param<double>("/navigation/R_mag_y", R_mag(1,1), 0.0000001);
            nh.param<double>("/navigation/R_mag_z", R_mag(2,2), 0.0000001);


            R_optitrack.setIdentity();
            nh.param<double>("/navigation/R_optitrack_x", R_optitrack(0,0), 0.000000001);
            nh.param<double>("/navigation/R_optitrack_y", R_optitrack(1,1), 0.000000001);
            nh.param<double>("/navigation/R_optitrack_z", R_optitrack(2,2), 0.000000001);


            R_gps.setIdentity();
            nh.param<double>("/navigation/R_gps_x", R_gps(0,0), 0.000064);
            nh.param<double>("/navigation/R_gps_y", R_gps(1,1), 0.000064);
            nh.param<double>("/navigation/R_gps_z", R_gps(2,2), 0.000064);


            R_gyro.setIdentity();
            nh.param<double>("/navigation/R_gyro_x", R_gyro(0,0), 0.0001);
            nh.param<double>("/navigation/R_gyro_y", R_gyro(1,1), 0.0001);
            nh.param<double>("/navigation/R_gyro_z", R_gyro(2,2), 0.0001);

            // process covariance
            W_noise.setIdentity();
            nh.param<double>("/navigation/W_gyro_x", W_noise(0,0), 0.0000005);
            nh.param<double>("/navigation/W_gyro_y", W_noise(1,1), 0.0000005);
            nh.param<double>("/navigation/W_gyro_z", W_noise(2,2), 0.0000005);

            nh.param<double>("/navigation/W_acc_x", W_noise(3,3), 0.0000005);
            nh.param<double>("/navigation/W_acc_y", W_noise(4,4), 0.0000005);
            nh.param<double>("/navigation/W_acc_z", W_noise(5,5), 0.0000005);

            nh.param<double>("/navigation/W_acc_bias_x", W_noise(6,6), 0.0000005);
            nh.param<double>("/navigation/W_acc_bias_y", W_noise(7,7), 0.0000005);
            nh.param<double>("/navigation/W_acc_bias_z", W_noise(8,8), 0.0000005);

            nh.param<double>("/navigation/W_gyro_bias_x", W_noise(9,9), 0.0000005);
            nh.param<double>("/navigation/W_gyro_bias_y", W_noise(10,10), 0.0000005);
            nh.param<double>("/navigation/W_gyro_bias_z", W_noise(11,11), 0.0000005);

            nh.param<double>("/navigation/W_baro_bias", W_noise(12,12), 1);

            nh.param<double>("/navigation/W_mag_x", W_noise(13,13), 0.00001);
            nh.param<double>("/navigation/W_mag_y", W_noise(14,14), 0.00001);
            nh.param<double>("/navigation/W_mag_z", W_noise(15,15), 0.00001);

            nh.param<double>("/navigation/W_mag_bias_x", W_noise(16,16), 0.00001);
            nh.param<double>("/navigation/W_mag_bias_y", W_noise(17,17), 0.00001);
            nh.param<double>("/navigation/W_mag_bias_z", W_noise(18,18), 0.00001);

            // sensor bias
            nh.param<double>("/navigation/gyro_bias_x", gyro_bias(0),0.0);
            nh.param<double>("/navigation/gyro_bias_y", gyro_bias(1),0.0);
            nh.param<double>("/navigation/gyro_bias_z", gyro_bias(2),0.0);

            nh.param<double>("/navigation/acc_bias_x", acc_bias(0),0.0);
            nh.param<double>("/navigation/acc_bias_y", acc_bias(1),0.0);
            nh.param<double>("/navigation/acc_bias_z", acc_bias(2),0.0);

            nh.param<double>("/navigation/mag_bias_x", mag_bias(0),0.0);
            nh.param<double>("/navigation/mag_bias_y", mag_bias(1),0.0);
            nh.param<double>("/navigation/mag_bias_z", mag_bias(2),0.0);


            /// magnetometer calibration matrices
            A_mag_calibration.setIdentity();
            nh.param<double>("/navigation/A_mag_00", A_mag_calibration(0,0),1.0);
            nh.param<double>("/navigation/A_mag_01", A_mag_calibration(0,1),0.0);
            nh.param<double>("/navigation/A_mag_02", A_mag_calibration(0,2),0.0);
            nh.param<double>("/navigation/A_mag_10", A_mag_calibration(1,0),0.0);
            nh.param<double>("/navigation/A_mag_11", A_mag_calibration(1,1),1.0);
            nh.param<double>("/navigation/A_mag_12", A_mag_calibration(1,2),0.0);
            nh.param<double>("/navigation/A_mag_20", A_mag_calibration(2,0),0.0);
            nh.param<double>("/navigation/A_mag_21", A_mag_calibration(2,1),0.0);
            nh.param<double>("/navigation/A_mag_22", A_mag_calibration(2,2),1.0);

            b_mag_calibration.setZero();
            nh.param<double>("/navigation/b_mag_0", b_mag_calibration(0),0.0);
            nh.param<double>("/navigation/b_mag_1", b_mag_calibration(1),0.0);
            nh.param<double>("/navigation/b_mag_2", b_mag_calibration(2),0.0);

			// Initialize publishers and subscribers
        	initTopics(nh);

			// Initialize rocket class with useful parameters
			rocket.init(nh);

			//Get initial orientation and convert in Radians
			float roll = 0, zenith = 0, azimuth = 0.0;
			nh.getParam("/environment/rocket_roll", roll);
			nh.getParam("/environment/rail_zenith", zenith);
			nh.getParam("/environment/rail_azimuth", azimuth);
            nh.getParam("/rocket/dry_mass", dry_mass);

            roll *= DEG2RAD; zenith *= DEG2RAD; azimuth *= DEG2RAD;

			typedef Eigen::EulerSystem<-Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_Z> Rail_system;
			typedef Eigen::EulerAngles<double, Rail_system> angle_type;

			angle_type init_angle(azimuth, zenith, roll);

			Eigen::Quaterniond q(init_angle);

            rocket_fsm.state_machine = rocket_utils::FSM::CALIBRATION;
            P.setZero(); // no error in the initial state

            z_baro.setZero();
            mag_vec_inertial << 1.0,0.0,0.0; // this is replaced by the sensor mesured velue if nor runing in simulation
            mag_vec_inertial_sum << 0.0,0.0,0.0;
            mag_vec_inertial_normalised << 1.0,0.0,0.0;
            sum_acc << 0,0,0;
            sum_gyro << 0,0,0;
            mag_data_normalized << 0.0,0.0,0.0;


            // initiate sensor variables
            rocket_sensor.IMU_acc.x = 0;
            rocket_sensor.IMU_acc.y = 0;
            rocket_sensor.IMU_acc.z = 0;
            rocket_sensor.IMU_gyro.x = 0;
            rocket_sensor.IMU_gyro.y = 0;
            rocket_sensor.IMU_gyro.z = 0;
            rocket_sensor.IMU_mag.x = 0;
            rocket_sensor.IMU_mag.y = 0;
            rocket_sensor.IMU_mag.z = 0;
            rocket_sensor.baro_height = 0;

            rocket_control.torque.x = 0;
            rocket_control.torque.y = 0;
            rocket_control.torque.z = 0;
            rocket_control.force.x = 0;
            rocket_control.force.y = 0;
            rocket_control.force.z = 0;


            /// Init state X
            total_mass = rocket.propellant_mass+dry_mass;
            X.setZero();
            X.segment(6,4) = q.coeffs();
            X.segment(20,3) = mag_vec_inertial; // if we are not in simulation, this gets replaced by the sensor estimation

            W.setZero();
        }

		void initTopics(ros::NodeHandle &nh)
		{

            if(is_simulation){
                // Subscribe to sensor for kalman correction
                sensor_sub = nh.subscribe("/simu_sensor_pub", 1, &NavigationNode::sensorCallback, this);

                // Subscribe to state to fake GPS
                gps_sub = nh.subscribe("/rocket_state", 1, &NavigationNode::gpsCallback, this);
            }else{

                px4_imu_sub = nh.subscribe("/mavros/imu/data_raw", 1, &NavigationNode::px4imuCallback, this);

                    px4_mag_sub = nh.subscribe("/mavros/imu/mag", 1, &NavigationNode::px4magCallback, this);

                if(use_barometer){
                    px4_baro_sub = nh.subscribe("/mavros/imu/static_pressure", 1, &NavigationNode::px4baroCallback, this);
                }

                if(use_gps){
                    px4_gps_sub = nh.subscribe("/mavros/global_position/raw/fix", 1, &NavigationNode::px4gpsCallback, this);
                }else{
                    optitrack_sub = nh.subscribe("/optitrack_client/Drone/optitrack_pose", 1, &NavigationNode::optitrackCallback, this);
                }

            }


			// Create filtered rocket state publisher
			//nav_pub = nh.advertise<rocket_utils::State>("/kalman_rocket_state", 10);

            // Create process covariance matrix diagonal publisher
            //cov_pub = nh.advertise<rocket_utils::StateCovariance>("/process_cov", 10);
            state_pub = nh.advertise<rocket_utils::State>("/kalman_rocket_state", 10);
            extended_state_pub = nh.advertise<rocket_utils::ExtendedState>("/extended_kalman_rocket_state", 10);


            // Create publisher for imu bias
            //bias_pub = nh.advertise<geometry_msgs::Wrench>("/kalman_bias", 10);

			// Subscribe to time_keeper for fsm and time
			fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &NavigationNode::fsmCallback, this);


		}

		/* ------------ Callbacks functions ------------ */

		// Callback function to store last received fsm
		void fsmCallback(const rocket_utils::FSM::ConstPtr& fsm)
		{
			rocket_fsm.state_machine = fsm->state_machine;
		}

        void px4imuCallback(const sensor_msgs::Imu::ConstPtr& sensor)
        {
            rocket_sensor.IMU_acc = sensor->linear_acceleration;
            rocket_sensor.IMU_gyro = sensor->angular_velocity;

            if(rocket_fsm.state_machine == rocket_utils::FSM::CALIBRATION)
            {
                calibrate_imu();
            }

            if(rocket_fsm.state_machine == rocket_utils::FSM::IDLE)
            {
                if(predict_on_idle){
                    predict_step();
                }
            }

            if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
               && rocket_fsm.state_machine != rocket_utils::FSM::IDLE)
            {
                predict_step();
            }

        }

        void px4magCallback(const sensor_msgs::MagneticField::ConstPtr& sensor)
        {
            rocket_sensor.IMU_mag = sensor->magnetic_field;
            raw_mag << rocket_sensor.IMU_mag.x,rocket_sensor.IMU_mag.y,rocket_sensor.IMU_mag.z; // raw data from the sensor
            mag_data = A_mag_calibration*(raw_mag-b_mag_calibration); // calibrated data

            mag_data_normalized = mag_data-mag_bias;
            mag_data_normalized.normalize();

            if(use_gps){
                if(rocket_fsm.state_machine == rocket_utils::FSM::CALIBRATION)
                {
                    homing_mag();
                }

                if(use_magnetometer){
                    if(imu_calibrated==1){
                        if(rocket_fsm.state_machine == rocket_utils::FSM::IDLE)
                        {
                            if(predict_on_idle){
                                if((!mag_data_normalized.hasNaN())&&(!mag_vec_inertial_normalised.hasNaN())){
                                    predict_step();
                                    update_step_mag(mag_data_normalized);
                                }
                            }
                        }

                        if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
                           && rocket_fsm.state_machine != rocket_utils::FSM::IDLE)
                        {
                            if((!mag_data_normalized.hasNaN())&&(!mag_vec_inertial_normalised.hasNaN())){
                                predict_step();
                                update_step_mag(mag_data_normalized);
                            }
                        }
                    }
                }
            }



        }

        void px4baroCallback(const sensor_msgs::FluidPressure::ConstPtr& sensor){
            double g0 = 9.81;
            double rho0 = 1.225;
            pressure = sensor->fluid_pressure;
            z_baro(0) =  (pressure0 - pressure)/(rho0*g0);


            if(rocket_fsm.state_machine == rocket_utils::FSM::CALIBRATION)
            {
                homing_baro();
            }

            if(imu_calibrated==1){
                if(rocket_fsm.state_machine == rocket_utils::FSM::IDLE)
                {
                    if(predict_on_idle){
                        if(!z_baro.hasNaN()){
                            predict_step();
                            update_step_baro(z_baro);
                        }
                    }
                }

                if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
                   && rocket_fsm.state_machine != rocket_utils::FSM::IDLE)
                {
                    if(!z_baro.hasNaN()){
                        predict_step();
                        update_step_baro(z_baro);
                    }
                }
            }



        }

        void px4gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps)
        {
            gps_latitude = gps->latitude;
            gps_longitude = gps->longitude;
            gps_alt = gps->altitude;

            // get sensor covariance diagonal from PX4
            R_gps(0,0) = gps->position_covariance[0];
            R_gps(1,1) = gps->position_covariance[4];
            R_gps(2,2) = gps->position_covariance[8];


            if(rocket_fsm.state_machine == rocket_utils::FSM::CALIBRATION || gps_started==0)
            {
                homing_gps();
            }

            double gps_x = (gps_latitude-gps_latitude0)*kx;
            double gps_y = (gps_longitude-gps_longitude0)*ky;
            double gps_z = (gps_alt-gps_alt0);

            gps_pos << gps_x,gps_y,gps_z;

            if((gps_started==1)&&(imu_calibrated==1)){
                if(rocket_fsm.state_machine == rocket_utils::FSM::IDLE)
                {
                    if(predict_on_idle){
                        if(!gps_pos.hasNaN()){
                            predict_step();
                            update_step_gps(gps_pos);
                        }
                    }
                }


                if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
                   && rocket_fsm.state_machine != rocket_utils::FSM::IDLE)
                {
                    if(!gps_pos.hasNaN()){
                        predict_step();
                        update_step_gps(gps_pos);
                    }
                }
            }


        }


		// Callback function to store last received sensor data from simulation
		void sensorCallback(const rocket_utils::Sensor::ConstPtr& sensor)
		{
            rocket_sensor.IMU_acc = sensor->IMU_acc;
            rocket_sensor.IMU_gyro = sensor->IMU_gyro;
			rocket_sensor.IMU_mag = sensor->IMU_mag;
			rocket_sensor.baro_height = sensor->baro_height;

            z_baro(0) = rocket_sensor.baro_height;
            mag_data << rocket_sensor.IMU_mag.x,rocket_sensor.IMU_mag.y,rocket_sensor.IMU_mag.z;
            gyro_data << rocket_sensor.IMU_gyro.x,rocket_sensor.IMU_gyro.y,rocket_sensor.IMU_gyro.z;

            if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
                && rocket_fsm.state_machine != rocket_utils::FSM::IDLE)
            {
                predict_step();
                update_step_baro(z_baro);
                update_step_mag(mag_data);
                update_step_gyro(gyro_data);
            }
		}

        void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
            optitrack_data << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;

            if(optitrack_started==0){
                optitrack_data0 = optitrack_data;
                optitrack_started = 1;
                std::cout << "optitrack fusion started\n\n";
            }

            if(rocket_fsm.state_machine == rocket_utils::FSM::IDLE)
            {
                if(predict_on_idle){
                    if(!optitrack_data.hasNaN()) {
                        predict_step();
                        update_step_optitrack(optitrack_data - optitrack_data0);
                    }
                }
            }

            if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
               && rocket_fsm.state_machine != rocket_utils::FSM::IDLE)
            {
                if(!optitrack_data.hasNaN()) {
                    predict_step();
                    update_step_optitrack(optitrack_data - optitrack_data0);
                }
            }
        }

		// Callback function to fake gps with sensor data !
        void gpsCallback(const rocket_utils::State::ConstPtr& state)
		{

			static double last_predict_time_gps = ros::Time::now().toSec();

            double dT_gps = ros::Time::now().toSec() - last_predict_time_gps;

			if(dT_gps>=1/gps_freq){
			            gps_pos << state->pose.position.x,state->pose.position.y,state->pose.position.z;

                        // construct a trivial random generator engine from a time-based seed:
                        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
                        std::default_random_engine generator(seed);
                        std::normal_distribution<double> gps_xy_noise(0.0, gps_noise);
                        gps_pos << gps_pos(0) + gps_xy_noise(generator),gps_pos(1) + gps_xy_noise(generator),gps_pos(2) + gps_xy_noise(generator);

                        if(rocket_fsm.state_machine != rocket_utils::FSM::CALIBRATION
                           && rocket_fsm.state_machine != rocket_utils::FSM::IDLE){
                            predict_step();
                            update_step_gps(gps_pos);
                        }

			            last_predict_time_gps = ros::Time::now().toSec();

			};
		}

		/* ------------ User functions ------------ */
        void calibrate_imu(){
            //average the initial sensor bias

            double g0 = 9.81;

            Matrix<double,3,1> IMU_omega_b_raw; IMU_omega_b_raw<< rocket_sensor.IMU_gyro.x, rocket_sensor.IMU_gyro.y, rocket_sensor.IMU_gyro.z;
            Matrix<double,3,1> IMU_acc_raw; IMU_acc_raw << rocket_sensor.IMU_acc.x, rocket_sensor.IMU_acc.y, rocket_sensor.IMU_acc.z-g0;

            sum_acc = sum_acc+IMU_acc_raw;
            sum_gyro = sum_gyro+IMU_omega_b_raw;
            imu_calibration_counter++;

            gyro_bias = sum_gyro/imu_calibration_counter;
            acc_bias = sum_acc/imu_calibration_counter;

            if(imu_calibration_counter>=imu_calibration_counter_calibrated){
                imu_calibrated = true;
            }

            std::cout << "calibrating IMU: steps: " << imu_calibration_counter << "/" << imu_calibration_counter_calibrated << "\n\n";

        }

    void homing_baro(){
        //average the initial pressure
        baro_pressure_sum = baro_pressure_sum+pressure;
        pressure_homing_counter++;
        pressure0 = baro_pressure_sum/pressure_homing_counter;
    }

    void homing_mag(){
            if(!mag_data.hasNaN()){
                //average the initial magnetic field
                mag_vec_inertial_sum = mag_vec_inertial_sum+mag_data;
                mag_homing_counter++;
                mag_vec_inertial = mag_vec_inertial_sum/mag_homing_counter;
                mag_vec_inertial_normalised = mag_vec_inertial;
                mag_vec_inertial_normalised.normalize();
                set_yaw_mag(); // use data from accelerometer and magnetometer to give inital attitude before take-off

                // estimate the normalised magnetic field in inertial frame and put in the state
                Eigen::Quaternion<double> attitude(X(9), X(6), X(7), X(8));
                attitude.normalize();
                Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
                X.segment(20,3) = rot_matrix*mag_vec_inertial_normalised;
            }
    }

    void set_yaw_mag(){

            // get normalised acceleration and magnetic vector from IMU
            Matrix<double,3,1> IMU_acc_norm; IMU_acc_norm << rocket_sensor.IMU_acc.x-acc_bias(0), rocket_sensor.IMU_acc.y-acc_bias(1), rocket_sensor.IMU_acc.z-acc_bias(2);
            IMU_acc_norm.normalize();
            Matrix<double,3,1> IMU_mag_norm = mag_vec_inertial;
            IMU_mag_norm.normalize();

            // get euler angles in the NED frame from the accelerometer and magnetometer data
            double yaw =-atan2(IMU_acc_norm[0],sqrt(IMU_acc_norm[1]*IMU_acc_norm[1]+IMU_acc_norm[2]*IMU_acc_norm[2]));
            double pitch = atan2(IMU_acc_norm[1],sqrt(IMU_acc_norm[0]*IMU_acc_norm[0]+IMU_acc_norm[2]*IMU_acc_norm[2]));
            double roll = atan2(-(IMU_mag_norm[0]*cos(pitch) - IMU_mag_norm[1]*sin(pitch)),(IMU_mag_norm[0]*cos(yaw) + IMU_mag_norm[1]*sin(pitch)*sin(yaw) + IMU_mag_norm[2]*cos(pitch)*sin(yaw))) +mag_declination;
            // double roll = atan2(-(IMU_mag_norm[1]*cos(pitch) - IMU_mag_norm[1]*sin(pitch)),(IMU_mag_norm[0]*cos(yaw) + IMU_mag_norm[1]*sin(pitch)*sin(yaw) + IMU_mag_norm[2]*cos(pitch)*sin(yaw))) +mag_declination;


            // updade the current attitude estimate with the sensor data
            typedef Eigen::EulerSystem<Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_X> euler_system;
            typedef Eigen::EulerAngles<double, euler_system> mag_angle_type;
            mag_angle_type init_angle(roll, pitch, yaw);
            Eigen::Quaterniond q(init_angle);

            X.segment(6,4) = q.coeffs();
    }

    void homing_gps(){
        gps_latitude_sum = gps_latitude_sum+gps_latitude;
        gps_longitude_sum = gps_longitude_sum+gps_longitude;
        gps_alt_sum = gps_alt_sum+gps_alt;
        gps_homing_counter++;

        gps_latitude0 = gps_latitude_sum/gps_homing_counter;
        gps_longitude0 = gps_longitude_sum/gps_homing_counter;
        gps_alt0 = gps_alt_sum/gps_homing_counter;

        latlongtometercoeffs(gps_latitude0*DEG2RAD,kx,ky);
        gps_started = 1;
    }

    template<int nz>
    void EKF_update(state &X,state_matrix &P,Matrix<double,nz,NX> H,Matrix<double, nz, nz> R,Matrix<double, nz, 1> inov){
        // compute EKF update
        // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
        Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
        Eigen::Matrix<double, nz, nz> S; // innovation covariance
        Eigen::Matrix<double, NX, nz> K; // Kalman gain
        Eigen::Matrix<double, NX, NX> I; // identity

        I.setIdentity();
        S = H * P * H.transpose() + R;
        K = S.llt().solve(H * P).transpose();
        X = X + K * (inov); // inov = z - h_x

        IKH = (I - K * H);
        P = IKH * P * IKH.transpose() + K * R * K.transpose();
    }

    void latlongtometercoeffs(double lat0,double &kx,double &ky){
        kx = 111132.92-559.82*cos(2*lat0)+1.175*cos(4*lat0)-0.0023*cos(6*lat0); // meters per degree of latitude
        ky = 111412.84*cos(lat0)-93.5*cos(3*lat0)+0.118*cos(5*lat0); // meters per degree of longitude
    }

    void fullDerivative(const state &x,
                        const state_matrix &P,
                        state &xdot,
                        state_matrix &Pdot) {

        //X derivative
        predictionModels->state_dynamics(x, xdot,IMU_omega_b,IMU_acc,control_torque_body,total_mass,I, I_inv);

        //P derivative
        //propagate xdot autodiff scalar at current x
        ADx = x;
        ad_state Xdot;
        predictionModels->state_dynamics(ADx, Xdot,IMU_omega_b,IMU_acc,control_torque_body,total_mass,I, I_inv);

        // fetch the jacobian wrt x of f(x,w)
        for (int i = 0; i < Xdot.size(); i++) {
            F.row(i) = Xdot(i).derivatives();
        }

        ADw = W;
        ad_state_noise Xdot_noise;

        predictionModels->noise_dynamics(ADw,Xdot_noise,x);

        // fetch the jacobian wrt w of f(x,w)
        for (int i = 0; i < Xdot_noise.size(); i++) {
            G_noise.row(i) = Xdot_noise(i).derivatives();
        }

        Q = G_noise*W_noise*(G_noise.transpose());

        Pdot = F * P + P * F.transpose() + Q;


    }

    void RK4(const double dT,state &X,const state_matrix &P,state &Xnext,state_matrix &Pnext) {
        state k1, k2, k3, k4;
        state_matrix k1_P, k2_P, k3_P, k4_P;

        //Inertia
        I_inv << 1 / rocket.total_Inertia[0], 1 / rocket.total_Inertia[1], 1 / rocket.total_Inertia[2];
        I << rocket.total_Inertia[0], rocket.total_Inertia[1],rocket.total_Inertia[2];

        // Current acceleration and angular rate from IMU
        IMU_omega_b << rocket_sensor.IMU_gyro.x-gyro_bias(0), rocket_sensor.IMU_gyro.y-gyro_bias(1), rocket_sensor.IMU_gyro.z-gyro_bias(2);
        IMU_acc << rocket_sensor.IMU_acc.x-acc_bias(0), rocket_sensor.IMU_acc.y-acc_bias(1), rocket_sensor.IMU_acc.z-acc_bias(2);

        //update rotation speed to those of gyro
        X.segment(10,3) = IMU_omega_b;

        //RK4 integration
        fullDerivative(X, P, k1, k1_P);
        fullDerivative(X + k1 * dT / 2, P + k1_P * dT / 2, k2, k2_P);
        fullDerivative(X + k2 * dT / 2, P + k2_P * dT / 2, k3, k3_P);
        fullDerivative(X + k3 * dT, P + k3_P * dT, k4, k4_P);

        Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;

        Pnext = P + (k1_P + 2 * k2_P + 2 * k3_P + k4_P) * dT / 6;
    }




		void predict_step()
		{

//             //Attitude diplay on terminal
//            Eigen::Quaternion<double> attitude(X(9), X(6), X(7), X(8));
//            Matrix<double,3,3> R = attitude.toRotationMatrix();
//
//            double alpha = atan2(-R(1,2),R(2,2));
//            double beta = atan2(R(0,2),sqrt(R(0,0) * R(0,0) + R(0,1) * R(0,1)));
//            double gamma = atan2(-R(0,1),R(0,0));
//            std::cout << "alpha:" << alpha/DEG2RAD << " beta:" << beta/DEG2RAD << " gamma:" << gamma/DEG2RAD << "\n\n";
//            //

            static double last_predict_time = ros::Time::now().toSec();
			double dT = ros::Time::now().toSec() - last_predict_time;
            RK4(dT,X,P,X,P);
            last_predict_time = ros::Time::now().toSec();

            // normalise the magnetic vector
            Matrix<double,3,1> magnetic_vector;
            magnetic_vector =  X.segment(20,3);
            magnetic_vector.normalize();
            X.segment(20,3) = magnetic_vector;

        }


        void update_step_baro(const sensor_data_baro &z)
        {
            //propagate hdot autodiff scalar at current x
            ADx = X;
            ad_sensor_data_baro hdot;
            mesurementModels->mesurementModelBaro(ADx, hdot);

            //compute h(x)
            sensor_data_baro h_x;
            mesurementModels->mesurementModelBaro(X, h_x);


            // obtain the jacobian of h(x)
            for (int i = 0; i < hdot.size(); i++) {
                H_baro.row(i) = hdot(i).derivatives();
            }

            Eigen::Matrix<double, NZBARO, 1> inov = z-h_x;
            EKF_update(X,P,H_baro,R_baro,inov);


        }

    void update_step_gps(const sensor_data_gps &z)
    {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_gps hdot;
        mesurementModels->mesurementModelGPS(ADx, hdot);

        //compute h(x)
        sensor_data_gps h_x;
        mesurementModels->mesurementModelGPS(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_gps.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZGPS, 1> inov = z-h_x;
        EKF_update(X,P,H_gps,R_gps,inov);
    }

    void update_step_optitrack(const sensor_data_optitrack &z)
    {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_optitrack hdot;
        mesurementModels->mesurementModelOptitrack(ADx, hdot);

        //compute h(x)
        sensor_data_optitrack h_x;
        mesurementModels->mesurementModelOptitrack(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_optitrack.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZOPTITRACK, 1> inov = z-h_x;
        EKF_update(X,P,H_optitrack,R_optitrack,inov);

    }


    void update_step_gyro(const sensor_data_gyro &z)
    {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_gyro hdot;
        mesurementModels->mesurementModelGyro(ADx, hdot,gyro_bias);

        //compute h(x)
        sensor_data_gyro h_x;
        mesurementModels->mesurementModelGyro(X, h_x,gyro_bias);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_gyro.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZGYRO, 1> inov = z-h_x;
        EKF_update(X,P,H_gyro,R_gyro,inov);
    }

    void update_step_mag(const sensor_data_mag &z)
    {

        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_mag hdot;
        mesurementModels->mesurementModelMag(ADx, hdot);

        //compute h(x)
        sensor_data_mag h_x;
        mesurementModels->mesurementModelMag(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_mag.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZMAG, 1> inov = z-h_x;
        EKF_update(X,P,H_mag,R_mag,inov);
    }


		void updateNavigation()
		{

            Eigen::Quaternion<double> attitude(X(9), X(6), X(7), X(8));
            attitude.normalize();
            Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
            Matrix<double,3,1> omega_body;
            omega_body << X(10),X(11),X(12);
            Matrix<double,3,1> omega_inertial;
            omega_inertial = rot_matrix*omega_body;

			// Parse navigation state and publish it on the /nav_pub topic
			rocket_utils::State rocket_state;
            rocket_utils::ExtendedState extended_rocket_state;

            rocket_state.header.stamp = ros::Time::now();
            extended_rocket_state.header.stamp = ros::Time::now();

			rocket_state.pose.position.x = X(0);
			rocket_state.pose.position.y = X(1);
			rocket_state.pose.position.z = X(2);

			rocket_state.twist.linear.x = X(3);
			rocket_state.twist.linear.y = X(4);
			rocket_state.twist.linear.z = X(5);

			rocket_state.pose.orientation.x = X(6);
			rocket_state.pose.orientation.y = X(7);
			rocket_state.pose.orientation.z = X(8);
			rocket_state.pose.orientation.w = X(9);

			rocket_state.twist.angular.x = omega_inertial(0);
			rocket_state.twist.angular.y = omega_inertial(1);
			rocket_state.twist.angular.z = omega_inertial(2);

			rocket_state.propeller_mass = rocket.propellant_mass;

            extended_rocket_state.state = rocket_state;

            extended_rocket_state.barometer_bias = X(19);

            if(is_simulation){
                extended_rocket_state.is_calibrated = true; // if we are runing in simulation, consider sensors calibrated
            }else{
                extended_rocket_state.is_calibrated = imu_calibrated;
            }


            extended_rocket_state.bias_gyro.x = X(13);
            extended_rocket_state.bias_gyro.y = X(14);
            extended_rocket_state.bias_gyro.z = X(15);
            extended_rocket_state.bias_acc.x = X(16);
            extended_rocket_state.bias_acc.y = X(17);
            extended_rocket_state.bias_acc.z = X(18);

            Matrix<double,NX,1> P_diag;
            P_diag = P.diagonal();
            std::vector<double> P_vec(P_diag.data(), P_diag.data() + NX);
            std::vector<double> P_quat(P.block(6,6,4,4).data(), P.block(6,6,4,4).data() + 4*4);
            extended_rocket_state.covariance = P_vec;
            extended_rocket_state.quat_covariance = P_quat;

            state_pub.publish(rocket_state);

            extended_state_pub.publish(extended_rocket_state);
        }
};


int main(int argc, char **argv)
{
	// Init ROS time keeper node
	ros::init(argc, argv, "data_fusion");
	ros::NodeHandle nh("~");

	NavigationNode navigationNode(nh);

	// Thread to compute navigation state. Duration defines interval time in seconds
	ros::Timer control_thread = nh.createTimer(ros::Duration(navigationNode.dt_ros),
	[&](const ros::TimerEvent&)
	{
		navigationNode.updateNavigation();
	});

	// Automatic callback of service and publisher from here
	ros::spin();
}
