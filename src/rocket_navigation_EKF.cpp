/*
* Node to estimate the rocket full state (position, velocity, attitude quaternion, angular rate and mass) 
* from the sensor data and commanded thrust and torque of the rocket engine
*
* Inputs: 
*   - Finite state machine from the rocket_fsm :	     /gnc_fsm_pub
*   - Measured 3D force and torque from av_interface:	 /control_measured
*   - Sensor data (IMU and barometer) from av_interface: /sensor_pub
*
* Parameters:
*   - Rocket model: 		/config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
#	- Kalman matrix: 		class NavigationNode
*
* Outputs:
*   - Complete estimated state : /kalman_rocket_state
*
*/

#include "ros/ros.h"

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Control.h"
#include "real_time_simulator/Sensor.h"
#include "real_time_simulator/StateCovariance.h"


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"


#include <time.h>
#include <sstream>
#include <string>

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <unsupported/Eigen/EulerAngles>

#include "rocket_model.hpp"

#include <type_traits>


// used added
#include <random>
#include <autodiff/AutoDiffScalar.h>
using namespace Eigen;
//end used added

#define DEG2RAD 0.01745329251

class NavigationNode {
	public:
        const float dt_ros = 0.005;

        const int simulation = 1; // 1=runs in simulation (SIL), 0=runs on drone with optitrack
        const int use_gps = 0; // 0=use optitrack, 1=use px4 gps

        static const int NX = 20; // number of states

		static const int NZBARO = 1;
        static const int NZGPS = 6;
        static const int NZMAG = 3;
        static const int NZACC = 3;
        static const int NZGYRO = 3;
        static const int NZFAKESENSOR= 3;
        static const int NZOPTITRACK = 3;



    /// AUTODIFF stuff
        // Autodiff for state
    	template<typename scalar_t>
    	using state_t = Eigen::Matrix<scalar_t, NX, 1>;
    	using state = state_t<double>;
    	using ad_state = state_t<AutoDiffScalar<state>>;// Autodiff state variable

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

        // Autodiff for accelerometer
        template<typename scalar_t>
        using sensor_data_acc_t = Eigen::Matrix<scalar_t, NZACC, 1>;
        using sensor_data_acc = sensor_data_acc_t<double>;
        using ad_sensor_data_acc = sensor_data_acc_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable magnetometer

        // Autodiff for gyroscope
        template<typename scalar_t>
        using sensor_data_gyro_t = Eigen::Matrix<scalar_t, NZGYRO, 1>;
        using sensor_data_gyro = sensor_data_gyro_t<double>;
        using ad_sensor_data_gyro = sensor_data_gyro_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable gyroscope

        // Autodiff for gyroscope
        template<typename scalar_t>
        using sensor_data_fsen_t = Eigen::Matrix<scalar_t, NZFAKESENSOR, 1>;
        using sensor_data_fsen = sensor_data_fsen_t<double>;
        using ad_sensor_data_fsen = sensor_data_fsen_t<AutoDiffScalar<state_t<double>>>;// Autodiff sensor variable fake sensor

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
        typedef Eigen::Matrix<double, NZACC, NZACC> sensor_matrix_acc;
        typedef Eigen::Matrix<double, NZGYRO, NZGYRO> sensor_matrix_gyro;
        typedef Eigen::Matrix<double, NZFAKESENSOR, NZFAKESENSOR> sensor_matrix_fsen;
        typedef Eigen::Matrix<double, NZOPTITRACK, NZOPTITRACK> sensor_matrix_optitrack;


    /// AUTODIFF stuff end

	private:

        sensor_data_gps gps_data;
        sensor_data_baro z_baro;
        sensor_data_mag mag_data;
        sensor_data_gyro gyro_data;
        sensor_data_fsen fsen_data;
        sensor_data_optitrack optitrack_data;


    // Class with useful rocket parameters and methods
		Rocket rocket;

		// Last received fsm
		real_time_simulator::FSM rocket_fsm;

		// Last received control
		real_time_simulator::Control rocket_control;

		// Last received sensor data
		real_time_simulator::Sensor rocket_sensor;

		// List of subscribers and publishers
		ros::Publisher nav_pub;
        ros::Publisher cov_pub;

		ros::Subscriber fsm_sub;
		ros::Subscriber control_sub;
		ros::Subscriber rocket_state_sub;
		ros::Subscriber sensor_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber optitrack_sub;
        ros::Subscriber px4_imu_sub;
        ros::Subscriber px4_mag_sub;
        ros::Subscriber px4_gps_sub;



    // Kalman matrix
        sensor_matrix_baro R_baro;
        sensor_matrix_gps R_gps;
        sensor_matrix_mag R_mag;
        sensor_matrix_acc R_acc;
        sensor_matrix_gyro R_gyro;
        sensor_matrix_fsen R_fsen;
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
        Matrix<double,NZACC,NX> H_acc; // computed using autodiff
        Matrix<double,NZGYRO,NX> H_gyro; // computed using autodiff
        Matrix<double,NZFAKESENSOR,NX> H_fsen; // computed using autodiff
        Matrix<double,NZOPTITRACK,NX> H_optitrack; // computed using autodiff



    // Kalman state
		state X , Z_fakesensor;
		ad_state ADx; // state used for autodiff

		/// Fake GPS
		// Last received fake GPS data
		Eigen::Matrix<double, 3, 1> gps_pos;
        Eigen::Matrix<double, 3, 1> gps_vel;


        double gps_freq = 10;

        double gps_noise_xy = .8;
        double gps_noise_z = .6;
		double gps_noise_xy_vel = .2;
		double gps_noise_z_vel = .2;


        //double gps_noise_xy = .008;
        //double gps_noise_z = .06;
        //double gps_noise_xy_vel = .008;
        //double gps_noise_z_vel = .06;

    // end fakegps

    float dry_mass;

    int gps_started = 0; // autmatically set to 0 once gps fix arrives
    float kx = 0;
    float ky = 0;
    float gps_latitude0 = 0;
    float gps_longitude0 = 0;
    float gps_alt0 = 0;
    float gps_latitude = 0;
    float gps_longitude = 0;
    float gps_alt = 0;

    Matrix<double,3,1> acc_bias;
    Matrix<double,3,1> gyro_bias;
    Matrix<double,3,1> mag_bias;
    double baro_bias;



public:
		NavigationNode(ros::NodeHandle nh)
		{
			// Initialize publishers and subscribers
        	initTopics(nh);

			// Initialize fsm
			rocket_fsm.time_now = 0;
			rocket_fsm.state_machine = "Idle";

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

            acc_bias << 0,0,0;
            gyro_bias << 0,0,0;
            mag_bias << 0,0,0;
            baro_bias = 0;

			/// Init state X
            X.setZero();
            X(13) = rocket.propellant_mass;
			X.segment(6,4) = q.coeffs();
            //X.segment(30,3) << 1.0,0.0,0.0;

            /// Initialize kalman parameters
            // sensor covarience matrices
            R_baro(0,0) = 0.001*0.001;

            R_gps.setIdentity();

            //R_gps = R_gps*0.8*0.8;
            //R_gps(2,2) = 0.6*0.6;
            //R_gps.block(3,3,3,3) =Eigen::Matrix<double, 3, 3>::Identity(3, 3) * 0.2*0.2;

            //R_gps = R_gps*0.8*0.8;
            //R_gps(2,2) = 0.6*0.6;
            //R_gps.block(3,3,3,3) =Eigen::Matrix<double, 3, 3>::Identity(3, 3) * 0.2*0.2;

            R_gps = R_gps*38;
            R_gps(2,2) = 50;
            R_gps.block(3,3,3,3) =Eigen::Matrix<double, 3, 3>::Identity(3, 3) * 60;


            R_mag.setIdentity();
            R_mag = R_mag*0.1*0.1;

            R_gyro.setIdentity();
            R_gyro = R_gyro*0.001*0.001;

            R_acc.setIdentity();

            R_fsen.setIdentity();
            R_fsen = R_fsen*10;

            R_optitrack.setIdentity();
            R_optitrack = R_optitrack*0.0001;

            P.setZero(); // no error in the initial state

            // process covariance matrix
			Q.setIdentity();
            Q.block(0,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3)*0.000003*dt_ros;
            Q(2,2) = 1*dt_ros;
            Q.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3)*0.000001*dt_ros;
            Q(5,5) = 1*dt_ros;

            Q(6,6) = 0.0001*dt_ros;
            Q(7,7) = 0.0001*dt_ros;
            Q(8,8) = 0.000001*dt_ros;
            Q(9,9) = 0.000001*dt_ros;

            Q.block(10,10,3,3) =  Eigen::Matrix<double, 3, 3>::Identity(3, 3)*0.001*0.001;
            Q.block(14,14,6,6) =Eigen::Matrix<double, 6, 6>::Identity(6, 6) * 0;


            // Init derivatives
        		ADx(X);
        		int div_size = ADx.size();
        		int derivative_idx = 0;
        		for (int i = 0; i < ADx.size(); ++i) {
            			ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
            			derivative_idx++;
        		}
		}

		void initTopics(ros::NodeHandle &nh) 
		{

            if(simulation==1){
                // Subscribe to sensor for kalman correction
                sensor_sub = nh.subscribe("sensor_pub", 1, &NavigationNode::sensorCallback, this);

                // Subscribe to state to fake GPS
                gps_sub = nh.subscribe("rocket_state", 1, &NavigationNode::gpsCallback, this);
            }else{

                px4_imu_sub = nh.subscribe("/mavros/imu/data_raw", 1, &NavigationNode::px4imuCallback, this);

                px4_mag_sub = nh.subscribe("/mavros/imu/mag", 1, &NavigationNode::px4magCallback, this);

                if(use_gps==1){
                    px4_gps_sub = nh.subscribe("/mavros/global_position/raw/fix", 1, &NavigationNode::px4gpsCallback, this);
                }else{
                    optitrack_sub = nh.subscribe("/optitrack_client/Drone/optitrack_pose", 1, &NavigationNode::optitrackCallback, this);
                }


            }



			// Create filtered rocket state publisher
			nav_pub = nh.advertise<real_time_simulator::State>("kalman_rocket_state", 1);

            // Create state covarience publisher
            cov_pub = nh.advertise<real_time_simulator::StateCovariance>("state_covariance", 1);

			// Subscribe to time_keeper for fsm and time
			fsm_sub = nh.subscribe("gnc_fsm_pub", 1, &NavigationNode::fsmCallback, this);

			// Subscribe to control for kalman estimator
			control_sub = nh.subscribe("control_measured", 1, &NavigationNode::controlCallback, this);

		}

		/* ------------ Callbacks functions ------------ */




		// Callback function to store last received fsm
		void fsmCallback(const real_time_simulator::FSM::ConstPtr& fsm)
		{
			rocket_fsm.time_now = fsm->time_now;
			rocket_fsm.state_machine = fsm->state_machine;
		}

		// Callback function to store last received control
		void controlCallback(const real_time_simulator::Control::ConstPtr& control)
		{
			rocket_control.torque = control->torque;
			rocket_control.force = control->force;
		}

        void px4imuCallback(const sensor_msgs::Imu::ConstPtr& sensor)
        {
            rocket_sensor.IMU_acc = sensor->linear_acceleration;
            rocket_sensor.IMU_gyro = sensor->angular_velocity;
        }

        void px4magCallback(const sensor_msgs::MagneticField::ConstPtr& sensor)
        {
            rocket_sensor.IMU_mag = sensor->magnetic_field;

            if(rocket_fsm.state_machine.compare("Coast") == 0||rocket_fsm.state_machine.compare("Launch") == 0||rocket_fsm.state_machine.compare("Rail") == 0)
            {
                predict_step();
                update_step_mag(mag_data);
            }
        }

        void px4gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps)
        {
            gps_latitude = gps->latitude;
            gps_longitude = gps->longitude;
            gps_alt = gps->altitude;

            // setup origin for gps
            if(gps_started==0){
                gps_latitude0 = gps_latitude;
                gps_longitude0 = gps_longitude;
                gps_alt0 = gps_alt;

                latlongtometercoeffs(gps_latitude0,kx,ky);
                gps_started = 1;
            }

//            float gps_x = (gps_latitude-gps_latitude0)*kx;
//            float gps_y = (gps_longitude-gps_longitude0)*ky;
//            float gps_z = (gps_alt-gps_alt0);
            gps_pos << (gps_latitude-gps_latitude0)*kx,(gps_longitude-gps_longitude0)*ky,(gps_alt-gps_alt0);


//
//            if(rocket_fsm.state_machine.compare("Coast") == 0||rocket_fsm.state_machine.compare("Launch") == 0||rocket_fsm.state_machine.compare("Rail") == 0)
//            {
//                predict_step();
//                update_step_gps_pos(gps_pos);
//            }
        }


		// Callback function to store last received sensor data
		void sensorCallback(const real_time_simulator::Sensor::ConstPtr& sensor)
		{
            rocket_sensor.IMU_acc = sensor->IMU_acc;
            rocket_sensor.IMU_gyro = sensor->IMU_gyro;
			rocket_sensor.IMU_mag = sensor->IMU_mag;
			rocket_sensor.baro_height = sensor->baro_height;

            z_baro(0) = rocket_sensor.baro_height;
            mag_data << rocket_sensor.IMU_mag.x,rocket_sensor.IMU_mag.y,rocket_sensor.IMU_mag.z;
            gyro_data << rocket_sensor.IMU_gyro.x,rocket_sensor.IMU_gyro.y,rocket_sensor.IMU_gyro.z;

            if(rocket_fsm.state_machine.compare("Coast") == 0||rocket_fsm.state_machine.compare("Launch") == 0||rocket_fsm.state_machine.compare("Rail") == 0)
            {
                predict_step();
                update_step_baro(z_baro);
                update_step_mag(mag_data);
            }


		}

        void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
            optitrack_data << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;

            if(rocket_fsm.state_machine.compare("Coast") == 0||rocket_fsm.state_machine.compare("Launch") == 0||rocket_fsm.state_machine.compare("Rail") == 0)
            {
                predict_step();
                update_step_optitrack(optitrack_data);
            }
        }
		
		// Callback function to fake gps with sensor data !
        void gpsCallback(const real_time_simulator::State::ConstPtr& state)
		{
			
			static double last_predict_time_gps = ros::Time::now().toSec();


            double dT_gps = ros::Time::now().toSec() - last_predict_time_gps;
			
			if(dT_gps>=1/gps_freq){
			            gps_pos << state->pose.position.x,state->pose.position.y,state->pose.position.z;
                        gps_vel << state->twist.linear.x,state->twist.linear.y,state->twist.linear.z;

                        // construct a trivial random generator engine from a time-based seed:
                        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
                        std::default_random_engine generator(seed);

                        std::normal_distribution<double> gps_xy_noise(0.0, gps_noise_xy);
                        std::normal_distribution<double> gps_z_noise(0.0, gps_noise_z);


                        std::normal_distribution<double> gps_xy_noise_vel(0.0, gps_noise_xy_vel);
                        std::normal_distribution<double> gps_z_noise_vel(0.0, gps_noise_z_vel);


                        gps_pos << gps_pos(0) + gps_xy_noise(generator),gps_pos(1) + gps_xy_noise(generator),gps_pos(2) + gps_z_noise(generator);
                        gps_vel << gps_vel(0) + gps_xy_noise_vel(generator),gps_vel(1) + gps_xy_noise_vel(generator),gps_vel(2) + gps_z_noise_vel(generator);

                        gps_data.segment(0,3) = gps_pos;
                        gps_data.segment(3,3) = gps_vel;

                        if(rocket_fsm.state_machine.compare("Coast") == 0||rocket_fsm.state_machine.compare("Launch") == 0||rocket_fsm.state_machine.compare("Rail") == 0){
                            predict_step();
                            update_step_gps(gps_data);
                        }

			            last_predict_time_gps = ros::Time::now().toSec();

			};

		}

		/* ------------ User functions ------------ */
		
		template<typename T>
		void state_dynamics(state_t<T> x, state_t<T> &xdot)
		{
			// -------------- Simulation variables -----------------------------
			T g0 = (T) 9.81;  // Earth gravity in [m/s^2]

			Eigen::Matrix<T, 3, 1> rocket_force;
			rocket_force << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;

            Eigen::Matrix<T, 3, 1> control_torque;
            control_torque << rocket_control.torque.x, rocket_control.torque.y, rocket_control.torque.z;

			// Orientation of the rocket with quaternion
			Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

			// Current acceleration and angular rate from IMU
			Eigen::Matrix<T, 3, 1> IMU_acc; IMU_acc << (T) rocket_sensor.IMU_acc.x-acc_bias(0),(T) rocket_sensor.IMU_acc.y-acc_bias(1),(T) rocket_sensor.IMU_acc.z-acc_bias(2);


            Eigen::Matrix<T, 3, 1> omega;
            omega << rocket_sensor.IMU_gyro.x-gyro_bias(0), rocket_sensor.IMU_gyro.y-gyro_bias(1), rocket_sensor.IMU_gyro.z-gyro_bias(2);


            // Angular velocity omega in quaternion format to compute quaternion derivative
            Eigen::Quaternion<T> omega_quat(0.0, omega(0), omega(1), omega(2));
            //Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

            //Inertia
            Matrix<T, 3, 1> I_inv;
            I_inv << 1 / rocket.total_Inertia[0], 1 / rocket.total_Inertia[1], 1 / rocket.total_Inertia[2];
            Matrix<T, 3, 1> I;
            I<< rocket.total_Inertia[0], rocket.total_Inertia[1],rocket.total_Inertia[2];

            Eigen::Matrix<T, 3, 1> dist_force;
            dist_force << x(14),x(15),x(16);

            Eigen::Matrix<T, 3, 1> dist_torque;
            dist_torque << x(17),x(18),x(19);

            // compute total torque in body frame
            Eigen::Matrix<T, 3, 1> total_torque_body;
            total_torque_body = control_torque + rot_matrix.transpose()*dist_torque;

            // -------------- Differential equation ---------------------

			// Position variation is speed
			xdot.head(3) = x.segment(3,3);

			// Speed variation is acceleration
			xdot.segment(3,3) =  rot_matrix*IMU_acc - Eigen::Vector3d::UnitZ().template cast<T>() *g0 +(dist_force)/(dry_mass+x(13));

			// Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
            xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();

			// Angular speed
            xdot.segment(10, 3) = rot_matrix*(total_torque_body - omega.cross(I.template cast<T>().cwiseProduct(omega))).cwiseProduct(I_inv.template cast<T>());

			// Mass variation
            xdot(13) = 0;

            // Disturbance forces
            xdot.segment(14, 3) << 0, 0, 0;

            // Disturbance moments
            xdot.segment(17, 3) << 0, 0, 0;

		}

    template<typename T>
    void state_dynamics_forcemodel(state_t<T> x, state_t<T> &xdot)
    {
        // -------------- Simulation variables -----------------------------
        T g0 = (T) 9.81;  // Earth gravity in [m/s^2]

        Eigen::Matrix<T, 3, 1> control_force;
        control_force << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;

        Eigen::Matrix<T, 3, 1> control_torque;
        control_torque << rocket_control.torque.x, rocket_control.torque.y, rocket_control.torque.z;

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        Eigen::Matrix<T, 3, 1> dist_force;
        dist_force << x(14),x(15),x(16);

        Eigen::Matrix<T, 3, 1> dist_torque;
        dist_torque << x(17),x(18),x(19);

        // compute total force inertial frame
        Eigen::Matrix<T, 3, 1> total_force_inertial;
        total_force_inertial = rot_matrix*(control_force) - Eigen::Vector3d::UnitZ().template cast<T>()*(x(13)+dry_mass)*g0 + dist_force;

        // compute total torque in body frame
        Eigen::Matrix<T, 3, 1> total_torque_body;
        total_torque_body = control_torque + rot_matrix.transpose()*dist_torque;

        Eigen::Matrix<T, 3, 1> omega;
        omega << x(10),x(11),x(12);
        omega = rot_matrix.transpose()*omega;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

        //Inertia
        Matrix<T, 3, 1> I_inv;
        I_inv << 1 / rocket.total_Inertia[0], 1 / rocket.total_Inertia[1], 1 / rocket.total_Inertia[2];
        Matrix<T, 3, 1> I;
        I << rocket.total_Inertia[0], rocket.total_Inertia[1],rocket.total_Inertia[2];

        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3,3);

        // Speed variation is acceleration given by newton's law
        xdot.segment(3,3) =  total_force_inertial/(x(13)+dry_mass);

        // Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
        xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();

        // Angular speed variation is given by euler's equation if in body frame
        xdot.segment(10, 3) = rot_matrix*(total_torque_body - omega.cross(I.template cast<T>().cwiseProduct(omega))).cwiseProduct(I_inv.template cast<T>());

        // no mass variation
        xdot(13) = 0;

        // Disturbance forces
        xdot.segment(14, 3) << 0, 0, 0;

        // Disturbance moments
        xdot.segment(17, 3) << 0, 0, 0;
    }

        template<typename T>
        void mesurementModelBaro(const state_t<T> &x, sensor_data_baro_t<T> &z) {
            z(0) = x(2)+baro_bias;
        }

        template<typename T>
        void mesurementModelOptitrack(const state_t<T> &x, sensor_data_optitrack_t<T> &z) {
            z = x.segment(0,3);
        }

        template<typename T>
        void mesurementModelMag(const state_t<T> &x, sensor_data_mag_t<T> &z) {
            // get rotation matrix
            Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
            attitude.normalize();
            Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();
            Eigen::Matrix<T, 3, 1> mag_vec;
            mag_vec << 1.0,0.0,0.0;

            // express inertial magnetic vector estimate in body-frame and add bias
            z = rot_matrix.transpose()*(mag_vec) + mag_bias;
        }

        template<typename T>
        void mesurementModelAcc(const state_t<T> &x, sensor_data_acc_t<T> &z) {
            T g0 = (T) 9.81;  // Earth gravity in [m/s^2]
            // get rotation matrix
            Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
            attitude.normalize();
            Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

            Eigen::Matrix<T, 3, 1> control_force;
            control_force << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;

            Eigen::Matrix<T, 3, 1> dist_force;
            dist_force << x(14),x(15),x(16);

            Eigen::Matrix<T, 3, 1> total_force_inertial;
            total_force_inertial = rot_matrix*(control_force) - Eigen::Vector3d::UnitZ().template cast<T>()*(x(13)+dry_mass)*g0 + dist_force;

            // express inertial magnetic vector estimate in body-frame and add bias
            z = rot_matrix.transpose()*(total_force_inertial/(dry_mass+x(13))) + acc_bias;
        }

        template<typename T>
        void mesurementModelGPS(const state_t<T> &x, sensor_data_gps_t<T> &z) {
            z = x.segment(0,6);
        }

        template<typename T>
        void mesurementModelFakeSensor(const state_t<T> &x, sensor_data_fsen_t<T> &z) {
            z = x.segment(3,3);
        }

        template<typename T>
        void mesurementModelGyro(const state_t<T> &x, sensor_data_gyro_t<T> &z) {
            Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
            attitude.normalize();
            Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

            z = rot_matrix.transpose()*(x.segment(10,3))+gyro_bias;
        }
		
		void fullDerivative(const state &x,
                        const state_matrix &P,
                        state &xdot,
                        state_matrix &Pdot) {
                        
        		//X derivative
        		state_dynamics(x, xdot);

        		//P derivative
        		//propagate xdot autodiff scalar at current x
        		ADx = X;
        		ad_state Xdot;	
        		state_dynamics(ADx, Xdot);

        		// fetch the jacobian of f(x)
        		for (int i = 0; i < Xdot.size(); i++) {
            			F.row(i) = Xdot(i).derivatives();
        		}

        		Pdot = F * P + P * F.transpose() + Q;
        }
		

		void RK4(const double dT,state &X,const state_matrix &P,state &Xnext,state_matrix &Pnext) {
        		state k1, k2, k3, k4;
        		state_matrix k1_P, k2_P, k3_P, k4_P;

                 //update rotation speed to those of gyro
                Matrix<double,3,1>omega_b;
                Eigen::Quaternion<double> attitude(X(9), X(6), X(7), X(8));
                attitude.normalize();
                Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
                omega_b <<rocket_sensor.IMU_gyro.x-gyro_bias(0), rocket_sensor.IMU_gyro.y-gyro_bias(1), rocket_sensor.IMU_gyro.z-gyro_bias(2);
                X.segment(10,3) = rot_matrix*omega_b;

                //RK4 integration
        		fullDerivative(X, P, k1, k1_P);
        		fullDerivative(X + k1 * dT / 2, P + k1_P * dT / 2, k2, k2_P);
        		fullDerivative(X + k2 * dT / 2, P + k2_P * dT / 2, k3, k3_P);
        		fullDerivative(X + k3 * dT, P + k3_P * dT, k4, k4_P);

                Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
                Pnext = P + (k1_P + 2 * k2_P + 2 * k3_P + k4_P) * dT / 6;
        }

    void RK4_fakesensor(const double dT,state &X,state &Xnext) {
        state k1, k2, k3, k4, Xinter;

        state_dynamics_forcemodel(X, k1);Xinter = X + k1 * dT / 2;
        state_dynamics_forcemodel( Xinter, k2);Xinter = X + k2 * dT / 2;
        state_dynamics_forcemodel( Xinter, k3); Xinter = X + k3 * dT;
        state_dynamics_forcemodel( Xinter, k4);

        Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;

    }

		void predict_step()
		{
			static double last_predict_time = ros::Time::now().toSec();
			double dT = ros::Time::now().toSec() - last_predict_time;
			RK4(dT,X,P,X,P);
            RK4_fakesensor(dT,X,Z_fakesensor);
			last_predict_time = ros::Time::now().toSec();
		}


        void update_step_baro(const sensor_data_baro &z)
        {
            //propagate hdot autodiff scalar at current x
            ADx = X;
            ad_sensor_data_baro hdot;
            mesurementModelBaro(ADx, hdot);

            //compute h(x)
            sensor_data_baro h_x;
            mesurementModelBaro(X, h_x);

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
        mesurementModelGPS(ADx, hdot);

        //compute h(x)
        sensor_data_gps h_x;
        mesurementModelGPS(X, h_x);

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
        mesurementModelOptitrack(ADx, hdot);

        //compute h(x)
        sensor_data_optitrack h_x;
        mesurementModelOptitrack(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_optitrack.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZOPTITRACK, 1> inov = z-h_x;
        EKF_update(X,P,H_optitrack,R_optitrack,inov);

    }

    void update_step_fsen(const sensor_data_fsen &z)
    {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_fsen hdot;
        mesurementModelFakeSensor(ADx, hdot);

        //compute h(x)
        sensor_data_fsen h_x;
        mesurementModelFakeSensor(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_fsen.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZFAKESENSOR, 1> inov = z-h_x;
        EKF_update(X,P,H_fsen,R_fsen,inov);

    }

    void update_step_gyro(const sensor_data_gyro &z)
    {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_gyro hdot;
        mesurementModelGyro(ADx, hdot);

        //compute h(x)
        sensor_data_gyro h_x;
        mesurementModelGyro(X, h_x);

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
        mesurementModelMag(ADx, hdot);

        //compute h(x)
        sensor_data_mag h_x;
        mesurementModelMag(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_mag.row(i) = hdot(i).derivatives();
        }
        Eigen::Matrix<double, NZMAG, 1> inov = z-h_x;
        EKF_update(X,P,H_mag,R_mag,inov);
    }

    // !!!!!! this does not work in flight !!!!!!!!!
    void update_step_acc(const sensor_data_acc &z)
    {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data_acc hdot;
        mesurementModelAcc(ADx, hdot);

        //compute h(x)
        sensor_data_acc h_x;
        mesurementModelAcc(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H_acc.row(i) = hdot(i).derivatives();
        }

        Eigen::Matrix<double, NZACC, 1> inov = z-h_x;
        EKF_update(X,P,H_acc,R_acc,inov);

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

    void latlongtometercoeffs(float lat0,float &kx,float &ky){
        kx = 111132.92-559.82*cos(2*lat0)+1.175*cos(4*lat0)-0.0023*cos(6*lat0); // meters per degree of latitude
        ky = 111412.84*cos(lat0)-93.5*cos(3*lat0)+0.118*cos(5*lat0); // meters per degree of longitude
    }

		void updateNavigation()
		{
			// ----------------- State machine -----------------

			if (rocket_fsm.state_machine.compare("Launch") == 0 || rocket_fsm.state_machine.compare("Rail") == 0||rocket_fsm.state_machine.compare("Coast") == 0)
			{
                predict_step();

                //update_step_fsen(Z_fakesensor.segment(3,3));

            }

			// Parse navigation state and publish it on the /nav_pub topic
			real_time_simulator::State rocket_state;

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

			rocket_state.twist.angular.x = X(10);
			rocket_state.twist.angular.y = X(11);
			rocket_state.twist.angular.z = X(12);

			rocket_state.propeller_mass = X(13);

			nav_pub.publish(rocket_state);

            real_time_simulator::StateCovariance state_covariance;

            //std::vector<Vector3d> P_vec(NX*NX);
            Matrix<double,NX,1> P_diag;
            P_diag = P.diagonal();
            std::vector<double> P_vec(P_diag.data(), P_diag.data() + NX);
            std::vector<double> P_quat(P.block(6,6,4,4).data(), P.block(6,6,4,4).data() + 4*4);
            state_covariance.covariance = P_vec;
            state_covariance.quat_covariance = P_quat;
            cov_pub.publish(state_covariance);
		}
};


int main(int argc, char **argv)
{
	// Init ROS time keeper node
	ros::init(argc, argv, "data_fusion");
	ros::NodeHandle nh;

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
