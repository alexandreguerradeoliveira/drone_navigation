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


#include "geometry_msgs/Vector3.h"

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
		static const int NX = 33;

		static const int NZBARO = 1;
        static const int NZGPS = 6;
        static const int NZMAG = 3;
        static const int NZACC = 3;


        //static const int NZOT = 10;
		
		
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


        // Sensor mesurement model matrices

    	typedef Eigen::Matrix<double, NX, NX> state_matrix;
    	typedef Eigen::Matrix<double, NZBARO, NZBARO> sensor_matrix_baro;
        typedef Eigen::Matrix<double, NZGPS, NZGPS> sensor_matrix_gps;
        typedef Eigen::Matrix<double, NZMAG, NZMAG> sensor_matrix_mag;
        typedef Eigen::Matrix<double, NZACC, NZACC> sensor_matrix_acc;


    /// AUTODIFF stuff end

	private:
        // sensor received flag
        bool imu_flag = false;
        bool gps_flag = false;

        sensor_data_gps gps_data;
        sensor_data_baro z_baro;
        sensor_data_mag mag_data;


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


        // Kalman matrix
        sensor_matrix_baro R_baro;
        sensor_matrix_gps R_gps;
        sensor_matrix_mag R_mag;
        sensor_matrix_acc R_acc;


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



    // Kalman state
		state X;
		ad_state ADx; // state used for autodiff
		
		
		
		/// Fake GPS
		// Last received fake GPS data
		Eigen::Matrix<double, 3, 1> gps_pos;
        Eigen::Matrix<double, 3, 1> gps_vel;
		double gps_noise_xy = .3;
		double gps_noise_z = .3;
		double gps_freq = 10;	
		double gps_noise_xy_vel = .3;
		double gps_noise_z_vel = .3;
		// end fakegps
	
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

			roll *= DEG2RAD; zenith *= DEG2RAD; azimuth *= DEG2RAD;

			typedef Eigen::EulerSystem<-Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_Z> Rail_system;
			typedef Eigen::EulerAngles<double, Rail_system> angle_type;

			angle_type init_angle(azimuth, zenith, roll);

			Eigen::Quaterniond q(init_angle);

			/// Init state X
			//X << 0, 0, 0,   0, 0, 0,     0.0, 0.0 , 0.0 , 1.0 ,     0.0, 0.0, 0.0,    rocket.propellant_mass;
            X.setZero();
            X(13) = rocket.propellant_mass;
			X.segment(6,4) = q.coeffs();
            X.segment(30,3) << 1.0,0.0,0.0;

            /// Initialize kalman parameters
            // sensor covarience matrices
            R_baro(0,0) = 0.1;

            R_gps.setIdentity();
            R_gps = R_gps*0.3;

            R_mag.setIdentity();
            R_mag = R_mag*0.001;

            R_acc.setIdentity()*100;

            P.setZero(); // no error in the initial state

            // process covariance matrix
			Q.setIdentity();
            Q = Q*0.000000001;
            Q(0,0) = 0.2;
            Q(1,1) = 0.2;
            Q(2,2) = 0.2;

            Q(3,3) = 0.2;
            Q(4,4) = 0.2;
            Q(5,5) = 0.2;

            Q(6,6) = 0.00001;
            Q(7,7) = 0.00001;
            Q(8,8) = 0.00000001;
            Q(9,9) = 0.00000001;

            Q(13,13) = 0.00025;


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
			// Create filtered rocket state publisher
			nav_pub = nh.advertise<real_time_simulator::State>("kalman_rocket_state", 10);

            // Create state covarience publisher
            cov_pub = nh.advertise<real_time_simulator::StateCovariance>("state_covariance", 10);

			// Subscribe to time_keeper for fsm and time
			fsm_sub = nh.subscribe("gnc_fsm_pub", 1, &NavigationNode::fsmCallback, this);

			// Subscribe to control for kalman estimator
			control_sub = nh.subscribe("control_measured", 1, &NavigationNode::controlCallback, this);

			// Subscribe to sensor for kalman correction
			sensor_sub = nh.subscribe("sensor_pub", 1, &NavigationNode::sensorCallback, this);

            // Subscribe to state to fake GPS
            gps_sub = nh.subscribe("rocket_state", 1, &NavigationNode::gpsCallback, this);
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

		// Callback function to store last received sensor data
		void sensorCallback(const real_time_simulator::Sensor::ConstPtr& sensor)
		{
			rocket_sensor.IMU_acc = sensor->IMU_acc;
			rocket_sensor.IMU_gyro = sensor->IMU_gyro;
			rocket_sensor.IMU_mag = sensor->IMU_mag;
			
			rocket_sensor.baro_height = sensor->baro_height;

            z_baro(0) = rocket_sensor.baro_height;

            mag_data << rocket_sensor.IMU_mag.x,rocket_sensor.IMU_mag.y,rocket_sensor.IMU_mag.z;

            //sensor_data_acc acc_data;
            //acc_data << rocket_sensor.IMU_acc.x,rocket_sensor.IMU_acc.y,rocket_sensor.IMU_acc.z;

            //double g0 = 9.81;  // Earth gravity in [m/s^2]
            //if(((acc_data).norm()<=1.1*g0)&&((acc_data).norm()>=0.9*g0)){
            //    std::cout << "updating with acc" << std::endl;
            //    std::cout << acc_data << std::endl;
            //    update_step_acc(acc_data);
            //}

            imu_flag = true;

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

                        gps_flag = true;

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
			Eigen::Matrix<T, 3, 1> IMU_acc; IMU_acc << (T) rocket_sensor.IMU_acc.x-x(17),(T) rocket_sensor.IMU_acc.y-x(18),(T) rocket_sensor.IMU_acc.z-x(19);

			// Angular velocity omega in quaternion format to compute quaternion derivative
            Eigen::Quaternion<T> omega_quat(0.0, rocket_sensor.IMU_gyro.x-x(14), rocket_sensor.IMU_gyro.y-x(15), rocket_sensor.IMU_gyro.z-x(16));

            //Inertia
            Matrix<T, 3, 1> I_inv;
            I_inv << 1 / rocket.total_Inertia[0], 1 / rocket.total_Inertia[1], 1 / rocket.total_Inertia[2];
            Matrix<T, 3, 1> I;
            I<< rocket.total_Inertia[0], rocket.total_Inertia[1],rocket.total_Inertia[2];

            Eigen::Matrix<T, 3, 1> dist_torque;
            dist_torque << x(27),x(28),x(29);

            // compute total torque in body frame
            Eigen::Matrix<T, 3, 1> total_torque_body;
            total_torque_body = control_torque + rot_matrix.transpose()*dist_torque;

            Eigen::Matrix<T, 3, 1> omega;
            omega << rocket_sensor.IMU_gyro.x-x(14), rocket_sensor.IMU_gyro.y-x(15), rocket_sensor.IMU_gyro.z-x(16);

            // -------------- Differential equation ---------------------

			// Position variation is speed
			xdot.head(3) = x.segment(3,3);

			// Speed variation is acceleration
			xdot.segment(3,3) =  rot_matrix*IMU_acc - Eigen::Vector3d::UnitZ().template cast<T>() *g0;

			// Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
            xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();

			// Angular speed
            xdot.segment(10, 3) = rot_matrix*(total_torque_body - omega.cross(I.template cast<T>().cwiseProduct(omega))).cwiseProduct(I_inv.template cast<T>());





			// Mass variation is proportional to total thrust (! autodiff .norm() gives nan)
            if(rocket_force.template cast<T>().norm()<0.01){
                xdot(13) = -(rocket_force.template cast<T>()(2))/((T)rocket.Isp*g0);
            }else{
                xdot(13) = -(rocket_force.template cast<T>().norm())/((T)rocket.Isp*g0);
            }

            //Bias for gyroscope, accelerometer, magnetometer, barometer - bias variation follows a static update
            xdot.segment(14, 10) << 0, 0, 0,  0, 0, 0,  0, 0, 0,  0;

            // Disturbance forces
            xdot.segment(24, 3) << 0, 0, 0;

            // Disturbance moments
            xdot.segment(27, 3) << 0, 0, 0;

            // Magnetic vector in earth's frame
            xdot.segment(30, 3) << 0, 0, 0;
			
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
        dist_force << x(24),x(25),x(26);

        Eigen::Matrix<T, 3, 1> dist_torque;
        dist_torque << x(27),x(28),x(29);

        // compute total force inertial frame
        Eigen::Matrix<T, 3, 1> total_force_inertial;
        total_force_inertial = rot_matrix*(control_force) - Eigen::Vector3d::UnitZ().template cast<T>()*x(13)*g0 + dist_force;

        // compute total torque in body frame
        Eigen::Matrix<T, 3, 1> total_torque_body;
        total_torque_body = control_torque + rot_matrix.transpose()*dist_torque;

        Eigen::Matrix<T, 3, 1> omega;
        omega << x(10),x(11),x(12);

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat(0.0, x(10)-x(14), x(11)-x(15), x(12)-x(16));

        //Inertia
        Matrix<T, 3, 1> I_inv;
        I_inv << 1 / rocket.total_Inertia[0], 1 / rocket.total_Inertia[1], 1 / rocket.total_Inertia[2];
        Matrix<T, 3, 1> I;
        I << rocket.total_Inertia[0], rocket.total_Inertia[1],rocket.total_Inertia[2];

        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3,3);

        // Speed variation is acceleration given by newton's law
        xdot.segment(3,3) =  total_force_inertial/x(13);

        // Quaternion variation is 0.5*q*omega_quat if omega is in the body frame
        xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();

        // Angular speed variation is given by euler's equation if in body frame
        //xdot.segment(10, 3) = I_inv( -omega*(I*omega) + total_torque_body); // in body frame
        //xdot.segment(10, 3) = rot_matrix*(I_inv*( -omega.cross(I*omega) + total_torque_body)); // in inertial frame
        xdot.segment(10, 3) = rot_matrix*(total_torque_body - omega.cross(I.template cast<T>().cwiseProduct(omega))).cwiseProduct(I_inv.template cast<T>());


        // Mass variation is proportional to total thrust (! autodiff .norm() gives nan)
        if(control_force.template cast<T>().norm()<0.01){
            xdot(13) = -(control_force.template cast<T>()(2))/((T)rocket.Isp*g0);
        }else{
            xdot(13) = -(control_force.template cast<T>().norm())/((T)rocket.Isp*g0);
        }

        //Bias for gyroscope, accelerometer, magnetometer, barometer - bias variation follows a static update
        xdot.segment(14, 10) << 0, 0, 0,  0, 0, 0,  0, 0, 0,  0;

        // Disturbance forces
        xdot.segment(24, 3) << 0, 0, 0;

        // Disturbance moments
        xdot.segment(27, 3) << 0, 0, 0;

        // Magnetic vector in earth's frame
        xdot.segment(30, 3) << 0, 0, 0;

    }

        template<typename T>
        void mesurementModelBaro(const state_t<T> &x, sensor_data_baro_t<T> &z) {
            z(0) = x(2)+x(23);
        }

        template<typename T>
        void mesurementModelMag(const state_t<T> &x, sensor_data_mag_t<T> &z) {
            // get rotation matrix
            Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
            attitude.normalize();
            Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

            // express inertial magnetic vector estimate in body-frame and add bias
            z = rot_matrix.transpose()*(x.segment(30,3)) + x.segment(20,3);
        }

        template<typename T>
        void mesurementModelAcc(const state_t<T> &x, sensor_data_acc_t<T> &z) {
            T g0 = (T) 9.81;  // Earth gravity in [m/s^2]
            // get rotation matrix
            Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
            attitude.normalize();
            Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();
            Eigen::Matrix<T, 3, 1> gravity_vec = Eigen::Vector3d::UnitZ().template cast<T>() *(-g0);

            // express inertial magnetic vector estimate in body-frame and add bias
            z = rot_matrix.transpose()*(gravity_vec) + x.segment(17,3);
    }

        template<typename T>
        void mesurementModelGPS(const state_t<T> &x, sensor_data_gps_t<T> &z) {
            z = x.segment(0,6);
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

                // update rotation speed to those of gyro
                Matrix<double,3,1>omega_b;
                Eigen::Quaternion<double> attitude(X(9), X(6), X(7), X(8));
                attitude.normalize();
                Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
                omega_b <<rocket_sensor.IMU_gyro.x-X(14), rocket_sensor.IMU_gyro.y-X(15), rocket_sensor.IMU_gyro.z-X(16);
                X.segment(10,3) = rot_matrix*omega_b;

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
			static double last_predict_time = ros::Time::now().toSec();
			double dT = ros::Time::now().toSec() - last_predict_time;
			RK4(dT,X,P,X,P);
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

            // compute EKF update
            // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
            Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
            Eigen::Matrix<double, NZBARO, NZBARO> S; // innovation covariance
            Eigen::Matrix<double, NX, NZBARO> K; // Kalman gain
            Eigen::Matrix<double, NX, NX> I; // identity

            I.setIdentity();
            S = H_baro * P * H_baro.transpose() + R_baro;
            K = S.llt().solve(H_baro * P).transpose();
            X = X + K * (z - h_x);

            //std::cout << K << std::endl << std::endl;

            IKH = (I - K * H_baro);
            P = IKH * P * IKH.transpose() + K * R_baro * K.transpose();

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

        // compute EKF update
        // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
        Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
        Eigen::Matrix<double, NZGPS, NZGPS> S; // innovation covariance
        Eigen::Matrix<double, NX, NZGPS> K; // Kalman gain
        Eigen::Matrix<double, NX, NX> I; // identity

        I.setIdentity();
        S = H_gps * P * H_gps.transpose() + R_gps;
        K = S.llt().solve(H_gps * P).transpose();
        X = X + K * (z - h_x);

        //std::cout << H_gps << std::endl << std::endl;

        IKH = (I - K * H_gps);
        P = IKH * P * IKH.transpose() + K * R_gps * K.transpose();

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

        // compute EKF update
        // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
        Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
        Eigen::Matrix<double, NZMAG, NZMAG> S; // innovation covariance
        Eigen::Matrix<double, NX, NZMAG> K; // Kalman gain
        Eigen::Matrix<double, NX, NX> I; // identity

        I.setIdentity();
        S = H_mag * P * H_mag.transpose() + R_mag;
        K = S.llt().solve(H_mag * P).transpose();
        X = X + K * (z - h_x);

        //std::cout << K << std::endl << std::endl;

        IKH = (I - K * H_mag);
        P = IKH * P * IKH.transpose() + K * R_mag * K.transpose();

    }

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

        // compute EKF update
        // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
        Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
        Eigen::Matrix<double, NZACC, NZACC> S; // innovation covariance
        Eigen::Matrix<double, NX, NZACC> K; // Kalman gain
        Eigen::Matrix<double, NX, NX> I; // identity

        I.setIdentity();
        S = H_acc * P * H_acc.transpose() + R_acc;
        K = S.llt().solve(H_acc * P).transpose();
        X = X + K * (z - h_x);

        //std::cout << K << std::endl << std::endl;

        IKH = (I - K * H_acc);
        P = IKH * P * IKH.transpose() + K * R_acc * K.transpose();

    }

		void updateNavigation()
		{
			// ----------------- State machine -----------------
			if (rocket_fsm.state_machine.compare("Idle") == 0)
			{
				// Do nothing
			}

			else if (rocket_fsm.state_machine.compare("Launch") == 0 || rocket_fsm.state_machine.compare("Rail") == 0)
			{
                if(imu_flag){
                    update_step_baro(z_baro);
                    update_step_mag(mag_data);
                    imu_flag = false;
                }
                if(gps_flag){
                    update_step_gps(gps_data);
                    gps_flag = false;
                }
                predict_step();

            }

			else if (rocket_fsm.state_machine.compare("Coast") == 0)
			{
                if(imu_flag){
                    update_step_baro(z_baro);
                    update_step_mag(mag_data);
                    imu_flag = false;
                }
                if(gps_flag){
                    update_step_gps(gps_data);
                    gps_flag = false;
                }
                predict_step();

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
            //std::cout<< X.segment(14+9,1)<< std::endl<< std::endl;

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
	ros::Timer control_thread = nh.createTimer(ros::Duration(0.005),
	[&](const ros::TimerEvent&) 
	{
		navigationNode.updateNavigation();
	});

	// Automatic callback of service and publisher from here
	ros::spin();
}
