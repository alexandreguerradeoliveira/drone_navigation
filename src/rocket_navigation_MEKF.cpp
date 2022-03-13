/*
* Node to estimate the rocket full state (position, velocity, attitude quaternion, angular rate and mass) 
* from the sensor data and commanded thrust and torque of the rocket engine
*
* Inputs: 
*   - Finite state machine from the test_rocket_fsm :	     /gnc_fsm_pub
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
#include <random>

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <unsupported/Eigen/EulerAngles>

#include "rocket_model.hpp"

#include <type_traits>

#define DEG2RAD 0.01745329251


class NavigationNode {
	public:
		typedef Eigen::Matrix<double, 23, 1> state_sim;
		typedef Eigen::Matrix<double, 22, 1> state;
		
		
		typedef Eigen::Matrix<double, 22, 22> state_matrix;
		typedef Eigen::Matrix<double, 22, 22> sensor_matrix;


	private:
		// Class with useful rocket parameters and methods
		Rocket rocket;

		// Last received fsm
		real_time_simulator::FSM rocket_fsm;

		// Last received control
		real_time_simulator::Control rocket_control;

		// Last received sensor data
		real_time_simulator::Sensor rocket_sensor;

		/// Fake GPS
		// Last received fake GPS data
		Eigen::Matrix<double, 3, 1> gps_pos;
	      	Eigen::Matrix<double, 3, 1> gps_vel;
		double gps_noise_xy = 2.0;
		double gps_noise_z = 2;
		double gps_freq = 2;
		double gps_noise_xy_vel = 2.0;
		double gps_noise_z_vel = 2.0;
		// end fakegps
		
		//Multiplicative EFK matrices
		//state_matrix Q;
		Eigen::Matrix<double,3,3> R_mag;
		Eigen::Matrix<double,1,1> R_barro;
		Eigen::Matrix<double,6,6> R_gps;
		
		state_matrix F;
		
		//Eigen::Matrix<double,22,18> G;
		
		Eigen::Matrix<double,22,22> Q;
		
		state_matrix P;
		
		Eigen::Matrix<double,6,22> H_gps;
		Eigen::Matrix<double,1,22> H_barro;
		Eigen::Matrix<double,3,22> H_mag;
		

		// Kalman state
		state_sim X;

		//Kalman delta_state
		state delta_x;
		
		
	
		// List of subscribers and publishers
		ros::Publisher nav_pub;
        ros::Publisher cov_pub;


    ros::Subscriber fsm_sub;
		ros::Subscriber control_sub;
		ros::Subscriber rocket_state_sub;
		ros::Subscriber sensor_sub;
		ros::Subscriber gps_sub;


	
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

			// Init state X  
			X << 0, 0, 0,   0, 0, 0,     0.0, 0.0 , 0.0 , 1.0 ,     0.0, 0.0, 0.0,    rocket.propellant_mass, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0;
			X.segment(6,4) = q.coeffs();
			delta_x << 0,0,0,  0,0,0,  0,0,0,  0,0,0, 0,0,0,  0,0,0;
			
			
			P.setZero();
			
			
			Q.setZero();
			Q.block(0,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3)*0.00001;
            Q(2,2) = 2;

			Q.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3)*0.05;
            Q(5,5) = 1;
			
			Q.block(6,6,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3)*0.0001;
			
			Q.block(13,13,9,9) = Eigen::Matrix<double, 9, 9>::Identity(9, 9)*0.00001;

            Q(12,12) = 0.0025;



            R_mag.setIdentity()*0.2*0.2;
			
			R_barro(0,0) = 0.1*0.1;
			
			R_gps.setIdentity();
			R_gps(0,0) = 2*2;
			R_gps(1,1) = 2*2;
			R_gps(2,2) = 2*2;
			R_gps(3,3) = 2*2;
			R_gps(4,4) = 2*2;
			R_gps(5,5) = 2*2;


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
			rocket_sensor.baro_height = sensor->baro_height;
			rocket_sensor.IMU_mag = sensor->IMU_mag;

			Eigen::Matrix<double,3,1> IMU_mag;
			IMU_mag << rocket_sensor.IMU_mag.x,rocket_sensor.IMU_mag.y,rocket_sensor.IMU_mag.z;
			
			update_step_barro(rocket_sensor.baro_height,X,delta_x);
			update_step_mag(IMU_mag,X,delta_x);
			
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
			
			update_step_gps(gps_pos,gps_vel,X, delta_x);
			
			last_predict_time_gps = ros::Time::now().toSec();

			};

		}


		/* ------------ User functions ------------ */
	
		void state_dynamics(const state_sim x, state_sim &xdot)
		{
			// -------------- Simulation variables -----------------------------
			double g0 = 9.81;  // Earth gravity in [m/s^2]

			Eigen::Matrix<double, 3, 1> rocket_force;
			rocket_force << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;

			// Orientation of the rocket with quaternion
			Eigen::Quaternion<double> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

			// Current acceleration and angular rate from IMU
			Eigen::Matrix<double, 3, 1> IMU_acc; IMU_acc << rocket_sensor.IMU_acc.x, rocket_sensor.IMU_acc.y, rocket_sensor.IMU_acc.z;
			IMU_acc += x.segment(17,3);
			
			Eigen::Matrix<double, 3, 1> IMU_gyro;IMU_gyro << rocket_sensor.IMU_gyro.x, rocket_sensor.IMU_gyro.y, rocket_sensor.IMU_gyro.z;
			IMU_gyro += x.segment(14,3);
			

			// Angular velocity omega in quaternion format to compute quaternion derivative
			Eigen::Quaternion<double> omega_quat(0.0, IMU_gyro(0), IMU_gyro(1), IMU_gyro(2));
			
			// -------------- Differential equation ---------------------

			// Position variation is speed
			xdot.head(3) = x.segment(3,3);

			// Speed variation is acceleration
			xdot.segment(3,3) =  rot_matrix*IMU_acc - Eigen::Vector3d::UnitZ()*g0;

			// Quaternion variation is 0.5*w◦q
			//xdot.segment(6, 4) =  0.5*(omega_quat*attitude).coeffs();
            xdot.segment(6, 4) =  0.5*(attitude*omega_quat).coeffs();


            // Angular speed assumed to be constant between two measure
			xdot.segment(10, 3) << 0, 0, 0;

				
			// Mass variation is proportional to total thrust
			xdot(13) = -rocket_force.norm()/(rocket.Isp*g0);
			
			
			//Constant bias for IMU
			xdot.segment(14, 9) << 0, 0, 0,  0, 0, 0,  0, 0, 0;
			

			
		}
		
		void setup_F(const state_sim x, state_matrix &F)
		{
			///Setup F matrix
			
			F.setZero();
			
			//position vatiation is velocity
			F.block(0,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
			
			
			//velocity variation is acceleration plus bias in inertial frame
			Eigen::Quaternion<double> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
			
			Eigen::Matrix<double, 3, 1> IMU_acc; IMU_acc << rocket_sensor.IMU_acc.x, rocket_sensor.IMU_acc.y, rocket_sensor.IMU_acc.z;			
			Eigen::Matrix<double, 3, 3> skew_sym_acc;
			skew_sym_acc << 0.0,-IMU_acc(2),IMU_acc(1),
				IMU_acc(2),0.0,-IMU_acc(0),
				-IMU_acc(1),IMU_acc(0),0.0;
							
			F.block(3,6,3,3) = -rot_matrix*skew_sym_acc;
			F.block(3,16,3,3) = -rot_matrix;
			
			// attitude variation
			
			Eigen::Matrix<double, 3, 1> IMU_gyro; IMU_gyro << rocket_sensor.IMU_gyro.x, rocket_sensor.IMU_gyro.y, rocket_sensor.IMU_gyro.z;			
			Eigen::Matrix<double, 3, 3> skew_sym_gyro;
			skew_sym_gyro << 0.0,-IMU_gyro(2),IMU_gyro(1),
				IMU_gyro(2),0.0,-IMU_gyro(0),
				-IMU_gyro(1),IMU_gyro(0),0.0;
				
			F.block(6,6,3,3) = -skew_sym_gyro;
			F.block(6,13,3,3) = -Eigen::Matrix<double, 3, 3>::Identity(3, 3);
			
			// Mass variation
			double g0 = 9.81;  // Earth gravity in [m/s^2]

			Eigen::Matrix<double, 3, 1> rocket_force;
			rocket_force << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;
			
			F(12,12) = -rocket_force.norm()/(rocket.Isp*g0);
			
			
		}
		
		void reset_state(state_sim &x,state &delta_x)
		{
			/// propagate delta_x mesurements into x
			
			// position and velocity
			x.head(3) += delta_x.head(3); 
			x.segment(3,3) += delta_x.segment(3,3);
			
			// set new quaternion reference
			Eigen::Quaternion<double> attitude(x(9), x(6), x(7), x(8));
			Eigen::Quaternion<double> delta_q(1, delta_x(6)/2.0, delta_x(7)/2.0, delta_x(8)/2.0);
			//attitude = delta_q*attitude;
			attitude = attitude*delta_q;
			
			attitude.normalize();
			x.segment(6,4) = attitude.coeffs();
			
			//set new angular rotation
			x.segment(10,3) += delta_x.segment(9,3);
			
			//set new mass
			x(13) += delta_x(12);
			
			//set new bias estimates for gyro accelerometer and magnetometer
			x.segment(14,9) += delta_x.segment(13,9);
			
						
			///reset delta_x
			
			//set error in position,velocity,attitude angles,angular velocity, mass, gyro bias, acc bias, mag bias to zero
			//delta_x << 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0, 0,0,0, 0,0,0, 0,0,0;
			delta_x.setZero();
		}
		


 		
 		void fullDerivative(const state_sim x,const state_matrix P,state_sim &xdot,state_matrix &Pdot) 
 		{
        		//X derivative
        		state_dynamics(x, xdot);
        		
        		
        		//P derivative
        		setup_F(x,F);
		        //Pdot = F * P + P * F.transpose() + G*Q*(G.transpose());
		  	    Pdot = F * P + P * F.transpose() + Q;
		  
		}
 		
 		
		
		void RK4(double dT)
		{
			state_sim k1, k2, k3, k4, X_inter;
			state_matrix k1_P, k2_P, k3_P, k4_P, P_inter;
		
			fullDerivative(X,      P,       k1,k1_P); 	X_inter = X+k1*dT/2; P_inter = P + k1_P*dT/2;
			fullDerivative(X_inter,P_inter, k2,k2_P); 	X_inter = X+k2*dT/2; P_inter = P + k2_P*dT/2;
			fullDerivative(X_inter,P_inter, k3,k3_P); 	X_inter = X+k3*dT;   P_inter = P + k3_P*dT;
			fullDerivative(X_inter,P_inter, k4,k4_P);
		
			X = X + (k1+2*k2+2*k3+k4)*dT/6;
			P = P + (k1_P + 2*k2_P + 2*k3_P + k4_P)*dT/6;

            // jenky stuff for the time beeing
            Eigen::Matrix<double,3,1>omega_b;
            Eigen::Quaternion<double> attitude(X(9), X(6), X(7), X(8));
            attitude.normalize();
            Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
            omega_b <<rocket_sensor.IMU_gyro.x-X(14), rocket_sensor.IMU_gyro.y-X(15), rocket_sensor.IMU_gyro.z-X(16);
            X.segment(10,3) = rot_matrix*omega_b;
		}
		
				
		void predict_step()
		{ 
			static double last_predict_time = ros::Time::now().toSec();
			
			reset_state(X,delta_x);

			double dT = ros::Time::now().toSec() - last_predict_time;
			RK4(dT); // integrate state and covariance
			
			last_predict_time = ros::Time::now().toSec();
			
			
		}
		
		void update_step_mag(const Eigen::Matrix<double,3,1> IMU_mag,const state_sim x, state &delta_x)
		{
			H_mag.setZero();
			Eigen::Quaternion<double> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
			
			Eigen::Matrix<double, 3, 1> mag_vector = {1.0, 0, 0};
			Eigen::Matrix<double, 3, 1> mag_vec_bodyframe = (rot_matrix.transpose())*mag_vector;
			
			Eigen::Matrix<double, 3, 3> skew_sym_mag_vec;
			skew_sym_mag_vec << 0.0,-mag_vec_bodyframe(2),mag_vec_bodyframe(1),
				mag_vec_bodyframe(2),0.0,-mag_vec_bodyframe(0),
				-mag_vec_bodyframe(1),mag_vec_bodyframe(0),0.0;
			
			H_mag.block(0,6,3,3) = skew_sym_mag_vec;
			H_mag.block(0,19,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
			
			
			
			Eigen::Matrix<double, 3, 1> mesurements;
			mesurements.block(0,0,3,1) = IMU_mag-(rot_matrix.transpose())*mag_vector-x.segment(19,3);


			// taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
			Eigen::Matrix<double, 22, 22> IKH;  // temporary matrix
        		Eigen::Matrix<double, 3, 3> S; // innovation covariance
        		Eigen::Matrix<double, 22, 3> K; // Kalman gain
        		Eigen::Matrix<double, 22, 22> I; // identity
        		I.setIdentity();
        		S = H_mag * P * H_mag.transpose() + R_mag;
        		K = S.llt().solve(H_mag * P).transpose();
        		delta_x = delta_x + K * (mesurements - H_mag*delta_x);
        		IKH = (I - K * H_mag);
        		P = IKH * P * IKH.transpose() + K * R_mag * K.transpose();
			
			
		}
		
		void update_step_barro(const double barro_z,const state_sim x, state &delta_x)
		{
			H_barro.setZero();
			H_barro(1,2) = 1.0;
			
			Eigen::Matrix<double, 1, 1> mesurements;
			mesurements(0) = barro_z-x(2);
			// taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
			Eigen::Matrix<double, 22, 22> IKH;  // temporary matrix
        		Eigen::Matrix<double, 1, 1> S; // innovation covariance
        		Eigen::Matrix<double, 22, 1> K; // Kalman gain
        		Eigen::Matrix<double, 22, 22> I; // identity
        		I.setIdentity();
        		S = H_barro * P * H_barro.transpose() + R_barro;
        		K = S.llt().solve(H_barro * P).transpose();
        		delta_x = delta_x + K * (mesurements - H_barro*delta_x);
        		IKH = (I - K * H_barro);
        		P = IKH * P * IKH.transpose() + K * R_barro * K.transpose();
			
			
			
		}
		
		void update_step_gps(const Eigen::Matrix<double,3,1> gps_pos,const Eigen::Matrix<double,3,1> gps_vel,const state_sim x, state &delta_x)
		{
			H_gps.setZero();
			
			H_gps.block(0,0,6,6) = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
			H_gps(2,2) = 0;

			
			//Eigen::Matrix<double,22,22> I;
			//I.setIdentity();
			Eigen::Matrix<double, 6, 1> mesurements;
			mesurements.setZero();

			mesurements.head(3) = gps_pos-x.head(3);
			mesurements(2) = 0.0;
			mesurements.segment(3,3) = gps_vel-x.segment(3,3);
			
			// taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
			Eigen::Matrix<double, 22, 22> IKH;  // temporary matrix
        		Eigen::Matrix<double, 6, 6> S; // innovation covariance
        		Eigen::Matrix<double, 22, 6> K; // Kalman gain
        		Eigen::Matrix<double, 22, 22> I; // identity
        		I.setIdentity();
        		S = H_gps * P * H_gps.transpose() + R_gps;
        		K = S.llt().solve(H_gps * P).transpose();
        		delta_x = delta_x + K * (mesurements - H_gps*delta_x);
        		IKH = (I - K * H_gps);
        		P = IKH * P * IKH.transpose() + K * R_gps * K.transpose();
			
			
			
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
				predict_step();
			}


			else if (rocket_fsm.state_machine.compare("Coast") == 0)
			{
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

			rocket_state.propeller_mass = X(13);

			nav_pub.publish(rocket_state);

            //std::cout << X << std::endl;

            real_time_simulator::StateCovariance state_covariance;

            //std::vector<Vector3d> P_vec(NX*NX);
            const int NX = 22;
            Eigen::Matrix<double,NX,1> P_diag;
            P_diag = P.diagonal();
            std::vector<double> P_vec(P_diag.data(), P_diag.data() + NX);
            state_covariance.covariance = P_vec;
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
