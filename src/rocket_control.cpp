/*
* Node to send control commands to the rocket engine. 
* Can also be used by the simulation for SIL and PIL tests.
*
* Inputs: 
*   - Finite state machine from the rocket_fsm :	    /gnc_fsm_pub
*   - Estimated state from rocket_navigation:		      /kalman_rocket_state
*
* Important parameters:
*   - Rocket model: 		  /config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
*	  - P gain: 		        P_control()
*   - Control loop period control_thread()
*
* Outputs:
*   - Commanded 3D force and torque for the rocket engine:  /control_pub
*
*/

#include "ros/ros.h"

#include <ros/package.h>

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Waypoint.h"
#include "real_time_simulator/Trajectory.h"

#include "real_time_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include "real_time_simulator/GetFSM.h"
#include "real_time_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#include <iomanip>
#include <iostream>
#include <chrono>

#include "rocket_model.hpp"

class ControlNode {
  private:
      // Class with useful rocket parameters and methods
      Rocket rocket;

      // Last received rocket state
      real_time_simulator::State rocket_state;

      // Last requested fsm
      real_time_simulator::FSM rocket_fsm;

      // List of subscribers and publishers
      ros::Publisher control_pub;

      ros::Subscriber rocket_state_sub;
      ros::Subscriber fsm_sub;

      ros::ServiceClient client_fsm;
      real_time_simulator::GetFSM srv_fsm;

    public:

      ControlNode(ros::NodeHandle &nh)
      {
        // Initialize publishers and subscribers
        initTopics(nh);

        // Initialize fsm
        rocket_fsm.time_now = 0;
        rocket_fsm.state_machine = "Idle";

        // Initialize rocket class with useful parameters
        rocket.init(nh);
      }

      void initTopics(ros::NodeHandle &nh) 
      {
        // Create control publisher
        control_pub = nh.advertise<real_time_simulator::Control>("control_pub", 10);

        // Subscribe to state message from basic_gnc
        rocket_state_sub = nh.subscribe("kalman_rocket_state", 1, &ControlNode::rocket_stateCallback, this);

        // Subscribe to fsm and time from time_keeper
        fsm_sub = nh.subscribe("gnc_fsm_pub", 1, &ControlNode::fsm_Callback, this);

        // Setup Time_keeper client and srv variable for FSM and time synchronization
        client_fsm = nh.serviceClient<real_time_simulator::GetFSM>("getFSM_gnc");

      }

      // Callback function to store last received state
      void rocket_stateCallback(const real_time_simulator::State::ConstPtr& new_rocket_state)
      {
        rocket_state.pose = new_rocket_state->pose;
        rocket_state.twist = new_rocket_state->twist;
        rocket_state.propeller_mass = new_rocket_state->propeller_mass;
      }

      void fsm_Callback(const real_time_simulator::FSM::ConstPtr& fsm)
      {
        rocket_fsm.state_machine = fsm->state_machine;
        rocket_fsm.time_now = fsm->time_now;
      }

      //real_time_simulator::Control P_control()
      //{
        // Init control message
       // real_time_simulator::Control control_law;
        //geometry_msgs::Vector3 thrust_force;
        //geometry_msgs::Vector3 thrust_torque;

        //thrust_force.x = -200*rocket_state.twist.angular.y;
        //thrust_force.y = -200*rocket_state.twist.angular.x;
        //thrust_force.x = 0.0;
        //thrust_force.y = 0.0;
        //thrust_force.z = rocket.get_full_thrust(rocket_fsm.time_now);

        //thrust_torque.x = thrust_force.y*rocket.total_CM;
        //thrust_torque.y = thrust_force.x*rocket.total_CM; // might be a minus
        //thrust_torque.z = -10*rocket_state.twist.angular.z;
        //thrust_torque.z = 0.0;


        //control_law.force = thrust_force;
        //control_law.torque = thrust_torque;
        //control_law.force.x = 0;
        //control_law.force.y = 0;
        //control_law.force.z = 0;

          ///return control_law;
      //}

    real_time_simulator::Control P_control()
    {
        // Init control message
        real_time_simulator::Control control_law;
        geometry_msgs::Vector3 thrust_force;
        geometry_msgs::Vector3 thrust_torque;

        static float angularX_sum = 0;
        static float angularY_sum = 0;

        angularX_sum += rocket_state.twist.angular.x;
        angularY_sum += rocket_state.twist.angular.y;


        thrust_force.x = 0;
        thrust_force.y = 0;
        thrust_force.z = 10*(20-rocket_state.pose.position.z) + 9.81*(rocket_state.propeller_mass+rocket.dry_mass) - 10*rocket_state.twist.linear.z;

        thrust_torque.x = 2*rocket_state.twist.linear.y - 100*rocket_state.twist.angular.x;
        thrust_torque.y = -2*rocket_state.twist.linear.x - 100*rocket_state.twist.angular.y;
        thrust_torque.z = -10*rocket_state.twist.angular.z;

        control_law.force = thrust_force;
        control_law.torque = thrust_torque;

        return control_law;
    }


      void updateControl()
      {
        // Init default control to zero
        real_time_simulator::Control control_law;

        //Get current FSM and time
        if(client_fsm.call(srv_fsm))
        {
          rocket_fsm = srv_fsm.response.fsm;
        }
      
        // ----------------- State machine -----------------
        if (rocket_fsm.state_machine.compare("Idle") == 0)
        {
          // Do nothing
        }

        else 
        {
          if (rocket_fsm.state_machine.compare("Rail") == 0)
          {
            control_law = P_control();
          }

          else if (rocket_fsm.state_machine.compare("Launch") == 0)
          {
            control_law = P_control();
          }

          else if (rocket_fsm.state_machine.compare("Coast") == 0)
          {

          }
        
          control_pub.publish(control_law);
        }
      }

      

};






int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;

  ControlNode controlNode(nh);
	
  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = nh.createTimer(ros::Duration(0.05), [&](const ros::TimerEvent&) 
	{
    controlNode.updateControl();
    
  });

	// Automatic callback of service and publisher from here
	ros::spin();

}

