/*
* Node to send reference trajectory to be tracked by rocket_control node. 
* Also shutdown the engine at the right time to reach apogee altitude
*
* Inputs: 
*   - Finite state machine from the rocket_fsm :	    /gnc_fsm_pub
*   - Estimated state from rocket_navigation:		      /kalman_rocket_state
*
* Important parameters:
*   - Rocket model: 		  /config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
*
* Outputs:
*   - Reference trajectory:                           /target_trajectory
*   - Zero force and torque at engine shutdown time:  /control_pub
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

class GuidanceNode{
  private:
      // Class with useful rocket parameters and methods
      Rocket rocket;

      // Last received rocket state
      real_time_simulator::State current_state;

      // Last requested fsm 
      real_time_simulator::FSM rocket_fsm;
      real_time_simulator::GetFSM srv_fsm;

      // List of subscribers and publishers
      ros::Publisher target_trajectory_pub;
	    ros::Publisher control_pub;

      ros::Subscriber rocket_state_sub;
      ros::Subscriber fsm_sub;

      ros::ServiceClient client_fsm;

    public:

      GuidanceNode(ros::NodeHandle &nh)
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
        // Create waypoint trajectory publisher
        target_trajectory_pub = nh.advertise<real_time_simulator::Trajectory>("target_trajectory", 10);

        // Create control publisher
	      control_pub = nh.advertise<real_time_simulator::Control>("control_pub", 1);

        // Subscribe to state message from basic_gnc
        rocket_state_sub = nh.subscribe("kalman_rocket_state", 1, &GuidanceNode::rocket_stateCallback, this);

        // Subscribe to fsm and time from time_keeper
        fsm_sub = nh.subscribe("gnc_fsm_pub", 1, &GuidanceNode::fsm_Callback, this);

        // Setup Time_keeper client and srv variable for FSM and time synchronization
        client_fsm = nh.serviceClient<real_time_simulator::GetFSM>("getFSM_gnc");
      }

      // Callback function to store last received state
      void rocket_stateCallback(const real_time_simulator::State::ConstPtr& rocket_state)
      {
        current_state.pose = rocket_state->pose;
        current_state.twist = rocket_state->twist;
        current_state.propeller_mass = rocket_state->propeller_mass;
      }


      void fsm_Callback(const real_time_simulator::FSM::ConstPtr& fsm)
      {
        rocket_fsm.state_machine = fsm->state_machine;
        rocket_fsm.time_now = fsm->time_now;
      }

      // Creates very basic trajectory to reach desired points at apogee using affine functions
      real_time_simulator::Trajectory linear_trajectory()
      {
        float final_time = 20;
        static int n_point_guidance = 10;
        float dT = final_time - rocket_fsm.time_now;

        // Define affine parameters for position trajectory
        float a_x = (rocket.target_apogee[0] - current_state.pose.position.x)/dT;
        float a_y = (rocket.target_apogee[1] - current_state.pose.position.y)/dT;
        float a_z = (rocket.target_apogee[2] - current_state.pose.position.z)/dT;

        float b_x = rocket.target_apogee[0] - a_x*final_time;
        float b_y = rocket.target_apogee[1] - a_y*final_time;
        float b_z = rocket.target_apogee[2] - a_z*final_time;

        // Fill the trajectory points' position
        real_time_simulator::Trajectory trajectory_msg;
        int i = 0;
        for(i = 0; i<n_point_guidance; i++)
        {
          real_time_simulator::Waypoint waypoint;

          waypoint.time = rocket_fsm.time_now + i*dT/(n_point_guidance-1);

          waypoint.position.x = a_x*waypoint.time + b_x;
          waypoint.position.y = a_y*waypoint.time + b_y;
          waypoint.position.z = a_z*waypoint.time + b_z;

          trajectory_msg.trajectory.push_back(waypoint);
        }
        return trajectory_msg;
      }


      void updateGuidance()
      {
        // Get current FSM and time 
        if(client_fsm.call(srv_fsm))
        {
          rocket_fsm = srv_fsm.response.fsm;
        }
      
        // ----------------- State machine -----------------
        if (rocket_fsm.state_machine.compare("Idle") == 0)
        {
          // Do nothing
        }
    
        else if (rocket_fsm.state_machine.compare("Coast") != 0)
        {

          if (rocket_fsm.state_machine.compare("Rail") == 0)
          {
            // Do nothing
          }    
    
          else if (rocket_fsm.state_machine.compare("Launch") == 0)
          {

            float predicted_apogee = rocket.check_apogee(current_state.propeller_mass+rocket.dry_mass, current_state.pose.position.z, current_state.twist.linear.z);
            if(predicted_apogee>rocket.target_apogee[2])
            {
              // Send zero force&torque control command
              real_time_simulator::Control control_law;
              control_pub.publish(control_law);
            }

            // Send full optimal state as waypoint trajectory     
            target_trajectory_pub.publish(linear_trajectory()); 
          }
          
        }
      }

};
 




int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "guidance");
  ros::NodeHandle nh;

  GuidanceNode guidanceNode(nh);

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = nh.createTimer(ros::Duration(0.200), [&](const ros::TimerEvent&) 
	{
    guidanceNode.updateGuidance();
  });
 
	// Automatic callback of service and publisher from here
	ros::spin(); 
}