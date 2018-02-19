/*
 * Author: Connor Fuhrman
 * Date: 2/19/2018
 * Purpose: This code will interface with the ROS Navigation Stack. It will send commands to
 * the Arduino.
 *
 * Input: /cmd_vel topic consisting of
 */

/* Pseduocode:

subscribe to cmd_vel
vx = linear.x
wz = angular.z

vr = vx + wz*L/2
vl = vx - wz*L/2

Dr = 91*vr/vmax + 164 = (Dmax_foward - D_min_foward) * (vx+wz*L/2)/robot_max + D_min_foward
Dl = 91*vl/vmax + 164 = (Dmax_foward - D_min_foward) * (vx-wz*L/2)/robot_max + D_min_foward

note that robot_max must equal vmax+wmax*L/2 for code to work. Check Nav stack for this information!!

publish(Dr)
publish(Dl) */

#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include <std_msgs/UInt16.h>

double vx = 0.0;
double wz = 0.0;
std_msgs::UInt16 D_right; // duty cycle percentage from 0 to 255 (for arduino)
std_msgs::UInt16 D_left;
int D_min_foward = 164;
int D_max_foward = 255;
int L = 0.62; // centemeters
double vr = 0.0; // meters/second --> the max velocity as defined in the ROS Navigation Stack
double vl = 0.0; // meters/second --> the min velocity as defined in the ROS Navigation Stack
double robot_max = 1.0; // meters/second --> the maximum allowable velocity of the robot.
// Note that robot_max must equal vmax+wmax*L/2 for code to work. Check Nav stack for this information!!

int publish = 0;

using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace std;


void callback (const Twist& vel_msg)
{
        publish = 1; // only if a new message is recieved will the node publish
        // data to the topics Duty_Cycle_Left and Duty_Cycle_Right
        vx = vel_msg.linear.x;
        wz = vel_msg.angular.z;
        ROS_INFO("I Heard: [%f, %f]", vx, wz);
        cout << "Twist Recieved" << endl;

        vr = vx + wz*L/2;
        vl = vx - wz*L/2;

        D_right.data = (D_max_foward - D_min_foward) * vr / robot_max + D_min_foward;
        D_left.data = (D_max_foward - D_min_foward) * vl / robot_max + D_min_foward;

}


int main( int argc, char *argv[] )
{
	init(argc, argv, "Cmd_vel_to_Duty");

    NodeHandle n;

	Subscriber Velocity_Commands = n.subscribe("/cmd_vel", 10, callback);
		// subscribing to the Nav Stack velocity commands
	Publisher Left_Duty = n.advertise<std_msgs::UInt16>("/Duty_Cycle_Left", 10);
	Publisher Right_Duty = n.advertise<std_msgs::UInt16>("/Duty_Cycle_Right", 10);
		// publishing the Left and Right motor duty cycle for the arduino.

	while( n.ok() )
	{
		if ( publish == 1 ){
			Left_Duty.publish(D_left);
			Right_Duty.publish(D_right);
			publish = 0;
		}

		spinOnce();
	}

	return 1;
}
