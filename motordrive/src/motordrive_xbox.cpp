/* Code to control the IGVC 2018 entry from The Citadel with a wireless Xbox 360 Controller
Requirements: joy_node
rosserial_arduino node
Xbox 360 controller
Arduino
Motor Controllers (NAME???)
================================================================================================================
Author: Connor Fuhrman
Contact: 512-897-0709
Contact: cfuhrman@citadel.edu
Contact: connorfuhrman@outlook.com
================================================================================================================
List of Variables:
- Global
	- Start
- Local
	- 

ROS Subscribers:

ROS Publishers:
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <array>
#include <vector>

int Start = 0; // the emergency stop button

class SubscribeAndPublish
{

	public:
	SubscribeAndPublish()

	{	sub_joy= n.subscribe<sensor_msgs::Joy>("joy", 10, &SubscribeAndPublish::joyCallback, this); // subscribe to the /joy topic
		pub_DutyCycle_L= n.advertise<std_msgs::UInt16>("Duty_Cycle_Left/command", 10); // publish the left motor duty cycle to the arduino
		pub_DutyCycle_R= n.advertise<std_msgs::UInt16>("Duty_Cycle_Right/command", 10); // publish the right motor duty cycle to the arduino
	}
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{

		typedef std_msgs::Float64 ROSfloat;
		typedef std_msgs::UInt16 ROSint;

		ROSfloat velocity_L; // the speed as a function of direction
			// iniialized to 0 so that the robot will be still initially
		ROSfloat velocity_R;
		ROSint duty_cycle_L; // the duty cycle as a function of speed (+ or -) and velocity
			// set to 158 so that the motor controllers will keep the wheels stationary
		ROSint duty_cycle_R;
		ROSfloat speed;
		ROSfloat direction;
		ROSfloat A;
		ROSfloat B;
		
		// initializing the maximum and minimum multipliers for the velocity commands acting on the speed command
		double Turn_Max = 0.75;
		double Turn_Min = 1.0-Turn_Max;
		
	
		// initialize the speed and direction from the joystick commands
		speed.data = joy->axes[1];
		direction.data = joy->axes[3];
		// initialize the start and stop buttons A and B
		A.data = joy->buttons[0];
		B.data = joy->buttons[1];

		
		if ( A.data > 0.9 ){
			Start = 1;
		}
		
		if ( Start == 1){
		// invariant is the Start variable. Exit the while loop by
		// pressing B

			if ( B.data > 0.9){
			 	Start = 0;
			 }

			// calculate the velocity based on the direction input (right stick)
			if ( direction.data > 0.0 ){
				velocity_L.data = speed.data * (1.0-Turn_Max*direction.data);
				velocity_R.data = speed.data * (1.0-Turn_Min*direction.data); 
			} else if ( direction.data < 0.0 ) {
				velocity_L.data = speed.data * (1.0+Turn_Min*direction.data);
				velocity_R.data = speed.data * (1.0+Turn_Max*direction.data);
			} else {
				velocity_L.data = speed.data;
				velocity_R.data = speed.data;
			} 
			
			if ( speed.data > 0.0 ){
				duty_cycle_L.data = 91.8*velocity_L.data+163.2;
				duty_cycle_R.data = 91.8*velocity_R.data+163.2;
			} else {
				duty_cycle_L.data = 153.0*velocity_L.data+153.0;
				duty_cycle_R.data = 153.0*velocity_R.data+153.0;
			}
			
			// Publish the left and right duty cycle
			pub_DutyCycle_L.publish(duty_cycle_L);
			pub_DutyCycle_R.publish(duty_cycle_R);

		} else {
		// if we are not moving, the duty should be 154
		duty_cycle_L.data = 154;
		duty_cycle_R.data = duty_cycle_L.data;
		
		// Publish the left and right duty cycle
		pub_DutyCycle_L.publish(duty_cycle_L);
		pub_DutyCycle_R.publish(duty_cycle_R);
		
		}

	}

	private:
	
	ros::NodeHandle n;
	ros::Publisher pub_DutyCycle_L;
	ros::Publisher pub_DutyCycle_R;
	ros::Subscriber sub_joy;

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"subscribe_and_publish");
	SubscribeAndPublish SAPObject;
	ros::spin();

	return 0;

}
