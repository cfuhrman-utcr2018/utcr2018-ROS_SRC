/* Comment here */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>


#include <array>
#include <vector>

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

		std_msgs::Float64 velocity_L; // the speed as a function of direction
			// iniialized to 0 so that the robot will be still initially
		std_msgs::Float64 velocity_R;
		std_msgs::UInt16 duty_cycle_L; // the duty cycle as a function of speed (+ or -) and velocity
			// set to 158 so that the motor controllers will keep the wheels stationary
		std_msgs::UInt16 duty_cycle_R;
		std_msgs::Float64 speed;
		std_msgs::Float64 direction;
		std_msgs::Float64 A;
		std_msgs::Float64 B;
		
		// initializing the maximum and minimum multipliers for the velocity commands acting on the speed command
		float Turn_Max = 0.75;
		float Turn_Min = 1.0-Turn_Max;
	
		// initialize the speed and direction from the joystick commands
		speed.data = joy->axes[1];
		direction.data = joy->axes[3];
		// initialize the start and stop buttons A and B
		A.data = joy->buttons[0];
		B.data = joy->buttons[1];
/*
		// wait for the A button to be pressed
		go_button:
		while ( A < 0.9 ){
			// do nothing
		}
		
		// B button stops the robot
		if ( B > 0.8 ){
			duty_cycle = 158;
			goto go_button;
		}
*/	
		
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
		
		pub_DutyCycle_L.publish(duty_cycle_L);
		pub_DutyCycle_R.publish(duty_cycle_R);
		
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


