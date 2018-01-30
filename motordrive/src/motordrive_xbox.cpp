/* Comment here */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/MultiArrayLayout.h"
//#include "std_msgs/MultiArrayDimension.h"
//#include "std_msgs/Float64MultiArray.h"
//#include "std_msgs/Int32MultiArray.h"


#include <array>
#include <vector>

class SubscribeAndPublish
{

	public:
	SubscribeAndPublish()

	{	sub_joy= n.subscribe<sensor_msgs::Joy>("joy", 1, &SubscribeAndPublish::joyCallback, this); // subscribe to the /joy topic
		pub_DutyCycle= n.advertise<std_msgs::Float64MultiArray>("Duty_Cycle/command", 1); // publish the duty cycle to the arduino
	}
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{

		std_msgs::Float64MultiArray velocity; // the speed as a function of direction
			// iniialized to 0 so that the robot will be still initially
			// structured as (left motor, right motor)
		std_msgs::Float64MultiArray duty_cycle; // the duty cycle as a function of speed (+ or -) and velocity
			// set to 158 so that the motor controllers will keep the wheels stationary
			// structured as (left motor, right motor)
		std_msgs::Float64 speed;
		std_msgs::Float64 direction;
		std_msgs::Float64 A;
		std_msgs::Float64 B;
		
		// initializing the maximum and minimum multipliers for the velocity commands acting on the speed command
		float Turn_Max = 0.75;
		float Turn_Min = 1-Turn_Max;
	
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
		if ( direction.data > 0.1 ){
			velocity.data[0] = speed.data * (1.0+Turn_Min*direction.data);
			velocity.data[1] = speed.data * (1.0-Turn_Max*direction.data); 
		} else if ( direction.data < -0.1 ) {
			velocity.data[0] = speed.data * (1.0+Turn_Min*direction.data);
			velocity.data[1] = speed.data * (1.0+Turn_Max*direction.data);
		} else {
			velocity.data = {0.0, 0.0};
		}
		
		if ( speed.data > 0 ){
			duty_cycle.data[0] = 91.8*velocity.data[0]+163.2;
			duty_cycle.data[1] = 91.8*velocity.data[1]+163.2;
		} else {
			duty_cycle.data[0] = 153.0*velocity.data[0]+153.0;
			duty_cycle.data[1] = 153.0*velocity.data[1]+153.0;
		}
		
		pub_DutyCycle.publish(duty_cycle);
	}

	private:
	
	ros::NodeHandle n;
	ros::Publisher pub_DutyCycle;
	ros::Subscriber sub_joy;

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"subscribe_and_publish");
	SubscribeAndPublish SAPObject;
	ros::spin();

	return 0;

}


