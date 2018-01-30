/* Comment here */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

class SubscribeAndPublish
{

	public:
	SubscribeAndPublish()

	{	sub_joy= n.subscribe<sensor_msgs::Joy>("joy", 1, &SubscribeAndPublish::joyCallback, this); // subscribe to the /joy topic
		pub_DutyCycle= n.advertise<std_msgs::Float64>("Duty_Cycle/command", 1); // publish the duty cycle to the arduino
	}
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		std_msgs::Float64 speed = 0.0; // the right joystick up and down for the maximum motor speed
		std_msgs::Float64 direction = 0.0; // the direction in x values for the motors
		
		std_msgs::array<float, 2> velocity{(0.0, 0.0)}; // the speed as a function of direction
			// iniialized to 0 so that the robot will be still initially
			// structured as (left motor, right motor)
		std_msgs::vector<int, 2> duty_cycle = {(158,158)}; // the duty cycle as a function of speed (+ or -) and velocity
			// set to 158 so that the motor controllers will keep the wheels stationary
			// structured as (left motor, right motor)
		
		// initializing the maximum and minimum multipliers for the velocity commands acting on the speed command
		std_msgs::Float64 Turn_Max = 0.75;
		std_msgs::Float64 Turn_Min = 1-Turn_Max;
	
		// initialize the speed and direction from the joystick commands
		speed = joy->axes[1];
		direction = joy->axes[3];
		// initialize the start and stop buttons A and B
		A = joy->buttons[0];
		B = joy->buttons[1];
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
		if ( direction > 0.1 ){
			velocity(0) = speed * (1.0+Turn_Min*direction);
			velocity(1) = speed * (1.0-Turn_Max*direction); 
		} else if ( direction < -0.1 ) {
			velocity(0) = speed * (1.0+Turn_Min*direction);
			velocity(1) = speed * (1.0+Turn_Max*direction);
		} else {
			velocity = {(0.0, 0.0)};
		}
		
		if ( speed > 0 ){
			duty_cycle = 91.8*velocity+163.2;
		} else {
			duty_cycle = 153*velocity+153;
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


