/* Code to control the IGVC 2018 entry from The Citadel with a wireless Xbox 360 Controller
Requirements:
joy_node
rosserial_arduino node
Xbox 360 controller
Arduino
Motor Controllers Victor 884
================================================================================================================
Author: Connor Fuhrman
Contact: 512-897-0709
Contact: cfuhrman@citadel.edu
Contact: connorfuhrman@outlook.com
================================================================================================================
List of Variables:
- Global
	- Start - This variable is the wireless emergency stop for the robot
					it is initialized to zero. This means that the robot PWM will be at
					about 58% duty cycle corresponding to a 154. When A is pushed on the
					controller the Start will be changed to 1. The robot will then move
					corresponding to the movement of the right and left stick. Pressing B
					on the xbox controller will cause the robot to stop by sending a
					value of 154 to the arduino for PWM.
- Local
	- speed - this is the variable connected to the left stick. The left stick
					moves from -1 to 1 for up and down. A -1 means that the stick is
					pulled fully down. A +1 means that the stick is pushed fully foward.
	- direction - this is the variable connected to the right stick. The left
							stick moves from -1 to 1 for left and right. A -1 means that
							the stick is pulled fully right. A +1 means that the stick is
							pushed fully left.
	- velocity_L - this command calculates the speed of the left wheel based on
							 the inputs of the left and the right stick. this command is based
							 on a line that assumes that turning fully one way makes the
							 opposite wheel turn faster than the inside wheel. This value is
							 based on Turn_Max and Turn_Min.
	- velocity_R- this command calculates the speed of the right wheel based on
							 the inputs of the left and the right stick. this command is based
							 on a line that assumes that turning fully one way makes the
							 opposite wheel turn faster than the inside wheel. This value is
							 based on Turn_Max and Turn_Min.
	- duty_cycle_L - takes the velocity_L command calculated based on stick
								inputs and turns it into duty cycle commands based on the
								motor controller input. The Arduino accepts an 8 bit unsigned
								integer meaning the value can range from 0 to 255.
	- duty_cycle_R - takes the velocity_R command calculated based on stick
								inputs and turns it into duty cycle commands based on the
								motor controller input. The Arduino accepts an 8 bit unsigned
								integer meaning the value can range from 0 to 255.
	- A - the A button on the joystick. Initially needs to be depressed to run the
			robot. After pressing B to stop the robot the A button will resume
			normal operation based on the stick inputs.
	- B - the B button is the emergency stop for the robot. At any time pressing
			B will cancel out any duty cycle commands and replace them with 154. This
			is because the middle duty cycle corresponds to a stop for the motor
			controllers.
	- Turn_Max - when the robot is turning 100% left or right this is what the
						outside wheel percentage of speed is.
	- Turn_Min - when the robot is turning 100% left or right this is what the
						inside wheel percentage of speed is. Currently the Turn_Max+Turn_Min
						= 1 (or 100%).

ROS Subscribers:
- This node subscribes to the /joy command from the joy_node. This gives the
information about the xbox controller movement.

ROS Publishers:
- This node publishes two duty cycle commands, Duty_Cycle_Left and
Duty_Cycle_Right to the Arduino thorugh the rosserial_arduino node.
================================================================================================================
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>


std_msgs::UInt16 Start; // the emergency stop button

class SubscribeAndPublish
{

	public:
	SubscribeAndPublish()

	{	sub_joy= n.subscribe<sensor_msgs::Joy>("joy", 10, &SubscribeAndPublish::joyCallback, this);
	// subscribe to the /joy topic
		pub_DutyCycle_L= n.advertise<std_msgs::UInt16>("Duty_Cycle_Left", 10);
		// publish the left motor duty cycle to the arduino
		pub_DutyCycle_R= n.advertise<std_msgs::UInt16>("Duty_Cycle_Right", 10);
		// publish the right motor duty cycle to the arduino
        light = n.advertise<std_msgs::UInt16>("Light",10);
        // publish the information as to what light should be on
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
		speed.data = joy->axes[1]; // the left stick corrsponding to the speed
		direction.data = joy->axes[3]; // the dright stick corresponding to the
		// direction of the robot

		// initialize the start and stop buttons A and B
		A.data = joy->buttons[0]; // Start button
		B.data = joy->buttons[1]; // Stop button


		if ( A.data > 0.9 ){
            Start.data = 1;
		}

        if ( Start.data == 1){
		// invariant is the Start variable. Exit the while loop by
		// pressing B

			if ( B.data > 0.9){
                Start.data = 0;
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
            light.publish(Start);

		} else {
		// if we are not moving, the duty should be 154
		duty_cycle_L.data = 154;
		duty_cycle_R.data = duty_cycle_L.data;

		// Publish the left and right duty cycle
		pub_DutyCycle_L.publish(duty_cycle_L);
		pub_DutyCycle_R.publish(duty_cycle_R);
        light.publish(Start);

		}

	}

	private:

	ros::NodeHandle n;
	ros::Publisher pub_DutyCycle_L;
	ros::Publisher pub_DutyCycle_R;
    ros::Publisher light;
	ros::Subscriber sub_joy;

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"subscribe_and_publish");
	SubscribeAndPublish SAPObject;
	ros::spin();

	return 0;

}
