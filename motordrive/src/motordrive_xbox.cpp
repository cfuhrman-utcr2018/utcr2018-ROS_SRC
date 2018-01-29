/* Comment here */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>

class SubscribeAndPublish
{

	public:
	SubscribeAndPublish()

	{

		sub_joint= n.subscribe<sensor_msgs::Joy>("joy", 1, &SubscribeAndPublish::joyCallback, this);
		pub_speed= n.advertise<std_msgs::Float64>("speed/command", 1);
		pub_direction = n.advertise<std_msgs::Float64>("direction/command", 1);
		
	
	}

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		std_msgs::Float64 speed; // the right joystick up and down for the maximum motor speed
		std_msgs::Float64 direction; // the direction in x values for the motors
	
		//do something with the code
		speed.data = joy->axes[1];
		direction.data = joy->axes[3];
		
		pub_speed.publish(speed);
		pub_direction.publish(direction);

	}

	private:
	
	ros::NodeHandle n;
	ros::Publisher pub_speed;
	ros::Publisher pub_direction;
	ros::Subscriber sub_joint;

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"subscribe_and_publish");
	SubscribeAndPublish SAPObject;
	ros::spin();

	return 0;

}


