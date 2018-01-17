#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>

class SubscribeAndPublish
{

	public:
	SubscribeAndPublish()

	{

		sub_joint= n.subscribe<sensor_msgs::Joy>("joy", 1, &SubscribeAndPublish::joyCallback, this);
		pub_velocity_left= n.advertise<std_msgs::Float64>("left_velocity_controller/command", 1);
	
	}

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		std_msgs::Float64 velocity_left;
	
		//do something with the code
		velocity_left.data = joy->buttons[0];
		pub_velocity_left.publish(velocity_left);

	}

	private:
	
	ros::NodeHandle n;
	ros::Publisher pub_velocity_left;
	ros::Subscriber sub_joint;

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"subscribe_and_publish");
	SubscribeAndPublish SAPObject;
	ros::spin();

	return 0;

}


