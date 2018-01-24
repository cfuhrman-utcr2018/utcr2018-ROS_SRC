#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>



ros::NodeHandle n;
ros::Subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &AutoExp::laserCallback, this);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
}
	
	
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  
  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
  
  ros::spin();
}
