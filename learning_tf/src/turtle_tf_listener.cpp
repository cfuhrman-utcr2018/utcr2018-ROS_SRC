#include <ros/ros.h>
#include <tf/transform_listener.h>
// provides an implimentation of the TransformListener to make recieving transforms easier
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;
// creating a TransformListener object. Once it is created, it begins to recieve tf tranfromations 
// over the wire and buffers them for up to 10 seconds. 

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/turtle2", "/carrot1",
                               ros::Time(0), transform);
/* there are four arguments for the main portion of the code
1) the transform that we wish to give to
2) the transform that we wish to take from
3) the time. This ros::Time(0) 
*/
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);
// this calculates the velocity messages for turtle2/cmd_vel based on what is happening in 
// the /turtle1

    rate.sleep();
  }
  return 0;
};
