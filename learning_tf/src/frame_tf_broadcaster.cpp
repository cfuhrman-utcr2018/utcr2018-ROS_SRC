// code from tutorial on Adding a Frame for the tf. Found here: http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
// a fixed frame that does not change over time
//    transform.setOrigin( tf::Vector3(-1.0, 0.0, 0.0) );
//    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    // the carrot will be 2 meters offset and offset to the left of turtle1
    
// a frame that is dynamic and changes over time
	transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
    // parent turtle1 will have a child carrot1
    rate.sleep();
  }
  return 0;
};

