#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
/* this provides an implementation of TransformBroadcaster to help make the task
of publishing transforms easier */
#include <turtlesim/Pose.h>

std::string turtle_name;



void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br; // creating a TransformBroadcaster object
// that is used to send the tfs over the wire

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
// creating a Transform object and copying the 2D turtle pose into a 3D transform
  
  transform.setRotation(q);
// setting the rotation

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
/* sending a transform with a TransformBroadcaster requires four arguments
1) passing the transform itself
2) need to give the transform being published a timestamp. Stamping with the current time ros::Time::now()
3) need to pass the name of the parent frame of the link that we are creating. We are calling the parent frame "world"
4) need to pass the name of the child frame of the link that we are creating. This is the name of the turtle iself */
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
