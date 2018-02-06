#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// this package provides the tf::TransformBroadcaster.

double x = 0.1; // offset in the x direction
double y = 0.0; // offset in the y direction
double z = 0.2; // offset in the z direction

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  // this is creating the transform object that sends the transform from
  // base_link --> base_laser

  while (n.ok()) {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, z)),
        ros::Time::now(), "base_link", "base_laser"
      )
    );
    r.sleep();
  }

}
