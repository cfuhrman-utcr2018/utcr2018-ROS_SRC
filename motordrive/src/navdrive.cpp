#include <geometry_msgs.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

double vx = 0.0;
double wz = 0.0;

int D = 0; // duty cycle percentage from 0 to 255 (for arduino)
int D_min_foward = 164;
int D_max_foward = 255;
const int L = 0.62; // centemeters
double vr = 0.0; // meters/second --> the max velocity as defined in the ROS Navigation Stack
double vl = 0.0; // meters/second --> the min velocity as defined in the ROS Navigation Stack
double robot_max = 1.0; // meters/second --> the maximum velocity physically realizable due to motors

/* Pseduocode:

subscribe to cmd_vel
vx = linear.x
wz = angular.z

vr = vx + wz*L/2
vl = vx - wz*L/2

Dr = 91*vr/vmax + 164 = (Dmax-foward - D_min_foward) * (vx+wz*L/2)/robot_max + D_min_foward
Dl = 91*vl/vmax + 164 = (Dmax-foward - D_min_foward) * (vx-wz*L/2)/robot_max + D_min_foward

note that robot_max must equal vmax+wmax*L/2 for code to work. Check Nav stack for this information!!

publish(Dr)
publish(Dl) */


