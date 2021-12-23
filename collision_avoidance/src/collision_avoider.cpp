#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

ros::Publisher velocity;
bool command_received = false;
float vel_x, vel_y, vel_angular = 0;

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg){
    command_received=true; 
    vel_x=msg->linear.x;
    vel_y=msg->linear.y; 
    vel_angular=msg->vel_angular.z;

}

int main(int argc, char **argv)
{}
