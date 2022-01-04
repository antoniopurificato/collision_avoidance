#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "geometry.h"

ros::Publisher velocity;
bool command_received = false;
float vel_x, vel_y, vel_angular = 0;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    
    command_received=true; 
    vel_x=msg->linear.x;
    vel_y=msg->linear.y; 
    vel_angular=msg->angular.z;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
 if (!command_received ) return;
 command_received = false;

 tf::TransformListener listener;
 laser_geometry::LaserProjection laser_projection;
 sensor_msgs::PointCloud cloud;
 laser_projection.transformLaserScanToPointCloud("base_laser_link", *msg, cloud, listener);


  tf::StampedTransform tf_obstacle;
   try{
        /*I convert the points from the laser scan to the reference frame of the robot*/
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10.0)); 
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), tf_obstacle); 
        ROS_INFO("%d", tf_obstacle.getOrigin().getY());
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", "Transformation error");
        return;
    }

    /*Coordinates in robot reference frame*/
    Eigen::Isometry2f laser_tf = convertPose2D(tf_obstacle);
    float force_x,force_y = 0;
    Eigen::Vector2f position;

    /*I scan all the elements of the cloud and i compute the x and y components of the obstacle's position*/
    for(auto& point: cloud.points){         
        position(0) = point.x;                 
        position(1) = point.y;

        /*Position of the obstacle in the robot reference frame*/
        position = laser_tf * position;

        /*Distance between the robot and the obstacle*/
        float distance = sqrt(pow(point.x,2) + pow(point.y,2));

        /*Modulus of the force*/
        float mod = 1/pow(distance,2);

        force_x += position(0) * mod;
        force_y += position(1) * mod;
    }

    /*I have to consider the opposite sign of the force*/
    force_x = -force_x;
    force_y  -force_y;
}

int main(int argc, char **argv){

    /*Starting the collision avoidance node */
    ros::init(argc, argv, "collision_avoidance"); 
    ros::NodeHandle n;

    /*Subscriber for the velocity command (topic) of the robot*/
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);

    /*Subscriber for the laser scan (topic) of the robot*/
    ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1, laser_callback);

    ROS_INFO("Collision avoidance node is active!\n");
    velocity = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::spin();
    return 0;
}
