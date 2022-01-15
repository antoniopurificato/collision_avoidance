#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <math.h>
#include "geometry.h"

const int AVOIDANCE_FACTOR = 10;
ros::Publisher velocity;
bool command_received = false;
geometry_msgs::Twist velocity_received;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    
    command_received=true; 
    velocity_received = *msg;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
 if (!command_received ) return;
 command_received = false;

 tf::TransformListener listener;
 laser_geometry::LaserProjection laser_projection;
 sensor_msgs::PointCloud cloud;

 /*Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud in target frame.*/
 laser_projection.transformLaserScanToPointCloud("base_laser_link", *msg, cloud, listener);


  tf::StampedTransform tf_obstacle;
   try{
    
    /*I convert the points from the laser scan to the reference frame of the robot*/
    /*The waitForTransform() takes four arguments:

    Wait for the transform from this frame...
    ... to this frame,
    at this time, and
    timeout: don't wait for longer than this maximum duration*/
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(1.0)); 
        
    /*We want the transform from frame base_footprint to frame base_laser_link.
    The time at which we want to transform. Providing ros::Time(0) will just get us the latest available transform.
    The object in which we store the resulting transform.*/
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), tf_obstacle);
    }
    catch(tf::TransformException &e){
        ROS_ERROR("%s", e.what());
        return;
    }

    /*Coordinates in robot reference frame*/
    Eigen::Isometry2f laser_tf = convertPose2D(tf_obstacle);

    float force_x,force_y = 0;
    Eigen::Vector2f initial_position,current_position;
    
    initial_position(0) = cloud.points[540].x;
    initial_position(1) = 0;

    /*I compute the initial distance between the laser scan of my robot and the closest obstacle*/
    initial_position = laser_tf * initial_position;
    float initial_distance = sqrt(pow(initial_position(0),2)+pow(initial_position(1),2));

    /*I scan all the elements of the cloud and i compute the x and y components of the obstacle's position*/
    for(auto& point: cloud.points){         
        current_position(0) = point.x;                 
        current_position(1) = point.y;

        /*Position of the obstacle in the robot reference frame*/
        current_position = laser_tf * current_position;

        /*Current distance between the robot and the obstacle*/
        float current_distance = sqrt(pow(point.x,2) + pow(point.y,2));

        /*If i'm closer to the obstacle wrt. the previous position i update the intial position with the current position*/
        if(current_distance < initial_distance){
            initial_distance = current_distance;
            initial_position = current_position;
        }
    }

    /*If i am too much close to the obstacle, i avoid it*/
    if(initial_distance < 0.3){
        float magnitude = (1.0 / initial_distance);
        magnitude /= AVOIDANCE_FACTOR;
        force_x = -(initial_position(0) / initial_distance) * magnitude;
        force_y = -(initial_position(1) / initial_distance) * magnitude;

        geometry_msgs::Twist msg_send;

        msg_send.linear.x = velocity_received.linear.x + force_x;


        msg_send.linear.y = velocity_received.linear.y + force_y;
        msg_send.linear.z = velocity_received.linear.z;

        if(initial_position(1) > 0)
            msg_send.angular.z = - magnitude; 
        else if (initial_position(1) < 0)
            msg_send.angular.z = magnitude; 

        velocity.publish(msg_send);
    }

    else {
      velocity.publish(velocity_received);
    }
}

int main(int argc, char **argv){

    /*Starting the collision avoidance node */
    ros::init(argc, argv, "collision_avoidance"); 
    ros::NodeHandle n;

    /*Subscriber for the velocity command (topic) of the robot*/
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_call", 1, cmd_vel_callback);

    /*Subscriber for the laser scan (topic) of the robot*/
    ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1, laser_callback);

    ROS_INFO("Collision avoidance node is active!\n");
    velocity = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::spin();
    return 0;
}