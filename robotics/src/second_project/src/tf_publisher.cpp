/*
Descrizione: si iscrive a odometry e pubblica su tf
*/ 

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class tf_sub_pub {
private:
    ros::NodeHandle n;  
    tf::TransformBroadcaster br; 
    ros::Subscriber sub; 
        
public: 
    tf_sub_pub() {
        sub = n.subscribe("/ugv/odom", 1000, &tf_sub_pub::callback, this); 
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TransformStamped position;
        position.child_frame_id = "base_footprint"; 

        position.header.stamp = msg->header.stamp; 
        position.header.frame_id = "odom"; 

        position.transform.translation.x = msg->pose.pose.position.x;
        position.transform.translation.y = msg->pose.pose.position.y;
        position.transform.translation.z = msg->pose.pose.position.z;

        position.transform.rotation.w = msg->pose.pose.orientation.w;
        position.transform.rotation.x = msg->pose.pose.orientation.x;
        position.transform.rotation.y = msg->pose.pose.orientation.y;
        position.transform.rotation.z = msg->pose.pose.orientation.z;

        br.sendTransform(position);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_publisher");
    tf_sub_pub my_tf_sub_pub;

    ros::spin();
    return 0;
}