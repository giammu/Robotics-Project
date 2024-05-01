/*
 Secondo nodo: odom_to_tf: si iscrive a odometry e pubblica su tf (è estremamente simile a quello visto durante il lab 5: cambio il nome/tipo dei topic)

Si iscrive all'odometry
    type: nav_msgs/Odometry
    topic name: input_odom (questo è un nome generico, bisogna usare il remapping per iscriversi al vero topic)

Prende come input input_odom e ha come parameters i due reference frame del tf (root_frame and child_frame)

Il nodo parte dal launch file, con topic remapping per l'input e pubblica parametri con il tf_broadcaster
Il launch file deve creare 2 istanze del nodo [N.B. devo scrivere un nodo generico]: 1) pubblica l'odometry dell'encoder, 2) pubblica l'odometry dal gps
Entrambi hanno lo stesso root = world, ma i child frame sono rispettivamente 1) wheel_odom 2)gps_odom (partono dalla stessa posizione ma probabilmente hanno posizioni diverse al passare del tempo)

Quello che devo fare: scrivere il nodo, metterlo nel launch file, configurarlo per partire 2 volte, iscriversi al correct odometry, publish the correct tf using the parameters to set the root and child frame
*/ 


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class tf_sub_pub{

    std::string root_frame;
    std::string child_frame;

    private:
        ros::NodeHandle n;  
        tf::TransformBroadcaster br; 
        ros::Subscriber sub; 
            
    public: tf_sub_pub(){

        n.getParam("root_frame",root_frame);
        n.getParam("child_frame",child_frame);

        ROS_INFO("\n    Root frame: %s, Child Frame %s", root_frame.c_str(), child_frame.c_str() );

        sub = n.subscribe("input_odom", 1, &tf_sub_pub::callback, this); 
        }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        
        tf::Transform transform; //creo la transform
        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) ); //Setto il primo elemento della transform: la position
        
        tf::Quaternion q; //Creo il quaternion
        
        //Metodo 1
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

        //Metodo 2 
        //double qx = msg->pose.pose.orientation.x;
        //double qy = msg->pose.pose.orientation.y;
        //double qz = msg->pose.pose.orientation.z;
        //double qw = msg->pose.pose.orientation.w;

        //double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz));
        //q.setRPY(0,0,yaw); 

        //Metodo 3 (penso sbagliato)
        //q.setRPY(0, 0, msg->pose.pose.orientation.w); //Setto il RPY:roll, pitch, yaw   
        

        transform.setRotation(q); //Setto il secondo elemento della transform: l'orientamento
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame)); //Pubblico la transformation
    }

};


int main(int argc, char **argv){
 ros::init(argc, argv, "odom_to_tf");
 tf_sub_pub my_tf_sub_pub;
 ros::spin();
 return 0;
}