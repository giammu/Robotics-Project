/*
Terzo nodo: lidar data visualization: abbiamo 2 odometry e dobbiamo capire quale dei due è meglio

Il nodo si iscrive a /os_cloud_node/points e cambia il reference frame, che è definito nell'header
Il valore è regolato da un dynamic reconfigure callback che permette di cambiarlo dinamicamente tra wheel_odom o gps_odom
Alla fine pubblica sul topic /pointcloud_remapped 

Il nodo deve permettere all'utente di selezionare da rqt_reconfigure a quale tf si dovrebbe connettere il lidar

(Così facendo da rviz dovrei vedere la laserscan centrata sul tf dell'encoder o sul tf del gps odometry)

*/

#include "ros/ros.h" 
#include "sensor_msgs/PointCloud2.h"



void callback(const sensor_msgs::PointCloud2& msg){ //funzione chiamata automaticamente ogni volta che arriva un nuovo messaggio


    ROS_INFO("header %d", msg.header);  //print header, not sure if it is the correct header, need to find the correct type
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "lidar_remap");
    ros::NodeHandle n; //forse devo usare più di 1 handler per fare pub e sub in contemporanea


    //Subscriber
    ros::Subscriber sub = n.subscribe("os_cloud_node/points", 1, callback); //topic: fix, buffer: dimensione 1, il subscriber chiama la funzione
    ros::spin(); //cicla aspettando i messaggi

    /*
    //Publisher
   	ros::Publisher pub = n.advertise<"typeMSG">("lidar_remap", 1); // type: nav_msgs/Odometry, topic: gps_odom, buffer: dimensione 1 (good practice)
    "typeMSG" msg; //creo il messaggio
    //msg = something  <--- assegno il valore (probabilmente sbagliato)
    pub.publish(msg);

    //Loop
    ros::Rate loop_rate(10); //loop MANUALE a 10hz 
  	while (ros::ok()){ //loop
        //qua eseguo qualcosa
    	ros::spinOnce();
    	loop_rate.sleep(); 
  	}
    //oppure loop AUTOMATICO:  ros::spin();
    */

}
