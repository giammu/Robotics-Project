/*
Terzo nodo: lidar data visualization: abbiamo 2 odometry e dobbiamo capire quale dei due è meglio

Il nodo si iscrive a /os_cloud_node/points e cambia il reference frame, che è definito nell'header
Il valore è regolato da un dynamic reconfigure callback che permette di cambiarlo dinamicamente tra wheel_odom o gps_odom
Alla fine pubblica sul topic /pointcloud_remapped 

Il nodo deve permettere all'utente di selezionare da rqt_reconfigure a quale tf si dovrebbe connettere il lidar

(Così facendo da rviz dovrei vedere la laserscan centrata sul tf dell'encoder o sul tf del gps odometry)

*/


#include "sensor_msgs/PointCloud2.h"
#include "string.h"
#include <sstream>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h> //includo la dynamic reconfigure perchè è quello che stiamo facendo 
#include <first_project/parametersConfig.h> // nome_package/"stringWroteInsideTheConfigurationFile" combined with "Config.h"


void callback2(const sensor_msgs::PointCloud2& msg){ 

    msg.header.frame_id = config.header;

    ROS_INFO("header %s", msg.header.frame_id.c_str());  //print if the reference is gps or wheel odom
    
}


void callback(first_project::parametersConfig &config, uint32_t level) { //la callback function ha 2 parametri: config=la lista di tutti i values, level= cioè quale parametro cambio (quello che prima abbiamo definito con 0,1,2,3,4...). uso uint32_t perchè solo 1 parmetro può cambiare alla volta, e quindi è sufficiente
    ros::NodeHandle n; 
    
    ROS_INFO("Reconfigure Request: %s ", config.header.c_str());  
    ROS_INFO ("%d",level); //se voglio vedere quale sta cambiando mi basta vedere il level

    ros::Subscriber sub = n.subscribe("os_cloud_node/points", 1, callback2); 
    
    ros:: Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1); // type: nav_msgs/Odometry, topic: gps_odom, buffer: dimensione 1 (good practice)
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap"); //inizializzo il ros node

    dynamic_reconfigure::Server<first_project::parametersConfig> server; //prima creo il sever: il tipo del server deve essere parameter_test::parametersConfig, cioè nome del package::nome del config file
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f; //qua creo la callback

    //Subscriber
    //ros::Subscriber sub = n.subscribe("os_cloud_node/points", 1); 

    f = boost::bind(&callback, _1, _2); //then i use boost to bind the callback che ho appena definito (riga 20) con la funzione che si occupa di fare la callback (riga 6)
    server.setCallback(f); //gli passo la callback che ho appena creato

    ROS_INFO("Spinning node");
    ros::spin(); //il nodo continua ad andare in attesa che succeda qualcosa sulla dynamic reconfigure, triggerando la callback
    return 0;
}
