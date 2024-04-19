/*
Primo Nodo: deve computare la odometry dai gps data

The node subscribe to fix and publish the odometry message:
    type: nav_msgs/Odometry
    topic name: gps_odom

Abbiamo latitude, longitude, altitude e li convertiamo in ECEF Cartesiano
Convertiamo ECEF a ENU
ENU è relativo: fornisco la starting position: di solito si mette la zero position del robot come la relative position

3 parametri del nodo: lat_r, lon_r, alt_r -> devo settarli manualmente nel launch file con il primo valore del gps

Il gps ti dà una posizione ma non l'orientation, devo computarla: ho multiple points (è la trajectory e il gps funziona a 10hz)
-> computo l'orientamento usando i punti consecutivi (la linea che passa tra i due punti) che mi sono trovato prima con ENU
*/

#include "ros/ros.h" 
#include "nav_msgs/Odometry.h" 
#include "sensor_msgs/NavSatFix.h"
#include "math.h"
#include "typeinfo"
#include <sstream>

std::float_t lat_r;
std::float_t lon_r;
std::float_t alt_r; 

float* castToECEF(int lat, int lon, int alt) {
    float a = 6378137.0;
    float b = 6356752.0;
    float e2 = 1 - (b*b)/(a*a);
    float n = a/sqrt(1.0-e2*sin(lat)*sin(lat));
    float coord[3];


    coord[1] = (n + alt)*cos(lat)*cos(lon);
    coord[2] = (n + alt)*cos(lat)*sin(lon);
    coord[3] = (n*(1.0-e2) + alt)*sin(lat);
    return coord;
}



void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){ //funzione chiamata automaticamente ogni volta che arriva un nuovo messaggio
    
    //ROS_INFO("\n header %d", msg->header.seq);
    //ROS_INFO("\n lat %f, lon %f, alt %f", msg->latitude, msg->longitude, msg->altitude);

    ROS_INFO("\n lat %f, lon %f, alt %f", lat_r, lon_r, alt_r);


    

/*
    
    //problema: entra nel ciclo solo se l'header è diverso da 1
    //test fatto eseguendo prima gps_to_odom e poi facendo partire robotics.bag

    if(msg->header.seq == 1) {
            float lat_r = msg->latitude;
            float lon_r = msg->longitude;
            float alt_r = msg->altitude;
            ROS_INFO("pippo");
    }


    
    float lat = msg->latitude;
    float lon = msg->longitude;
    float alt = msg->altitude;



    //need to cast lat long to radiant for find sin 
    const double PI = 3.14159265358979323846;
    float latRad = (lat*PI)/180.0;
    float lonRad = (lon*PI)/180.0;

    //cast to ECEF


    float* coord = castToECEF(latRad, lonRad, alt);


    //cast to ENU

    */
}



int main(int argc, char **argv){

    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n; //forse devo usare più di 1 handler per fare pub e sub in contemporanea

              //PRENDO I PARAMETRI HARDCODATI NEL LAUNCHFILE
	n.getParam("lat_r", lat_r);
	n.getParam("lon_r", lon_r);
	n.getParam("alt_r", alt_r);

    

    //Subscriber
    ros::Subscriber sub = n.subscribe("fix", 1, callback); //topic: fix, buffer: dimensione 1, il subscriber chiama la funzione
    ros::spin(); //cicla aspettando i messaggi
    

    /*
    //Publisher
   	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1); // type: nav_msgs/Odometry, topic: gps_odom, buffer: dimensione 1 (good practice)
    nav_msgs::Odometry odom; //creo il messaggio
    //odom.pose.x = 1; //assegnao il valore (probabilmente sbagliato)
    pub.publish(odom);

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
