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

void castToECEF(float* lat, float* lon, float* alt){

    // Conversione da gradi a radianti
    float lat_rad = (*lat * M_PI) / 180.0;
    float lon_rad = (*lon * M_PI) / 180.0;
    
    float a = 6378137.0; 
    float b = 6356752.0; 
    float e2 = 1 - (b*b)/(a*a); 
    float n = a / sqrt(1.0 - e2 * sin(lat_rad) * sin(lat_rad)); 

    // Calcolo delle coordinate ECEF
    float x = (n + *alt) * cos(lat_rad) * cos(lon_rad);
    float y = (n + *alt) * cos(lat_rad) * sin(lon_rad);
    float z = (n * (1.0 - e2) + *alt) * sin(lat_rad);

    // Assegnazione delle coordinate ECEF alle variabili di output
    *lat = x;
    *lon = y;
    *alt = z;
}

void castToENU(float* lat, float* lon, float* alt, float lat_original, float lon_original){

    float slat = sin(lat_original);
    float clat = cos(lat_original);
    float slon = sin(lon_original);
    float clon = cos(lon_original);

    float xp = *lat - lat_r;
    float yp = *lon - lon_r;
    float zp = *alt - alt_r;

    float x = -slon*xp + clon*yp;
    float y = -slat*clon*xp - slat*slon*yp + clat*zp;
    float z = clat*clon*xp + clat*slon*yp + slat*zp;

    *lat=x;
    *lon=y;
    *alt=z;
}


void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){ //funzione chiamata automaticamente ogni volta che arriva un nuovo messaggio
    
    //ROS_INFO("\n header %d", msg->header.seq);
    //ROS_INFO("\n lat %f, lon %f, alt %f", msg->latitude, msg->longitude, msg->altitude);

    
    
    float lat = msg->latitude;
    float lon = msg->longitude;
    float alt = msg->altitude;

    float lat_original = msg->latitude;
    float lon_original = msg->longitude;

    //cast to ECEF
    castToECEF(&lat, &lon, &alt);

    //cast to ENU
    castToENU(&lat, &lon, &alt, lat_original, lon_original);

    ROS_INFO("\n x %f, y %f, z %f", lat, lon, alt);
    
}



int main(int argc, char **argv){

    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n; //forse devo usare più di 1 handler per fare pub e sub in contemporanea

	n.getParam("lat_r", lat_r); //prendo i parametri dal launchfile
	n.getParam("lon_r", lon_r);
	n.getParam("alt_r", alt_r);
    castToECEF(&lat_r, &lon_r, &alt_r);



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
