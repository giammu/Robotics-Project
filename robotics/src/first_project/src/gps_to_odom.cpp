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


float ToRad(float grad) {
    return (grad*M_PI)/180.0;
}

float castLatToECEF(float n, float alt, float lat_rad, float lon_rad) {
    float x = (n + alt) * cos(lat_rad) * cos(lon_rad);
    return x;
}

float castLonToECEF(float n, float alt, float lat_rad, float lon_rad) {
    float y = (n + alt) * cos(lat_rad) * sin(lon_rad);
    return y;
}

float castAltToECEF(float n, float alt, float lat_rad, float e2) {
    float z = (n * (1.0 - e2) + alt) * sin(lat_rad);
    return z;
}

/*
float* castToENU(float lat, float lon, float xp, float yp, float zp){

    float slat = sin(ToRad(lat));
    float clat = cos(ToRad(lat));
    float slon = sin(ToRad(lon));
    float clon = cos(ToRad(lon));
    float coords[3];

    coords[0] = -slon*xp + clon*yp;
    coords[1] = -slat*clon*xp - slat*slon*yp + clat*zp;
    coords[2] = clat*clon*xp + clat*slon*yp + slat*zp;

    return coords;

}
*/



void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){ //funzione chiamata automaticamente ogni volta che arriva un nuovo messaggio
    
    ROS_INFO("header %d\n", msg->header.seq);
    ROS_INFO("lat %f, lon %f, alt %f\n", msg->latitude, msg->longitude, msg->altitude);

    
    float lat = msg->latitude;
    float lon = msg->longitude;
    float alt = msg->altitude;

    float a = 6378137.0; 
    float b = 6356752.0; 
    float e2 = 1 - (b*b)/(a*a);
    float n = a / sqrt(1.0 - e2 * sin(ToRad(lat_r)) * sin(ToRad(lat_r)));
 
    float x_ecef_rif = castLatToECEF(n, alt,ToRad(lat_r), ToRad(lon_r));
    float y_ecef_rif = castLonToECEF(n, alt,ToRad(lat_r), ToRad(lon_r));
    float z_ecef_rif = castAltToECEF(n, alt,ToRad(lat_r), e2);

    ROS_INFO("ECEF_RIF: x %f, y %f, z %f\n", x_ecef_rif, y_ecef_rif, z_ecef_rif);

    //cast to ECEF

    float x_ecef = castLatToECEF(n, alt, ToRad(lat), ToRad(lon));
    float y_ecef = castLonToECEF(n, alt, ToRad(lat), ToRad(lon));
    float z_ecef = castAltToECEF(n, alt, ToRad(lat), e2);



    //castToECEF(&lat, &lon, &alt);

    ROS_INFO("ECEF: x %f, y %f, z %f\n", x_ecef, y_ecef, z_ecef);

    //cast to ENU

    float xp = x_ecef - x_ecef_rif;
    float yp = y_ecef - y_ecef_rif;
    float zp = z_ecef - z_ecef_rif;

    ROS_INFO("partial: xp %f, yp %f, zp %f\n", xp, yp, zp);

    float slat = sin(ToRad(lat));
    float clat = cos(ToRad(lat));
    float slon = sin(ToRad(lon));
    float clon = cos(ToRad(lon));
    float coords[3];

    coords[0] = -slon*xp + clon*yp;
    coords[1] = -slat*clon*xp - slat*slon*yp + clat*zp;
    coords[2] = clat*clon*xp + clat*slon*yp + slat*zp;

    ROS_INFO("ENU: x %f, y %f, z %f\n", coords[0], coords[1], coords[2]);
    
}



int main(int argc, char **argv){

    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n; //forse devo usare più di 1 handler per fare pub e sub in contemporanea

	n.getParam("lat_r", lat_r); //prendo i parametri dal launchfile
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
