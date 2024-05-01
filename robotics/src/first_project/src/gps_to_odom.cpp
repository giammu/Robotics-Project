/*
Descrizione Primo Nodo: deve computare la odometry dai gps data

The node subscribe to fix and publish the odometry message:
    type: nav_msgs/Odometry
    topic name: gps_odom

Abbiamo latitude, longitude, altitude e li convertiamo in ECEF Cartesiano
Convertiamo ECEF a ENU
ENU è relativo: fornisco la starting position: di solito si mette la zero position del robot come la relative position

3 parametri del nodo: lat_r, lon_r, alt_r -> devo settarli manualmente nel launch file con il primo valore del gps

Il gps dà una posizione ma non l'orientation, devo computarla: ho multiple points (è la trajectory e il gps funziona a 10hz)
-> computo l'orientamento usando i punti consecutivi (la linea che passa tra i due punti) che mi sono trovato prima con ENU
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "cmath"
#include "typeinfo"
#include <sstream>


class pub_sub
{

        double lat_r;
        double lon_r;
        double alt_r;

        const double conversion = M_PI/180;

        nav_msgs::Odometry message; 

        double x_prev=0, y_prev=0, z_prev=0;

    private:

        ros::NodeHandle n; 
        ros::Subscriber sub;
        ros::Publisher pub;

    public: pub_sub(){

        n.getParam("lat_r", lat_r); //prendo i parametri dal launchfile
        n.getParam("lon_r", lon_r);
        n.getParam("alt_r", alt_r);
        
        //ROS_INFO("\n    lat_r: %f, lon_r: %f, alt_r: %f", lat_r, lon_r, alt_r );

        //Subscriber
        sub = n.subscribe("/fix", 1, &pub_sub::callback, this); //topic: fix, buffer: dimensione 1, il subscriber chiama la funzione
        //Publisher
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1); // type: nav_msgs/Odometry, topic: gps_odom, buffer: dimensione 1 (good practice)

    }

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){ 

        //ROS_INFO("\n header %d", msg->header.seq);
        //ROS_INFO("\n lat %f, lon %f, alt %f", msg->latitude, msg->longitude, msg->altitude);

        double lat = msg->latitude; 
        double lon = msg->longitude; 
        double alt = msg->altitude; 

        double lat_r_ecef = lat_r;
        double lon_r_ecef = lon_r;
        double alt_r_ecef = alt_r;

        //ROS_INFO("\n\n  GPS: x %f, y %f, z %f", lat, lon, alt);

        //cast to ECEF
        castToECEF(&lat, &lon, &alt); //così ottengo Xp Yp Zp
        castToECEF(&lat_r_ecef, &lon_r_ecef, &alt_r_ecef); //così ottengo Xr Yr Zr

        //ROS_INFO("\n    ECEF: x %f, y %f, z %f", lat, lon, alt);

        //cast to ENU
        castToENU(&lat, &lon, &alt, &lat_r_ecef, &lon_r_ecef, &alt_r_ecef);

        //ROS_INFO("\n    ENU: x %f, y %f, z %f", lat, lon, alt);

        //Ruoto le coordinate
        double x = lat*cos(-130*conversion)+lon*sin(-130*conversion);
        double y = -lat*sin(-130*conversion)+lon*cos(-130*conversion);
        double z = alt;

        //ROS_INFO("\n    COORD RUOTATE: x %f, y %f, z %f", x, y, z);

        //Computo la orientation (Quaternion: z e w)
        double zq;
        double w;
        orientation(x, y, z, x_prev, y_prev, z_prev, &zq, &w);

        //ROS_INFO("\n    QUATERNIONE: z %f, w %f", zq, w);

        //Publisher
        message.pose.pose.position.x = x;
        message.pose.pose.position.y = y;
        message.pose.pose.position.z = z;

        message.pose.pose.orientation.x = 0; //0 perchè il piano è 2D
        message.pose.pose.orientation.y = 0; //0 perchè il piano è 2D
        message.pose.pose.orientation.z = zq;
        message.pose.pose.orientation.w = w;

        pub.publish(message); //pubblico il message su gps_odom

        x_prev=x; //le coordinate correnti diventano le coordinate precedenti
        y_prev=y;
        z_prev=z;

    }

    void castToECEF(double* lat, double* lon, double* alt){

        // Conversione da gradi a radianti
        double lat_rad = *lat * conversion;
        double lon_rad = *lon * conversion;

        double a = 6378137.0;
        double b = 6356752.0;
        double e2 = 1 - (b*b)/(a*a);
        double n = a / sqrt(1.0 - e2 * sin(lat_rad) * sin(lat_rad));

        // Calcolo delle coordinate ECEF
        double x = (n + *alt) * cos(lat_rad) * cos(lon_rad);
        double y = (n + *alt) * cos(lat_rad) * sin(lon_rad);
        double z = (n * (1.0 - e2) + *alt) * sin(lat_rad);

        // Assegnazione delle coordinate ECEF alle variabili di output
        *lat = x;
        *lon = y;
        *alt = z;
    }

    void castToENU(double* Xp, double* Yp, double* Zp, double* Xr, double* Yr, double* Zr){

        //Conversione da gradi a radianti
        double lat_rad = lat_r * conversion;
        double lon_rad = lon_r * conversion;

        //calcolo seni e coseni delle lat e long di riferimento
        double slat = sin(lat_rad);
        double clat = cos(lat_rad);
        double slon = sin(lon_rad);
        double clon = cos(lon_rad);

        //calcolo la differenza
        double dx = *Xp - *Xr;
        double dy = *Yp - *Yr;
        double dz = *Zp - *Zr;

        //calcolo il prodotto matriciale
        double x = -slon*dx + clon*dy;
        double y = -slat*clon*dx - slat*slon*dy + clat*dz;
        double z = clat*clon*dx + clat*slon*dy + slat*dz;

        //Assegno i valori
        *Xp=x;
        *Yp=y;
        *Zp=z;
    }

    void orientation(double x, double y, double z, double x_prev, double y_prev, double z_prev, double* zq, double* w){
        // Calcola il vettore di spostamento tra due punti consecutivi
        double delta_x = x - x_prev;
        double delta_y = y - y_prev;
        double delta_z = z - z_prev;

        // Normalizza il vettore per ottenere le direzioni
        double norm = std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
        double dir_x = delta_x / norm;
        double dir_y = delta_y / norm;

        // Calcola l'angolo tra la direzione iniziale e quella corrente   
        double angle = std::atan2(dir_y, dir_x);

        // Calcola i componenti zq e w del quaternion
        double half_angle = angle / 2.0;
        double z_quat = std::sin(half_angle);
        double w_quat = std::cos(half_angle);

        *zq = z_quat;
        *w = w_quat;
    }

};

int main(int argc, char **argv){

    ros::init(argc, argv, "gps_to_odom");
    pub_sub my_pub_sub;
    ros::spin();
 	return 0;    
}
