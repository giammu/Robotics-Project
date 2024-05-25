#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;

	ros::Publisher pub1 = n.advertise<geometry_msgs::Vector3Stamped>("topic1", 1000); //Generiamo 2 publisher che pubblichiamo i due topics
	ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3Stamped>("topic2", 1000);

	ros::Rate loop_rate(1); //pubblica a 1hz

	int count = 0;
  
  
  	while (ros::ok()){
	    
		geometry_msgs::Vector3Stamped msg1; //ci serve usare il Vector3Stamped per avere il timestamp
		geometry_msgs::Vector3Stamped msg2;
			//Ora settiamo i 2 messaggi
		msg1.header.stamp = ros::Time::now(); //il message filter ha bisogno del timestamp nell'header per funzionare (in cpp) [in python funziona anche senza header], per usare messaggi che non hanno un header, devo aggiungere un header
		msg1.header.frame_id = "f1"; //inseriamo dei valori nel messaggio
		msg1.vector.x = 1;
		msg1.vector.y = 1;
		msg1.vector.z = 1;
		
		
		msg2.header.stamp = ros::Time::now(); // la soluzione al problema del timestamp diverso è mettere qui = msg1.header.stamp
		msg2.header.frame_id = "f2";
		msg2.vector.x = 2;
		msg2.vector.y = 2;
		msg2.vector.z = 2;



    		pub1.publish(msg1); //pubblichiamo i messaggi
    		pub2.publish(msg2);
    		ROS_INFO ("Publishing message");

    		ros::spinOnce();

    		loop_rate.sleep();
    		++count;
  	}


  	return 0;
}
