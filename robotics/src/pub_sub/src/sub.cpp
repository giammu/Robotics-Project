#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg){ //specifico i messaggi che ricevo: sono iscritto a 1 solo topic in questo caso, good practice che sia constant il std_msg
  ROS_INFO("I heard: [%s]", msg->data.c_str()); //dentro alla callback facciamo il processing dei data: è un puntatore e quindi abbiamo la freccia
}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener"); //inizializzo il nodo con il name "listener"

	ros::NodeHandle n; 
  	ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback); //creo il subscriber object: chatter è il topic a cui mi iscrivo (era lo stesso del publisher), la dimensione del buffer dove venogno salvati i dati è 1, il subscriber aspetta la chattercallback

  	ros::spin(); //continua ad andare il più velocemente possibile, ad ogni spin controlla se il topic è arrivato, e se è arrivato chiama la callback relativa

  return 0;
}


