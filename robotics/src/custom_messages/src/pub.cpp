#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/Num.h" // aggiungo i custom messages: name del package/name del custom message

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<custom_messages::Num>("chatter", 1000); //creiamo un publisher dove specifichiamo il tipo di custom message

	ros::Rate loop_rate(10);

	int count = 0;
  
  
  	while (ros::ok()){
	    
	    	static int i=0;
		i=(i+1)%1000;
		custom_messages::Num msg; //creiamo il custom message: il tipo è custom message, num è il nome
		msg.num =i; //setto il field al valore che voglio pubblicare
		chatter_pub.publish (msg); //alla fine lo pubblico

  	}


  	return 0;
}
