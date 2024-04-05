#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/Num.h" //includo i custom messages

void chatterCallback(const custom_messages::Num::ConstPtr& msg){ //quando mi iscrivo al messaggio, devo specificare che il tipo è custom messages
  ROS_INFO("I heard: [%ld]", msg->num); //per stamparlo, devo accedere al field
}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  	ros::spin();

  return 0;
}


