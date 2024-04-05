//questo file serve solo per generare i dati

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>  

#include <sstream>


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "publisher");


	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); //abbiamo 2 publisher che pubblicano su chatter e chatter2
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::String>("chatter2", 1000);

	ros::Rate loop_rate(10);

	
	int count = 0;
	while (ros::ok())
	{
		
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		if (rand() % 10 <6){ //con questo modo le stringhe vengono pubblicate non esattamente a 10hz ma ad una frequenza diversa e randomica
			chatter_pub.publish(msg);
		}
		if (rand() % 10 <2){
			chatter_pub2.publish (msg);
		}
		

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
