//questo è come si crea un timer

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <time.h>




void timerCallback(const ros::TimerEvent& ev){ //questa callback semplicemente stampa un messaggio

	ROS_INFO_STREAM("Callback called at time: " <<  ros::Time::now()); //ros-info-stream è una alternativa al cout: ho una stringa e concateno il timestamp. now() ci fornisce il time dentro a ros, che di solito è lo stesso del mio sistema (tranne quando sto runnando delle simulazioni)

}

int main(int argc, char **argv){
    
	ros::init(argc, argv, "timed_talker");
	ros::NodeHandle n;

	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback); //creo il timer object: durata 0.1 significa che il timer sta runnando a 10hz, la callback funziona quando il timer è elapsed
	
	ros::spin();

  	return 0;
}
