#include "ros/ros.h"
#include "std_msgs/String.h"


class pub_sub //uso le classi per definire tutti gli elementi diversi (è più comodo per i progetti complessi)
{

std_msgs::String messagio; //questi sono i 2 messaggi ricevuti dai 2 nodi
std_msgs::String messagio2;

private: //definisco 2 subscriber, 1 publisher, 1 timer
ros::NodeHandle n; 

ros::Subscriber sub;
ros::Subscriber sub2;
ros::Publisher pub; 
ros::Timer timer1;
	
	
public:
  	pub_sub(){ //inizializzo gli elementi
  	sub = n.subscribe("/chatter", 1, &pub_sub::callback, this); //subscriber, ha una callback
	sub2 = n.subscribe("/chatter2", 1, &pub_sub::callback2, this); //subscriber, ha una seconda callback
	pub = n.advertise<std_msgs::String>("/rechatter", 1); //publisher
	timer1 = n.createTimer(ros::Duration(1), &pub_sub::callback1, this); //timer callback funziona a 1hz
	

}
void callback(const std_msgs::String::ConstPtr& msg){ //definiamo la prima callback: che prende il messaggio e lo copia
messagio=*msg;

}

void callback2(const std_msgs::String::ConstPtr& msg){ //stessa cosa
messagio2=*msg;

}

void callback1(const ros::TimerEvent&) //pubblica primo e secondo messaggio
{
	pub.publish(messagio);
	pub.publish(messagio2);
  	ROS_INFO("Callback 1 triggered");
}





};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish"); //per prima cosa inizializzo il nodo
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
