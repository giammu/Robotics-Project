
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



void callback(const geometry_msgs::Vector3StampedConstPtr& msg1, const geometry_msgs::Vector3StampedConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y, msg2->vector.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "topic1", 1); //creiamo i message filters subscribers (non sono standard subscriber)
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(n, "topic2", 1);
  message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> sync(sub1, sub2, 10); //creiamo il time synchronizer: gli passiamo il type dei message da sincronizzare, poi gli passiamo i subscriber, 10 è la size del buffer
  sync.registerCallback(boost::bind(&callback, _1, _2)); //faccio partire il synchronizer settando la callback, _1 e _2 sono 2 placeholder, sono i due messaggi che ricevo dai due topic a cui mi iscrivo (se avessi avuto più subscriber avrei fatto più placeholder)

  ros::spin();

  return 0;
}


