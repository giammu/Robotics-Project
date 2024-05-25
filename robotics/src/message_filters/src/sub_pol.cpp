#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


void callback(const geometry_msgs::Vector3StampedConstPtr& msg1, const geometry_msgs::Vector3StampedConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y, msg2->vector.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;

  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "topic1", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(n, "topic2", 1);

  //Posso definire 2 tipi di policy:
  //1) Exact time è la policy che avevamo già prima
  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  //2) Approximate time policy
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy; //questo specifica una policy per sincronizzare i data: possiamo avere Approximate time  o ExactTime: spesso è meglio usare la Approximate time, gli scenari in cui si usa ExactTIme sono pochi: es. due fotocamere
  
  //Quando creo il synchronizer, gli passo la policy, la size del buffer, poi gli passo i due subscriber
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2); 
  sync.registerCallback(boost::bind(&callback, _1, _2)); // chiamo la callback come visto prima

  ros::spin();

  return 0;
}


