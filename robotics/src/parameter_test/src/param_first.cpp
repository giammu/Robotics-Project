#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "param_first");
	ros::NodeHandle n; // this is a global node handle
	ros::NodeHandle nh_private("~"); // this is a private node handle

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("global_parameter", 1000); // publish global topic, no node name //definiamo il publisher
	ros::Publisher local_chatter_pub = nh_private.advertise<std_msgs::String>("local_parameter", 1000); // publish topic with node name
	
	std::string name;
	n.getParam("name", name);  //get global param  //la funzione chiede il parametro name e poi lo salva nella variabile name: questo lo metto fuori dal loop while perchè è un valore statico che chiedo al server e non voglio continuare a chiederglielo e generare traffico inutile
	
	std::string local_name;
	nh_private.getParam("name", local_name); //get local param
	
	std::string local_name_from_global;
	std::string param_name = ros::this_node::getName() + "/name"; 
	ROS_INFO("local param name: %s", param_name.c_str());
	n.getParam(param_name, local_name_from_global);  //get local param using global nodehandle

	ros::Rate loop_rate(10);

  
  	while (ros::ok()){
	    
	    	std_msgs::String msg;
    		msg.data = name;
    		
    		std_msgs::String local_msg;
    		local_msg.data = local_name;

    		ROS_INFO("%s", local_name_from_global.c_str());

    		chatter_pub.publish(msg);
    		local_chatter_pub.publish(local_msg);

    		ros::spinOnce();

    		loop_rate.sleep();
    		
  	}


  	return 0;
}
