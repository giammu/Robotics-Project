#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "param_first");
	//ho bisogno 2 nodehandle: 1 per vedere le cose globali e uno per vedere le cose private: (quelli privati possono comunque essere visti da chiunque, non sono davvero "privati")
	ros::NodeHandle n; // this is a global node handle //:ci permette di vedere tutti i parametres settati globalmente, e pubblica i topic con lo standard name
	ros::NodeHandle nh_private("~"); // this is a private node handle //:serve per vedere i parametri privati (es. distanza tra le ruote di un robot ecc...)

	//sintassi del publisher è la stessa: nel primo pubblichiamo globalmente, nel secondo pubblichiamo privatamente
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("global_parameter", 1000); // publish global topic, no node name //definiamo il publisher, global_parameter è il nome del topic che viene pubblicato sulla ros network
	ros::Publisher local_chatter_pub = nh_private.advertise<std_msgs::String>("local_parameter", 1000); // publish topic with node name //ogni topic che viene pubblicato con un private node handle: avrà prima il nome del nodo e poi il nome del topic: questo mi permette di identificare che un topic proviene da uno specifico nodo (altrimenti es. se avessi 2 robot e per entrambi volessi sapere la distanza tra le ruote, non riuscirei a distinguerli)
	
	std::string name;
	n.getParam("name", name);  //get global param  //la funzione chiede il parametro name e poi lo salva nella variabile name: questo lo metto fuori dal loop while perchè è un valore statico che chiedo al server e non voglio continuare a chiederglielo e generare traffico inutile
	
	std::string local_name;
	nh_private.getParam("name", local_name); //get local param //prendiamo il valore del parametro che si chiama name, però stiamo usando il private node handle, quindi ros sa che stiamo guardando all'interno del namespace
	
	std::string local_name_from_global;
	std::string param_name = ros::this_node::getName() + "/name"; //posso anche settare manualmente il parametro privato: combino il nome del nodo con il nome del parametro. Questo non è un buon approccio ma funziona
	ROS_INFO("local param name: %s", param_name.c_str());
	n.getParam(param_name, local_name_from_global);  //get local param using global nodehandle

	ros::Rate loop_rate(10);

  
  	while (ros::ok()){
	    
	    	std_msgs::String msg;
    		msg.data = name;
    		
    		std_msgs::String local_msg;
    		local_msg.data = local_name;

    		ROS_INFO("%s", local_name_from_global.c_str());

    		chatter_pub.publish(msg); //pubblico con il global node handle
    		local_chatter_pub.publish(local_msg); //pubblico con il private node handle

    		ros::spinOnce();

    		loop_rate.sleep();
    		
  	}


  	return 0;
}
